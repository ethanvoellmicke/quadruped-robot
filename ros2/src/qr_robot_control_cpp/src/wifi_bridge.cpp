#include <rclcpp/rclcpp.hpp>
#include <qr_robot_msgs/msg/servo_trajectory.hpp>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

class WifiBridge : public rclcpp::Node
{
public:
  WifiBridge()
  : Node("wifi_bridge")
  {
    nano_ip_ = this->declare_parameter<std::string>(
      "nano_ip",
      "172.20.10.7");

    nano_port_ = this->declare_parameter<int>(
      "nano_port",
      5005);

    send_duplicate_packets_ = this->declare_parameter<bool>(
      "send_duplicate_packets",
      true);//write true if turning back on

    createUdpSocket();

    subscription_ =
      this->create_subscription<qr_robot_msgs::msg::ServoTrajectory>(
        "/servo_trajectory",
        10,
        std::bind(
          &WifiBridge::trajectoryCallback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Wi-Fi bridge receiving future frames from /servo_trajectory");

    RCLCPP_INFO(
      this->get_logger(),
      "Sending binary trajectory packets to %s:%d",
      nano_ip_.c_str(),
      nano_port_);

    RCLCPP_INFO(
      this->get_logger(),
      "Maximum frames per UDP packet: %u",
      static_cast<unsigned int>(MAX_FRAMES_PER_PACKET));
  }

  ~WifiBridge() override
  {
    if (socket_fd_ >= 0)
    {
      close(socket_fd_);
    }
  }

private:
  static constexpr double PI =
    3.14159265358979323846;

  static constexpr std::size_t NUM_SERVOS = 12;
  static constexpr double KNEE_LINKAGE_RATIO = 1.08;

  // This matches the binary trajectory protocol already planned
  // for the Arduino trajectory buffer.
  static constexpr uint32_t PACKET_MAGIC =
    0x31545251UL;

  static constexpr uint8_t PROTOCOL_VERSION = 1;
  static constexpr uint8_t MAX_FRAMES_PER_PACKET = 5;
  static constexpr std::size_t PACKET_HEADER_SIZE = 14;
  static constexpr std::size_t CRC_SIZE = 2;

  enum class JointType
  {
    Shoulder,
    Hip,
    Knee
  };

  struct ServoCalibration
  {
    const char * name;

    double neutral_deg;
    double direction;

    double joint_min_deg;
    double joint_max_deg;

    double servo_min_deg;
    double servo_max_deg;

    JointType joint_type;
  };

  // Servo order:
  //
  // 0-2   Front Left:  shoulder, hip, knee
  // 3-5   Front Right: shoulder, hip, knee
  // 6-8   Back Left:   shoulder, hip, knee
  // 9-11  Back Right:  shoulder, hip, knee
  const std::array<ServoCalibration, NUM_SERVOS>
  servo_calibrations_ =
  {{
    {
      "Front Left Shoulder",
      60.0, 1.0,
      -17.0, 20.0,
      43.0, 80.0,
      JointType::Shoulder
    },
    {
      "Front Left Hip",
      95.0, 1.0,
      -36.0, 24.0,
      71.0, 131.0,
      JointType::Hip
    },
    {
      "Front Left Knee",
      107.0, 1.0,
      -25.0, 25.0,
      82.0, 135.0,
      JointType::Knee
    },

    {
      "Front Right Shoulder",
      91.0, 1.0,
      -17.0, 20.0,
      74.0, 111.0,
      JointType::Shoulder
    },
    {
      "Front Right Hip",
      85.0, 1.0,
      -36.0, 24.0,
      49.0, 109.0,
      JointType::Hip
    },
    {
      "Front Right Knee",
      107.0, 1.0,
      -25.0, 25.0,
      79.0, 132.0,
      JointType::Knee
    },

    {
      "Back Left Shoulder",
      68.0, 1.0,
      -17.0, 20.0,
      51.0, 88.0,
      JointType::Shoulder
    },
    {
      "Back Left Hip",
      88.0, 1.0,
      -36.0, 24.0,
      64.0, 124.0,
      JointType::Hip
    },
    {
      "Back Left Knee",
      85.0, 1.0,
      -25.0, 25.0,
      60.0, 113.0,
      JointType::Knee
    },

    {
      "Back Right Shoulder",
      108.0, 1.0,
      -17.0, 20.0,
      91.0, 128.0,
      JointType::Shoulder
    },
    {
      "Back Right Hip",
      98.0, 1.0,
      -36.0, 24.0,
      62.0, 122.0,
      JointType::Hip
    },
    {
      "Back Right Knee",
      135.0, 1.0,
      -25.0, 25.0,
      110.0, 160.0,
      JointType::Knee
    }
  }};

  void createUdpSocket()
  {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

    if (socket_fd_ < 0)
    {
      throw std::runtime_error(
        std::string("Could not create UDP socket: ") +
        std::strerror(errno));
    }

    nano_address_ = {};
    nano_address_.sin_family = AF_INET;
    nano_address_.sin_port =
      htons(static_cast<uint16_t>(nano_port_));

    const int address_result = inet_pton(
      AF_INET,
      nano_ip_.c_str(),
      &nano_address_.sin_addr);

    if (address_result != 1)
    {
      close(socket_fd_);
      socket_fd_ = -1;

      throw std::runtime_error(
        "Invalid Nano IP address: " + nano_ip_);
    }
  }

  static double radiansToDegrees(double radians)
  {
    return radians * 180.0 / PI;
  }

  static double linkageRatio(JointType joint_type)
  {
    return joint_type == JointType::Knee ?
      KNEE_LINKAGE_RATIO : 1.0;
  }

  double jointToAbsoluteServoAngle(
    std::size_t servo_index,
    double joint_angle_rad) const
  {
    const ServoCalibration & calibration =
      servo_calibrations_.at(servo_index);

    if (!std::isfinite(joint_angle_rad))
    {
      throw std::runtime_error(
        std::string("Non-finite joint angle for ") +
        calibration.name);
    }

    const double requested_joint_deg =
      radiansToDegrees(joint_angle_rad);

    const double safe_joint_deg =
      std::clamp(
        requested_joint_deg,
        calibration.joint_min_deg,
        calibration.joint_max_deg);

    const double actuator_offset_deg =
      safe_joint_deg *
      linkageRatio(calibration.joint_type);

    const double unclamped_servo_deg =
      calibration.neutral_deg +
      calibration.direction *
      actuator_offset_deg;

    const double safe_servo_deg =
      std::clamp(
        unclamped_servo_deg,
        calibration.servo_min_deg,
        calibration.servo_max_deg);

    if (std::abs(requested_joint_deg - safe_joint_deg) > 0.001)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "%s joint command clamped: %.2f deg -> %.2f deg",
        calibration.name,
        requested_joint_deg,
        safe_joint_deg);
    }

    if (std::abs(unclamped_servo_deg - safe_servo_deg) > 0.001)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "%s servo command clamped: %.2f deg -> %.2f deg",
        calibration.name,
        unclamped_servo_deg,
        safe_servo_deg);
    }

    return safe_servo_deg;
  }

  std::array<uint16_t, NUM_SERVOS> encodeFrame(
    const qr_robot_msgs::msg::ServoFrame & frame) const
  {
    std::array<uint16_t, NUM_SERVOS> encoded_frame{};

    for (std::size_t servo_index = 0;
         servo_index < NUM_SERVOS;
         ++servo_index)
    {
      const double absolute_servo_deg =
        jointToAbsoluteServoAngle(
          servo_index,
          static_cast<double>(
            frame.servo_angles[servo_index]));

      const long angle_x100 =
        std::lround(absolute_servo_deg * 100.0);

      encoded_frame[servo_index] =
        static_cast<uint16_t>(
          std::clamp<long>(
            angle_x100,
            0,
            18000));
    }

    return encoded_frame;
  }

  void trajectoryCallback(
    const qr_robot_msgs::msg::ServoTrajectory::SharedPtr msg)
  {
    if (msg->servo_count != NUM_SERVOS)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Expected %zu servos, but trajectory says %u",
        NUM_SERVOS,
        static_cast<unsigned int>(msg->servo_count));

      return;
    }

    if (msg->frame_period_ms == 0)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Received trajectory with frame_period_ms = 0");

      return;
    }

    if (msg->frames.empty())
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Received trajectory with no frames");

      return;
    }

    std::vector<std::array<uint16_t, NUM_SERVOS>>
      encoded_frames;

    encoded_frames.reserve(msg->frames.size());

    try
    {
      for (const auto & frame : msg->frames)
      {
        encoded_frames.push_back(encodeFrame(frame));
      }
    }
    catch (const std::exception & error)
    {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Could not convert trajectory: %s",
        error.what());

      return;
    }

    // UDP packets carry up to five future frames each.
    // A longer ROS trajectory is split into consecutive packets.
    std::size_t frame_offset = 0;

    while (frame_offset < encoded_frames.size())
    {
      const std::size_t remaining_frames =
        encoded_frames.size() - frame_offset;

      const uint8_t frames_in_packet =
        static_cast<uint8_t>(
          std::min<std::size_t>(
            remaining_frames,
            MAX_FRAMES_PER_PACKET));

      sendTrajectoryPacket(
        msg->first_frame_id +
          static_cast<uint32_t>(frame_offset),
        msg->frame_period_ms,
        encoded_frames,
        frame_offset,
        frames_in_packet);

      frame_offset += frames_in_packet;
    }

    trajectories_received_++;
    frames_forwarded_ +=
      static_cast<uint32_t>(encoded_frames.size());

    RCLCPP_DEBUG(
      this->get_logger(),
      "Forwarded trajectory: first frame=%u, frames=%zu, period=%u ms",
      msg->first_frame_id,
      encoded_frames.size(),
      static_cast<unsigned int>(msg->frame_period_ms));
  }

  static void appendUint16LittleEndian(
    std::vector<uint8_t> & packet,
    uint16_t value)
  {
    packet.push_back(
      static_cast<uint8_t>(value & 0xFF));

    packet.push_back(
      static_cast<uint8_t>((value >> 8) & 0xFF));
  }

  static void appendUint32LittleEndian(
    std::vector<uint8_t> & packet,
    uint32_t value)
  {
    packet.push_back(
      static_cast<uint8_t>(value & 0xFF));

    packet.push_back(
      static_cast<uint8_t>((value >> 8) & 0xFF));

    packet.push_back(
      static_cast<uint8_t>((value >> 16) & 0xFF));

    packet.push_back(
      static_cast<uint8_t>((value >> 24) & 0xFF));
  }

  static uint16_t calculateCrc16(
    const uint8_t * data,
    std::size_t length)
  {
    uint16_t crc = 0xFFFF;

    for (std::size_t byte_index = 0;
         byte_index < length;
         ++byte_index)
    {
      crc ^=
        static_cast<uint16_t>(data[byte_index]) << 8;

      for (uint8_t bit = 0; bit < 8; ++bit)
      {
        if ((crc & 0x8000) != 0)
        {
          crc = static_cast<uint16_t>(
            (crc << 1) ^ 0x1021);
        }
        else
        {
          crc = static_cast<uint16_t>(crc << 1);
        }
      }
    }

    return crc;
  }

  void sendTrajectoryPacket(
    uint32_t first_frame_id,
    uint16_t frame_period_ms,
    const std::vector<
      std::array<uint16_t, NUM_SERVOS>> & frames,
    std::size_t start_index,
    uint8_t frame_count)
  {
    const std::size_t packet_size =
      PACKET_HEADER_SIZE +
      static_cast<std::size_t>(frame_count) *
        NUM_SERVOS *
        sizeof(uint16_t) +
      CRC_SIZE;

    std::vector<uint8_t> packet;
    packet.reserve(packet_size);

    appendUint32LittleEndian(packet, PACKET_MAGIC);

    packet.push_back(PROTOCOL_VERSION);
    packet.push_back(frame_count);
    packet.push_back(static_cast<uint8_t>(NUM_SERVOS));

    // Reserved byte for future protocol flags.
    packet.push_back(0);

    appendUint16LittleEndian(
      packet,
      frame_period_ms);

    appendUint32LittleEndian(
      packet,
      first_frame_id);

    for (std::size_t frame_index = 0;
         frame_index < frame_count;
         ++frame_index)
    {
      const auto & frame =
        frames[start_index + frame_index];

      for (const uint16_t angle_x100 : frame)
      {
        appendUint16LittleEndian(
          packet,
          angle_x100);
      }
    }

    const uint16_t crc =
      calculateCrc16(
        packet.data(),
        packet.size());

    appendUint16LittleEndian(packet, crc);

    if (packet.size() != packet_size)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Internal packet-size error: built %zu bytes, expected %zu",
        packet.size(),
        packet_size);

      return;
    }

    sendRawPacket(packet);

    if (send_duplicate_packets_)
    {
      sendRawPacket(packet);
    }

    packets_generated_++;
  }

  void sendRawPacket(
    const std::vector<uint8_t> & packet)
  {
    const ssize_t bytes_sent =
      sendto(
        socket_fd_,
        packet.data(),
        packet.size(),
        0,
        reinterpret_cast<const sockaddr *>(
          &nano_address_),
        sizeof(nano_address_));

    if (bytes_sent < 0)
    {
      udp_send_errors_++;

      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "UDP send failed: %s",
        std::strerror(errno));

      return;
    }

    if (static_cast<std::size_t>(bytes_sent) !=
        packet.size())
    {
      udp_send_errors_++;

      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "UDP packet was not completely sent");

      return;
    }

    datagrams_sent_++;
  }

  std::string nano_ip_;
  int nano_port_;
  bool send_duplicate_packets_;

  int socket_fd_ = -1;
  sockaddr_in nano_address_{};

  uint32_t trajectories_received_ = 0;
  uint32_t packets_generated_ = 0;
  uint32_t datagrams_sent_ = 0;
  uint32_t frames_forwarded_ = 0;
  uint32_t udp_send_errors_ = 0;

  rclcpp::Subscription<
    qr_robot_msgs::msg::ServoTrajectory>::SharedPtr
    subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try
  {
    auto node =
      std::make_shared<WifiBridge>();

    rclcpp::spin(node);
  }
  catch (const std::exception & error)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("wifi_bridge"),
      "Wi-Fi bridge failed: %s",
      error.what());
  }

  rclcpp::shutdown();
  return 0;
}