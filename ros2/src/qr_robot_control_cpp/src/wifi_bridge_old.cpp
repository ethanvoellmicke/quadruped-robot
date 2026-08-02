#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

class WifiBridge : public rclcpp::Node
{
public:
  WifiBridge()
  : Node("wifi_bridge")
  {
    // These parameters can be changed from the terminal
    // without recompiling.
    nano_ip_ = this->declare_parameter<std::string>(
      "nano_ip",
      "172.20.10.7"
    );

    nano_port_ = this->declare_parameter<int>(
      "nano_port",
      5005
    );

    createUdpSocket();

    subscription_ =
      this->create_subscription<sensor_msgs::msg::JointState>(
        "/servo_joint_commands",
        10,
        std::bind(
          &WifiBridge::jointStateCallback,
          this,
          std::placeholders::_1
        )
      );

    RCLCPP_INFO(
      this->get_logger(),
      "Wi-Fi bridge sending 12 absolute servo angles to %s:%d",
      nano_ip_.c_str(),
      nano_port_
    );
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

  // Your knee linkage is close to linear:
  //
  // desired physical knee movement
  //     × 1.08
  // required actuator movement
  //
  // The per-servo direction below determines whether
  // that movement is added to or subtracted from neutral.
  static constexpr double KNEE_LINKAGE_RATIO = 1.08;

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

  // This array order must match:
  //
  // 0–2   Front Left
  // 3–5   Front Right
  // 6–8   Back Left
  // 9–11  Back Right
  //
  // Each leg:
  // Shoulder, Hip, Knee
  const std::array<std::string, NUM_SERVOS> joint_names_ =
  {
    "FrontLeftJoint1",
    "FrontLeftJoint2",
    "FrontLeftJoint3",

    "FrontRightJoint1",
    "FrontRightJoint2",
    "FrontRightJoint3",

    "BackLeftJoint1",
    "BackLeftJoint2",
    "BackLeftJoint3",

    "BackRightJoint1",
    "BackRightJoint2",
    "BackRightJoint3"
  };

  // All hardware-specific translation now lives here.
  //
  // The Arduino should receive only the final absolute
  // servo angles produced from this table.
  const std::array<ServoCalibration, NUM_SERVOS>
  servo_calibrations_ =
  {{
    // ---------------------------------------------------
    // Front Left
    // ---------------------------------------------------
// Front Left
{
  "Front Left Shoulder",
  60.0,
  1.0,
  -17.0,
  20.0,
  43.0,
  80.0,
  JointType::Shoulder
},

{
  "Front Left Hip",
  95.0,
  1.0,
  -36.0,
  24.0,
  71.0,
  131.0,
  JointType::Hip
},

{
  "Front Left Knee",
  107.0,
  1.0,
  -25.0,
  25.0,
  82.0,
  135.0,
  JointType::Knee
},

// Front Right
{
  "Front Right Shoulder",
  91.0,
  1.0,
  -17.0,
  20.0,
  74.0,
  111.0,
  JointType::Shoulder
},

{
  "Front Right Hip",
  85.0,
  1.0,
  -36.0,
  24.0,
  49.0,
  109.0,
  JointType::Hip
},

{
  "Front Right Knee",
  107.0,
  1.0,
  -25.0,
  25.0,
  79.0,
  132.0,
  JointType::Knee
},

// Back Left
{
  "Back Left Shoulder",
  68.0,
  1.0,
  -17.0,
  20.0,
  51.0,
  88.0,
  JointType::Shoulder
},

{
  "Back Left Hip",
  88.0,
  1.0,
  -36.0,
  24.0,
  64.0,
  124.0,
  JointType::Hip
},

{
  "Back Left Knee",
  85.0,
  1.0,
  -25.0,
  25.0,
  60.0,
  113.0,
  JointType::Knee
},

// Back Right
{
  "Back Right Shoulder",
  108.0,
  1.0,
  -17.0,
  20.0,
  91.0,
  128.0,
  JointType::Shoulder
},

{
  "Back Right Hip",
  98.0,
  1.0,
  -36.0,
  24.0,
  62.0,
  122.0,
  JointType::Hip
},

{
  "Back Right Knee",
  135.0,
  1.0,
  -25.0,
  25.0,
  110.0,
  160.0,
  JointType::Knee}
}};

  void createUdpSocket()
  {
    socket_fd_ = socket(
      AF_INET,
      SOCK_DGRAM,
      0
    );

    if (socket_fd_ < 0)
    {
      throw std::runtime_error(
        std::string("Could not create UDP socket: ") +
        std::strerror(errno)
      );
    }

    nano_address_ = {};

    nano_address_.sin_family = AF_INET;

    nano_address_.sin_port = htons(
      static_cast<uint16_t>(nano_port_)
    );

    const int address_result = inet_pton(
      AF_INET,
      nano_ip_.c_str(),
      &nano_address_.sin_addr
    );

    if (address_result != 1)
    {
      close(socket_fd_);
      socket_fd_ = -1;

      throw std::runtime_error(
        "Invalid Nano IP address: " + nano_ip_
      );
    }
  }

  static double radiansToDegrees(double radians)
  {
    return radians * 180.0 / PI;
  }

  static double linkageRatio(
    JointType joint_type)
  {
    if (joint_type == JointType::Knee)
    {
      return KNEE_LINKAGE_RATIO;
    }

    return 1.0;
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
        calibration.name
      );
    }

    const double requested_joint_deg =
      radiansToDegrees(joint_angle_rad);

    // First clamp the desired physical robot-joint angle.
    const double safe_joint_deg =
      std::clamp(
        requested_joint_deg,
        calibration.joint_min_deg,
        calibration.joint_max_deg
      );

    // Compensate for the knee linkage.
    //
    // Shoulders and hips:
    // actuator offset = joint angle
    //
    // Knees:
    // actuator offset = joint angle × 1.08
    const double actuator_offset_deg =
      safe_joint_deg *
      linkageRatio(calibration.joint_type);

    // Convert the robot-joint command into the final
    // absolute physical servo angle.
    const double unclamped_servo_deg =
      calibration.neutral_deg +
      calibration.direction *
      actuator_offset_deg;

    // Final physical servo clamp.
    const double safe_servo_deg =
      std::clamp(
        unclamped_servo_deg,
        calibration.servo_min_deg,
        calibration.servo_max_deg
      );

    if (
      std::abs(requested_joint_deg - safe_joint_deg) >
      0.001
    )
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "%s joint command clamped: %.2f deg -> %.2f deg",
        calibration.name,
        requested_joint_deg,
        safe_joint_deg
      );
    }

    if (
      std::abs(unclamped_servo_deg - safe_servo_deg) >
      0.001
    )
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "%s servo command clamped: %.2f deg -> %.2f deg",
        calibration.name,
        unclamped_servo_deg,
        safe_servo_deg
      );
    }

RCLCPP_INFO(
    this->get_logger(),
    "%s -> %.1f",
    calibration.name,
    safe_servo_deg
);

    return safe_servo_deg;
  }

  void jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.size() != msg->position.size())
    {
      RCLCPP_WARN(
        this->get_logger(),
        "JointState name and position arrays have different sizes"
      );

      return;
    }

    // Match every ROS joint name with its corresponding
    // robot-joint angle in radians.
    std::unordered_map<std::string, double> positions;

    positions.reserve(msg->name.size());

    for (std::size_t i = 0; i < msg->name.size(); ++i)
    {
      positions[msg->name[i]] =
        msg->position[i];
    }

    // These are final physical servo positions in degrees.
    std::array<double, NUM_SERVOS> servo_angles_deg{};

    for (
      std::size_t i = 0;
      i < joint_names_.size();
      ++i
    )
    {
      const auto joint =
        positions.find(joint_names_[i]);

      if (joint == positions.end())
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Waiting for joint: %s",
          joint_names_[i].c_str()
        );

        return;
      }

      try
      {
        servo_angles_deg[i] =
          jointToAbsoluteServoAngle(
            i,
            joint->second
          );
      }
      catch (const std::exception & error)
      {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Could not map servo command: %s",
          error.what()
        );

        return;
      }
    }

    sendServoAngles(servo_angles_deg);
  }

  void sendServoAngles(
    const std::array<double, NUM_SERVOS> &
    servo_angles_deg)
  {
    std::ostringstream packet;

    packet << '<';
    packet << std::fixed;
    packet << std::setprecision(2);

    for (
      std::size_t i = 0;
      i < servo_angles_deg.size();
      ++i
    )
    {
      packet << servo_angles_deg[i];

      if (i < servo_angles_deg.size() - 1)
      {
        packet << ',';
      }
    }

    packet << '>';

    const std::string message =
      packet.str();

    const ssize_t bytes_sent =
      sendto(
        socket_fd_,
        message.c_str(),
        message.size(),
        0,
        reinterpret_cast<sockaddr *>(
          &nano_address_
        ),
        sizeof(nano_address_)
      );

    if (bytes_sent < 0)
    {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "UDP send failed: %s",
        std::strerror(errno)
      );

      return;
    }

    if (
      static_cast<std::size_t>(bytes_sent) !=
      message.size()
    )
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "UDP packet was not completely sent"
      );

      return;
    }

    RCLCPP_DEBUG(
      this->get_logger(),
      "Sent absolute servo angles: %s",
      message.c_str()
    );
  }

  std::string nano_ip_;
  int nano_port_;

  int socket_fd_ = -1;

  sockaddr_in nano_address_{};

  rclcpp::Subscription<
    sensor_msgs::msg::JointState>::SharedPtr
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
      error.what()
    );
  }

  rclcpp::shutdown();

  return 0;
}

/*
Terminal 1:

ros2 run qr_robot_control_cpp wifi_bridge \
  --ros-args \
  -p nano_ip:=172.20.10.7 \
  -p nano_port:=5005


Terminal 2:

ros2 run qr_robot_control_cpp stand_pose
*/