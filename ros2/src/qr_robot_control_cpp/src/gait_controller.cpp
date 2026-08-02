#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#include "qr_robot_msgs/msg/servo_frame.hpp"
#include "qr_robot_msgs/msg/servo_trajectory.hpp"

#include "qr_robot_control_cpp/foot_trajectory.hpp"
#include "qr_robot_control_cpp/leg_ik.hpp"

class GaitController : public rclcpp::Node
{
public:
    GaitController()
        : Node("gait_controller")
    {
        // Current absolute joint pose for RViz.
        joint_publisher_ =
            create_publisher<sensor_msgs::msg::JointState>(
                "/joint_states",
                10
            );

        // A rolling window of future relative servo commands.
        trajectory_publisher_ =
            create_publisher<qr_robot_msgs::msg::ServoTrajectory>(
                "/servo_trajectory",
                10
            );

        gait_subscription_ =
            create_subscription<std_msgs::msg::String>(
                "/gait_cmd",
                10,
                std::bind(
                    &GaitController::gaitCallback,
                    this,
                    std::placeholders::_1
                )
            );

        speed_subscription_ =
            create_subscription<std_msgs::msg::Float32>(
                "/speed_cmd",
                10,
                std::bind(
                    &GaitController::speedCallback,
                    this,
                    std::placeholders::_1
                )
            );

        height_subscription_ =
            create_subscription<std_msgs::msg::Float32>(
                "/height_cmd",
                10,
                std::bind(
                    &GaitController::heightCallback,
                    this,
                    std::placeholders::_1
                )
            );

        timer_ = create_wall_timer(
            std::chrono::milliseconds(FRAME_PERIOD_MS),
            std::bind(&GaitController::update, this)
        );

        RCLCPP_INFO(
            get_logger(),
            "Gait controller started: %zu future frames at %u ms per frame.",
            FUTURE_FRAME_COUNT,
            FRAME_PERIOD_MS
        );
//new
        const LegAngles neutral =
    solveLegIK(
        NEUTRAL_FOOT_X,
        z_ground_
    );

RCLCPP_INFO(
    get_logger(),
    "Neutral IK -> Hip: %.2f deg   Knee: %.2f deg",
    neutral.hip * 180.0 / PI,
    neutral.knee * 180.0 / PI
);

    }

private:
    static constexpr double PI = 3.14159265358979323846;

    static constexpr std::uint16_t FRAME_PERIOD_MS = 20;
    static constexpr double TIME_STEP =
        static_cast<double>(FRAME_PERIOD_MS) / 1000.0;

    // Change this one value to alter the trajectory look-ahead.
    static constexpr std::size_t FUTURE_FRAME_COUNT = 50;
    static constexpr std::uint8_t SERVO_COUNT = 12;

    // Reduce this if the real robot moves too aggressively.
    static constexpr double GAIT_SCALE = 0.40;//gaitscale, default is 0.40

    // Center point used by the foot trajectory.
    static constexpr double NEUTRAL_FOOT_X = 0.040;

    // Exact RViz pose matching the physical servo-neutral posture.
    static constexpr double LEFT_STAND_HIP = 40.0 * PI / 180.0;
    static constexpr double LEFT_STAND_KNEE = 6.0 * PI / 180.0;
    static constexpr double RIGHT_STAND_HIP = -40.0 * PI / 180.0;
    static constexpr double RIGHT_STAND_KNEE = -6.0 * PI / 180.0;

    struct PoseCommands
    {
        std::array<double, SERVO_COUNT> rviz;
        std::array<double, SERVO_COUNT> servo;
    };

    const std::array<std::string, SERVO_COUNT> joint_names_ = {
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

    std::array<double, 4> phaseOffsets() const
    {
        // Order: front-left, front-right, back-left, back-right.
        if (gait_ == "trot")
        {
            return {0.0, PI, PI, 0.0};
        }

        // Crawl is the default gait.
        return {
    0.0,              // Front Left
    3.0 * PI / 2.0,               // Front Right
    PI,         // Back Left
    PI / 2.0,    // Back Right
};
    }

    PoseCommands standPose() const
    {
        return {
            {
                0.0, LEFT_STAND_HIP,  LEFT_STAND_KNEE,
                0.0, RIGHT_STAND_HIP, RIGHT_STAND_KNEE,
                0.0, LEFT_STAND_HIP,  LEFT_STAND_KNEE,
                0.0, RIGHT_STAND_HIP, RIGHT_STAND_KNEE
            },
            {
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0
            }
        };
    }

    LegAngles gaitDelta(
        double sample_time,
        double phase,
        const LegAngles & neutral,
        bool right_side
    ) const
    {
        const FootPosition foot = calculateStepTrajectory(
            sample_time,
            phase,
            speed_,
            z_ground_
        );

        const LegAngles current = solveLegIK(foot.x, foot.z);

        double hip_delta =
           (current.hip - neutral.hip) * GAIT_SCALE;

        double knee_delta =
            -(current.knee - neutral.knee) * GAIT_SCALE;

        // Right-side URDF joints use the opposite sign convention.
        if (right_side)
        {
            hip_delta = -hip_delta;
            knee_delta = -knee_delta;
        }

        return {hip_delta, knee_delta};
    }

    PoseCommands gaitPose(double sample_time) const
    {
        const auto phases = phaseOffsets();
        const LegAngles neutral =
            solveLegIK(NEUTRAL_FOOT_X, z_ground_);

        const FootPosition fl_foot = calculateStepTrajectory(
        sample_time,
        phases[0],
        speed_,
        z_ground_);

        RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    1000,
    "FL Foot: x=%.3f z=%.3f",
    fl_foot.x,
    fl_foot.z
);

        const LegAngles fl =
            gaitDelta(sample_time, phases[0], neutral, false);

        const LegAngles fr =
            gaitDelta(sample_time, phases[1], neutral, true);

        const LegAngles bl =
            gaitDelta(sample_time, phases[2], neutral, false);

        const LegAngles br =
            gaitDelta(sample_time, phases[3], neutral, true);

        const std::array<double, SERVO_COUNT> servo = {
            0.0, fl.hip, fl.knee,
            0.0, fr.hip, fr.knee,
            0.0, bl.hip, bl.knee,
            0.0, br.hip, br.knee
        };

        const std::array<double, SERVO_COUNT> rviz = {
            0.0,
            LEFT_STAND_HIP + fl.hip,
            LEFT_STAND_KNEE + fl.knee,

            0.0,
            RIGHT_STAND_HIP + fr.hip,
            RIGHT_STAND_KNEE + fr.knee,

            0.0,
            LEFT_STAND_HIP + bl.hip,
            LEFT_STAND_KNEE + bl.knee,

            0.0,
            RIGHT_STAND_HIP + br.hip,
            RIGHT_STAND_KNEE + br.knee
        };

        return {rviz, servo};
    }

    PoseCommands poseAt(double sample_time) const
    {
        if (standing_)
        {
            return standPose();
        }

        return gaitPose(sample_time);
    }

    void publishRvizPose(
        const rclcpp::Time & stamp,
        const std::array<double, SERVO_COUNT> & positions
    )
    {
        sensor_msgs::msg::JointState message;
        message.header.stamp = stamp;

        message.name.assign(
            joint_names_.begin(),
            joint_names_.end()
        );

        message.position.assign(
            positions.begin(),
            positions.end()
        );

        joint_publisher_->publish(message);
    }

    qr_robot_msgs::msg::ServoFrame makeServoFrame(
        const std::array<double, SERVO_COUNT> & servo_positions
    ) const
    {
        qr_robot_msgs::msg::ServoFrame frame;

        // These values preserve the old /servo_joint_commands semantics:
        // relative joint movement in radians. The WiFi bridge can convert
        // them to calibrated physical-servo commands before UDP transmission.
        for (std::size_t index = 0; index < SERVO_COUNT; ++index)
        {
            frame.servo_angles[index] =
                static_cast<float>(servo_positions[index]);
        }

        return frame;
    }

    void publishFutureTrajectory(const rclcpp::Time & stamp)
    {
        qr_robot_msgs::msg::ServoTrajectory trajectory;

        trajectory.header.stamp = stamp;
        trajectory.header.frame_id = "servo_trajectory";

        trajectory.first_frame_id = next_frame_id_;
        trajectory.frame_period_ms = FRAME_PERIOD_MS;
        trajectory.servo_count = SERVO_COUNT;

        trajectory.frames.reserve(FUTURE_FRAME_COUNT);

        for (std::size_t frame_index = 0;
             frame_index < FUTURE_FRAME_COUNT;
             ++frame_index)
        {
            const double future_time =
                time_ + static_cast<double>(frame_index) * TIME_STEP;

            const PoseCommands pose = poseAt(future_time);

            trajectory.frames.push_back(
                makeServoFrame(pose.servo)
            );
        }

        trajectory_publisher_->publish(trajectory);
    }

    void update()
    {
        const rclcpp::Time stamp = get_clock()->now();

        // RViz displays the first frame in the same trajectory window.
        const PoseCommands current_pose = poseAt(time_);
        publishRvizPose(stamp, current_pose.rviz);

        // The real robot receives this frame plus the following future frames.
        publishFutureTrajectory(stamp);

        // Each new packet advances the rolling window by exactly one frame.
        ++next_frame_id_;

        if (!standing_)
        {
            time_ += TIME_STEP;
        }
    }

    void gaitCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string command = msg->data;

        std::transform(
            command.begin(),
            command.end(),
            command.begin(),
            [](unsigned char character)
            {
                return static_cast<char>(std::tolower(character));
            }
        );

        if (command == "stand")
        {
            standing_ = true;
            RCLCPP_INFO(
                get_logger(),
                "Standing at calibrated servo neutrals."
            );
            return;
        }

        if (command == "walk")
        {
            command = "crawl";
        }

        if (command == "crawl" || command == "trot")
        {
            gait_ = command;
            standing_ = false;

            RCLCPP_INFO(
                get_logger(),
                "Selected gait: %s",
                gait_.c_str()
            );
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "Unknown gait command: %s",
            command.c_str()
        );
    }

    void speedCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        speed_ = std::max(
            0.1,
            static_cast<double>(msg->data)
        );

        RCLCPP_INFO(
            get_logger(),
            "Speed set to %.2f",
            speed_
        );
    }

    void heightCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        z_ground_ = static_cast<double>(msg->data);

        RCLCPP_INFO(
            get_logger(),
            "Ground height set to %.3f",
            z_ground_
        );
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        joint_publisher_;

    rclcpp::Publisher<qr_robot_msgs::msg::ServoTrajectory>::SharedPtr
        trajectory_publisher_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        gait_subscription_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
        speed_subscription_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
        height_subscription_;

    rclcpp::TimerBase::SharedPtr timer_;

    double time_ = 0.0;
    double speed_ = 2.0;
    double z_ground_ = -0.20;

    std::uint32_t next_frame_id_ = 0;

    std::string gait_ = "crawl";
    bool standing_ = true;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaitController>());
    rclcpp::shutdown();
    return 0;
}