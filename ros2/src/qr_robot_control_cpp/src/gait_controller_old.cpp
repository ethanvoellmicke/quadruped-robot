#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#include "qr_robot_control_cpp/foot_trajectory.hpp"
#include "qr_robot_control_cpp/leg_ik.hpp"

class GaitController : public rclcpp::Node
{
public:
    GaitController()
        : Node("gait_controller")
    {
        // Absolute joint angles for RViz.
        joint_publisher_ =
            create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // Relative angles for the real robot.
        // Zero on every joint means every servo is at its calibrated neutral.
        servo_publisher_ =
            create_publisher<sensor_msgs::msg::JointState>(
                "/servo_joint_commands",
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
            std::chrono::milliseconds(20),//default is 20, this is sending speed
            std::bind(&GaitController::update, this)
        );

        RCLCPP_INFO(
            get_logger(),
            "Gait controller started. Stand sends zero servo movement."
        );
    }

private:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double TIME_STEP = 0.02;

    // Reduce this if the real robot moves too aggressively.
    static constexpr double GAIT_SCALE = 0.40;

    // The center point used by the foot trajectory.
    static constexpr double NEUTRAL_FOOT_X = 0.080;

    // Exact ROS pose that matches the real robot's servo-neutral posture.
    static constexpr double LEFT_STAND_HIP = 40.0 * PI / 180.0;
    static constexpr double LEFT_STAND_KNEE = 6.0 * PI / 180.0;
    static constexpr double RIGHT_STAND_HIP = -40.0 * PI / 180.0;
    static constexpr double RIGHT_STAND_KNEE = -6.0 * PI / 180.0;

    struct PoseCommands
    {
        std::vector<double> rviz;
        std::vector<double> servo;
    };

    const std::vector<std::string> joint_names_ = {
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
        return {PI, 0.0, 3.0 * PI / 2.0, PI / 2.0};
    }

    PoseCommands standPose() const
    {
        return {
            // Absolute pose shown in RViz.
            {
                0.0, LEFT_STAND_HIP,  LEFT_STAND_KNEE,
                0.0, RIGHT_STAND_HIP, RIGHT_STAND_KNEE,
                0.0, LEFT_STAND_HIP,  LEFT_STAND_KNEE,
                0.0, RIGHT_STAND_HIP, RIGHT_STAND_KNEE
            },

            // Relative movement sent to the real robot.
            std::vector<double>(12, 0.0)
        };
    }

    LegAngles gaitDelta(
        double phase,
        const LegAngles & neutral,
        bool right_side
    ) const
    {
        const FootPosition foot = calculateStepTrajectory(
            time_,
            phase,
            speed_,
            z_ground_
        );

        const LegAngles current = solveLegIK(foot.x, foot.z);

        double hip_delta =
            (current.hip - neutral.hip) * GAIT_SCALE;

        double knee_delta =
            (current.knee - neutral.knee) * GAIT_SCALE;

        // The right-side URDF joints use the opposite sign convention.
        if (right_side)
        {
            hip_delta = -hip_delta;
            knee_delta = -knee_delta;
        }

        return {hip_delta, knee_delta};
    }

    PoseCommands gaitPose() const
    {
        const auto phases = phaseOffsets();
        const LegAngles neutral = solveLegIK(NEUTRAL_FOOT_X, z_ground_);

        const LegAngles fl = gaitDelta(phases[0], neutral, false);
        const LegAngles fr = gaitDelta(phases[1], neutral, true);
        const LegAngles bl = gaitDelta(phases[2], neutral, false);
        const LegAngles br = gaitDelta(phases[3], neutral, true);

        const std::vector<double> servo = {
            0.0, fl.hip, fl.knee,
            0.0, fr.hip, fr.knee,
            0.0, bl.hip, bl.knee,
            0.0, br.hip, br.knee
        };

        // RViz receives the real neutral geometry plus gait movement.
        const std::vector<double> rviz = {
            0.0, LEFT_STAND_HIP  + fl.hip, LEFT_STAND_KNEE  + fl.knee,
            0.0, RIGHT_STAND_HIP + fr.hip, RIGHT_STAND_KNEE + fr.knee,
            0.0, LEFT_STAND_HIP  + bl.hip, LEFT_STAND_KNEE  + bl.knee,
            0.0, RIGHT_STAND_HIP + br.hip, RIGHT_STAND_KNEE + br.knee
        };

        return {rviz, servo};
    }

    void publishPose(
        const rclcpp::Time & stamp,
        const std::vector<double> & positions,
        const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr & publisher
    ) const
    {
        sensor_msgs::msg::JointState message;
        message.header.stamp = stamp;
        message.name = joint_names_;
        message.position = positions;
        publisher->publish(message);
    }

    void update()
    {
        if (!standing_)
        {
            time_ += TIME_STEP;
        }

        const PoseCommands pose = standing_ ? standPose() : gaitPose();
        const rclcpp::Time stamp = get_clock()->now();

        publishPose(stamp, pose.rviz, joint_publisher_);
        publishPose(stamp, pose.servo, servo_publisher_);
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
            RCLCPP_INFO(get_logger(), "Standing at servo neutrals");
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
            RCLCPP_INFO(get_logger(), "Selected gait: %s", gait_.c_str());
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
        speed_ = std::max(0.1, static_cast<double>(msg->data));
        RCLCPP_INFO(get_logger(), "Speed set to %.2f", speed_);
    }

    void heightCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        z_ground_ = static_cast<double>(msg->data);
        RCLCPP_INFO(get_logger(), "Ground height set to %.3f", z_ground_);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        joint_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        servo_publisher_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        gait_subscription_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
        speed_subscription_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
        height_subscription_;

    rclcpp::TimerBase::SharedPtr timer_;

    double time_ = 0.0;
    double speed_ = 2.0;
    double z_ground_ = -0.16;

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