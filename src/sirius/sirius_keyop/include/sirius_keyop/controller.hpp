#ifndef SIRIUS_CONTROLLER_HPP_
#define SIRIUS_CONTROLLER_HPP_

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "sirius_interfaces/msg/controller_input.hpp"
#include "sirius_interfaces/action/play_audio.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

#include "sirius_keyop/controller.hpp"

namespace sirius_controller
{
    class Controller final : public rclcpp::Node
    {
        public:

            explicit Controller(const rclcpp::NodeOptions & options);
            ~Controller() override;
            Controller & operator = (Controller && c) = delete;
            Controller(const Controller & c) = delete;
            Controller & operator = (const Controller & c) = delete;

        private:
            bool last_zero_vel_sent_;
            bool quit_requested_;
            bool shift_flag;
            bool assisted_teleop_active_;
            int key_file_descriptor_;
            float target_odom_pose_x;
            float target_odom_pose_y;
            float target_odom_orientation_z;
            float target_odom_orientation_w;
            
            // 滑らかな減速用の目標速度
            double target_linear_vel_;
            double target_angular_vel_;
            double linear_accel_rate_;   // 並進加速度/減速度 [m/s^2]
            double angular_accel_rate_;  // 旋回加速度/減速度 [rad/s^2]

            std::mutex cmd_mutex_;
            std::shared_ptr<geometry_msgs::msg::Twist> cmd_;
            std::shared_ptr<std_msgs::msg::String> signal_;
            std::shared_ptr<std_msgs::msg::String> face_;
            std::shared_ptr<std_msgs::msg::Bool> trans_;
            std::shared_ptr<rclcpp::TimerBase> timer_;
            std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> initial_pose_;

            rclcpp::Subscription<sirius_interfaces::msg::ControllerInput>::SharedPtr keyinput_subscriber_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_odom_subscription_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_teleop_publisher_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr face_publisher_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr signal_publisher_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trans_publisher_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bumper_publisher_;
            rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
            rclcpp_action::Client<sirius_interfaces::action::PlayAudio>::SharedPtr play_audio_client;
            rclcpp_action::ClientGoalHandle<sirius_interfaces::action::PlayAudio>::SharedPtr goal_handle_;
            rclcpp_action::Client<nav2_msgs::action::AssistedTeleop>::SharedPtr assisted_teleop_client;
            rclcpp_action::ClientGoalHandle<nav2_msgs::action::AssistedTeleop>::SharedPtr assisted_teleop_goal_handle_;
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client;
            rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navigate_through_poses_client;
            rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_client;

            void spin();
            void remoteKeyInputReceived(const sirius_interfaces::msg::ControllerInput::SharedPtr key);
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
            void keyboardInputLoop();
            void processKeyboardInput(char c);
            void disable();
            void incrementLinearVelocity();
            void decrementLinearVelocity();
            void incrementAngularVelocity();
            void decrementAngularVelocity();
            void resetLinearVelocity();
            void resetAngularVelocity();
            void smoothVelocityUpdate();  // 滑らかな速度更新
            void cancelGoal();
            void publishFace(const std::string &msg);
            void publishAudio(const std::string &file_name);
            void publishSignal(const std::string &msg);
            void publishTrans(bool state);
            void publishBumpper(bool state);
            void startAssistedTeleop();
            void stopAssistedTeleop();
            void cancelAllNavigationGoals();
            void publishPoseEstimate();

            struct termios original_terminal_state_;
            std::thread thread_;
    };
}

#endif
