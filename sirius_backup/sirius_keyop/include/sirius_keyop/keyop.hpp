#ifndef SIRIUS_KEYOP_HPP_
#define SIRIUS_KEYOP_HPP_

#include <termios.h>

#include <memory>
#include <mutex>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sirius_interfaces/msg/keyboard_input.hpp>

namespace sirius_keyop
{
    class KeyOp final : public rclcpp::Node
    {
        public:
        
            explicit KeyOp(const rclcpp::NodeOptions & options);
            ~KeyOp() override;
            KeyOp(KeyOp && c) = delete;
            KeyOp & operator = (KeyOp && c) = delete;
            KeyOp(const KeyOp & c) = delete;
            KeyOp & operator = (const KeyOp & c) = delete;

        private:
            rclcpp::Subscription<sirius_interfaces::msg::KeyboardInput>::SharedPtr keyinput_subscriber_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr waypoint_publisher_;
            bool last_zero_vel_sent_;
            std::mutex cmd_mutex_;
            std::shared_ptr<geometry_msgs::msg::Twist> cmd_;
            std::shared_ptr<std_msgs::msg::Bool> waypoint_; 
            std::shared_ptr<rclcpp::TimerBase> timer_;

            void spin();

            void enable();
            void disable();
            void waypoint();
            void incrementLinearVelocity();
            void decrementLinearVelocity();
            void incrementAngularVelocity();
            void decrementAngularVelocity();
            void resetVelocity();

            void keyboardInputLoop();
            void processKeyboardInput(char c);
            void remoteKeyInputReceived(const std::shared_ptr<sirius_interfaces::msg::KeyboardInput> key);
            void restoreTerminal();
            bool quit_requested_;
            int key_file_descriptor_;
            struct termios original_terminal_state_;
            std::thread thread_;
    };
}

#endif
