#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

#include <std_msgs/msg/bool.hpp>

class SiriusKeyopV2 : public rclcpp::Node {
public:
    SiriusKeyopV2() : Node("sirius_keyop_v2") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_teleop", 10);
        stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/stop", 10);
        
        RCLCPP_INFO(this->get_logger(), "Sirius Keyop V2 (Priority Mode) Started");
        RCLCPP_INFO(this->get_logger(), "---------------------------");
        RCLCPP_INFO(this->get_logger(), "Moving around:");
        RCLCPP_INFO(this->get_logger(), "   w");
        RCLCPP_INFO(this->get_logger(), "a  s  d");
        RCLCPP_INFO(this->get_logger(), "   x");
        RCLCPP_INFO(this->get_logger(), "---------------------------");
        RCLCPP_INFO(this->get_logger(), "q   : EMERGENCY STOP (Lock Mux)");
        RCLCPP_INFO(this->get_logger(), "e   : RELEASE STOP (Unlock Mux)");
        RCLCPP_INFO(this->get_logger(), "w/x : linear vel +/-");
        RCLCPP_INFO(this->get_logger(), "a/d : angular vel +/-");
        RCLCPP_INFO(this->get_logger(), "s   : reset velocity to 0");
        RCLCPP_INFO(this->get_logger(), "CTRL-C to quit");
    }

    void run() {
        char c;
        while (rclcpp::ok()) {
            c = get_key();
            
            auto msg = geometry_msgs::msg::Twist();
            auto stop_msg = std_msgs::msg::Bool();
            bool changed = false;

            if (c == 'q') {
                stop_msg.data = true;
                stop_publisher_->publish(stop_msg);
                linear_vel_ = 0.0;
                angular_vel_ = 0.0;
                printf("\n>>> EMERGENCY STOP (LOCKED) <<<\n");
                changed = true;
            } else if (c == 'e') {
                stop_msg.data = false;
                stop_publisher_->publish(stop_msg);
                printf("\n>>> STOP RELEASED (UNLOCKED) <<<\n");
                changed = true;
            } else if (c == 'w') {
                linear_vel_ += 0.1;
                changed = true;
            } else if (c == 'x') {
                linear_vel_ -= 0.1;
                changed = true;
            } else if (c == 'a') {
                angular_vel_ += 0.1;
                changed = true;
            } else if (c == 'd') {
                angular_vel_ -= 0.1;
                changed = true;
            } else if (c == 's') {
                linear_vel_ = 0.0;
                angular_vel_ = 0.0;
                changed = true;
            }

            if (changed) {
                if (linear_vel_ > 1.0) linear_vel_ = 1.0;
                if (linear_vel_ < -1.0) linear_vel_ = -1.0;
                if (angular_vel_ > 1.5) angular_vel_ = 1.5;
                if (angular_vel_ < -1.5) angular_vel_ = -1.5;

                msg.linear.x = linear_vel_;
                msg.angular.z = angular_vel_;
                publisher_->publish(msg);
                
                printf("\rV2 Mode - Linear: %+.2f, Angular: %+.2f    ", linear_vel_, angular_vel_);
                fflush(stdout);
            }
            
            rclcpp::spin_some(this->get_node_base_interface());
            usleep(100);
        }
    }

private:
    char get_key() {
        struct termios oldt, newt;
        char ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        newt.c_cc[VMIN] = 0;
        newt.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        read(STDIN_FILENO, &ch, 1);
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_publisher_;
    double linear_vel_ = 0.0;
    double angular_vel_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SiriusKeyopV2>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
