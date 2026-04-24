#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <thread>

class SiriusKeyopV2 : public rclcpp::Node {
public:
    SiriusKeyopV2() : Node("sirius_keyop_v2"), 
                      last_input_time_(0, 0, RCL_ROS_TIME), 
                      last_release_time_(0, 0, RCL_ROS_TIME) {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_teleop", 10);
        stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/stop", 10);
        reset_publisher_ = this->create_publisher<std_msgs::msg::Empty>("/sirius/reset", 10);
        initialpose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        
        // クロックソースをノードに合わせる
        last_input_time_ = this->now();
        last_release_time_ = this->now();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SiriusKeyopV2::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Sirius Keyop V2 (Smart Mode) Started");
        RCLCPP_INFO(this->get_logger(), "---------------------------");
        RCLCPP_INFO(this->get_logger(), "Manual control will override Nav2 ONLY when keys are being used.");
        RCLCPP_INFO(this->get_logger(), "Otherwise, Nav2 will have control.");
        RCLCPP_INFO(this->get_logger(), "---------------------------");
        RCLCPP_INFO(this->get_logger(), "q/e : EMERGENCY STOP / RELEASE");
        RCLCPP_INFO(this->get_logger(), "r   : RESET POSITION (Teleport)");
        RCLCPP_INFO(this->get_logger(), "w/x : linear vel +/-");
        RCLCPP_INFO(this->get_logger(), "a/d : angular vel +/-");
        RCLCPP_INFO(this->get_logger(), "s   : reset velocity to 0 (yields control to Nav2)");
        RCLCPP_INFO(this->get_logger(), "CTRL-C to quit");
    }

    void run() {
        while (rclcpp::ok()) {
            char c = get_key();
            if (c == 'q') {
                is_stopped_ = true;
                linear_vel_ = 0.0;
                angular_vel_ = 0.0;
                last_input_time_ = this->now();
                printf("\n>>> EMERGENCY STOP (LOCKED) <<<\n");
            } else if (c == 'e') {
                is_stopped_ = false;
                last_input_time_ = this->now();
                printf("\n>>> STOP RELEASED (UNLOCKED) <<<\n");
            } else if (c == 'r') {
                reset_system();
            } else if (c == 'w') {
                linear_vel_ += 0.1;
                last_input_time_ = this->now();
                print_status();
            } else if (c == 'x') {
                linear_vel_ -= 0.1;
                last_input_time_ = this->now();
                print_status();
            } else if (c == 'a') {
                angular_vel_ += 0.1;
                last_input_time_ = this->now();
                print_status();
            } else if (c == 'd') {
                angular_vel_ -= 0.1;
                last_input_time_ = this->now();
                print_status();
            } else if (c == 's') {
                linear_vel_ = 0.0;
                angular_vel_ = 0.0;
                last_input_time_ = this->now();
                print_status();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

private:
    void reset_system() {
        reset_publisher_->publish(std_msgs::msg::Empty());
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = this->now();
        pose_msg.pose.pose.position.x = 0.0;
        pose_msg.pose.pose.position.y = 0.0;
        pose_msg.pose.pose.position.z = 0.0;
        pose_msg.pose.pose.orientation.w = 1.0;
        pose_msg.pose.covariance[0] = 0.25;
        pose_msg.pose.covariance[7] = 0.25;
        pose_msg.pose.covariance[35] = 0.06;
        initialpose_publisher_->publish(pose_msg);
        linear_vel_ = 0.0;
        angular_vel_ = 0.0;
        is_stopped_ = false;
        last_input_time_ = this->now();
        printf("\n>>> SYSTEM RESET <<<\n");
    }

    void timer_callback() {
        auto now = this->now();
        
        // 1. 速度指令の送信条件判定
        bool has_manual_vel = (std::abs(linear_vel_) > 0.01 || std::abs(angular_vel_) > 0.01);
        bool within_grace_period = false;
        try {
            within_grace_period = (now - last_input_time_).seconds() < 1.0;
        } catch (const std::runtime_error &) {
            // 万が一 clock source が不一致になった場合のセーフティ
            last_input_time_ = now;
        }

        if (has_manual_vel || within_grace_period) {
            auto twist_msg = geometry_msgs::msg::Twist();
            if (linear_vel_ > 1.0) linear_vel_ = 1.0;
            if (linear_vel_ < -1.0) linear_vel_ = -1.0;
            if (angular_vel_ > 1.5) angular_vel_ = 1.5;
            if (angular_vel_ < -1.5) angular_vel_ = -1.5;
            twist_msg.linear.x = linear_vel_;
            twist_msg.angular.z = angular_vel_;
            publisher_->publish(twist_msg);
        }

        // 2. 停止信号の送信
        if (is_stopped_) {
            auto stop_msg = std_msgs::msg::Bool();
            stop_msg.data = true;
            stop_publisher_->publish(stop_msg);
            last_stop_state_ = true;
        } else {
            if (last_stop_state_) {
                last_release_time_ = now;
                last_stop_state_ = false;
            }
            try {
                if ((now - last_release_time_).seconds() < 0.5) {
                    auto stop_msg = std_msgs::msg::Bool();
                    stop_msg.data = false;
                    stop_publisher_->publish(stop_msg);
                }
            } catch (const std::runtime_error &) {
                last_release_time_ = now;
            }
        }
    }

    void print_status() {
        printf("\rSmart Mode - Linear: %+.2f, Angular: %+.2f  [%s]  ", 
               linear_vel_, angular_vel_, is_stopped_ ? "STOPPED" : "ACTIVE");
        fflush(stdout);
    }

    char get_key() {
        struct termios oldt, newt;
        char ch = 0;
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
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double linear_vel_ = 0.0;
    double angular_vel_ = 0.0;
    bool is_stopped_ = false;
    bool last_stop_state_ = false;
    rclcpp::Time last_input_time_;
    rclcpp::Time last_release_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SiriusKeyopV2>();
    std::thread spin_thread([&]() { rclcpp::spin(node); });
    node->run();
    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
