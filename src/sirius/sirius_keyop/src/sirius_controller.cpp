#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sirius_interfaces/msg/controller_input.hpp"
#include "sirius_interfaces/action/play_audio.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

#include "sirius_keyop/controller.hpp"

using namespace std::chrono_literals;
using std::string;

namespace sirius_controller
{
    Controller::Controller(const rclcpp::NodeOptions & options) : rclcpp::Node("sirius_controller", options), last_zero_vel_sent_(true), quit_requested_(false), shift_flag(false), assisted_teleop_active_(false), key_file_descriptor_(0), target_linear_vel_(0.0), target_angular_vel_(0.0), linear_accel_rate_(1.0), angular_accel_rate_(1.0)
    {
    tcgetattr(key_file_descriptor_, &original_terminal_state_);
    cmd_ = std::make_shared<geometry_msgs::msg::Twist>();
    signal_ = std::make_shared<std_msgs::msg::String>();
    face_ = std::make_shared<std_msgs::msg::String>();
    trans_ = std::make_shared<std_msgs::msg::Bool>();
    initial_pose_ = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    double linear_vel_step = this->declare_parameter("linear_vel_step", 0.1);
    double linear_vel_max = this->declare_parameter("linear_vel_max", 2.0);
    double angular_vel_step = this->declare_parameter("angular_vel_step", 0.157);
    double angular_vel_max = this->declare_parameter("angular_vel_max", 3.14);
    linear_accel_rate_ = this->declare_parameter("linear_accel_rate", 0.5);    // 並進加速度/減速度 [m/s^2]
    angular_accel_rate_ = this->declare_parameter("angular_accel_rate", 1.0);  // 旋回加速度/減速度 [rad/s^2]

    RCLCPP_INFO(get_logger(), "KeyOp : using linear vel step [%f].", linear_vel_step);
    RCLCPP_INFO(get_logger(), "KeyOp : using linear vel max [%f].", linear_vel_max);
    RCLCPP_INFO(get_logger(), "KeyOp : using angular vel step [%f].", angular_vel_step);
    RCLCPP_INFO(get_logger(), "KeyOp : using angular vel max [%f].", angular_vel_max);
    RCLCPP_INFO(get_logger(), "KeyOp : using linear accel rate [%f].", linear_accel_rate_);
    RCLCPP_INFO(get_logger(), "KeyOp : using angular accel rate [%f].", angular_accel_rate_);

    keyinput_subscriber_ = this->create_subscription<sirius_interfaces::msg::ControllerInput>("controller", rclcpp::QoS(1), std::bind(&Controller::remoteKeyInputReceived, this, std::placeholders::_1));

    target_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/target_odom", 10, std::bind(&Controller::odomCallback, this, std::placeholders::_1));

    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    velocity_teleop_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_teleop", 1);

    face_publisher_ = this->create_publisher<std_msgs::msg::String>("face_expression", 10);

    signal_publisher_ = this->create_publisher<std_msgs::msg::String>("signal", 10);

    trans_publisher_ = this->create_publisher<std_msgs::msg::Bool>("trans", 10);

    bumper_publisher_ = this->create_publisher<std_msgs::msg::Bool>("stop", 10);

    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    
    play_audio_client = rclcpp_action::create_client<sirius_interfaces::action::PlayAudio>(
        this, "play_audio"
    );

    assisted_teleop_client = rclcpp_action::create_client<nav2_msgs::action::AssistedTeleop>(
        this, "assisted_teleop"
    );

    navigate_to_pose_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose"
    );

    navigate_through_poses_client = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
        this, "navigate_through_poses"
    );

    follow_waypoints_client = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
        this, "follow_waypoints"
    );

    RCLCPP_INFO(get_logger(), "KeyOp : connected.");

    target_odom_pose_x = 0.0;
    target_odom_pose_y = 0.0;
    target_odom_orientation_z = 0.0;
    target_odom_orientation_w = 1.0;

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Controller::spin, this));

    thread_ = std::thread(&Controller::keyboardInputLoop, this);
    }

    Controller::~Controller()
    {
        disable();
        
        // Assisted Teleopが実行中なら停止
        if (assisted_teleop_active_ && assisted_teleop_goal_handle_)
        {
            assisted_teleop_client->async_cancel_goal(assisted_teleop_goal_handle_);
        }
        
        quit_requested_ = true;
        thread_.join();
        tcsetattr(key_file_descriptor_, TCSANOW, &original_terminal_state_);
    }

    void Controller::spin()
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);

        // 滑らかな速度更新を実行
        smoothVelocityUpdate();

        if((cmd_->linear.x != 0.0) || (cmd_->angular.z != 0.0))
        {
            // Assisted Teleop有効時は cmd_vel_teleop に送信、無効時は cmd_vel に送信
            if(assisted_teleop_active_)
            {
                velocity_teleop_publisher_->publish(*cmd_);
            }
            else
            {
                velocity_publisher_->publish(*cmd_);
            }
            last_zero_vel_sent_ = false;
        }
        else if(last_zero_vel_sent_ == false)
        {
            // 停止コマンドも同様に適切なトピックに送信
            if(assisted_teleop_active_)
            {
                velocity_teleop_publisher_->publish(*cmd_);
            }
            else
            {
                velocity_publisher_->publish(*cmd_);
            }
            last_zero_vel_sent_ = true;
        }
    }

    void Controller::keyboardInputLoop()
    {
        struct termios raw;
        std::memcpy(&raw, &original_terminal_state_, sizeof(struct termios));

        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOL] = 2;
        tcsetattr(key_file_descriptor_, TCSANOW, &raw);

        puts("Reading from controller");
        puts("-----------------------");
        puts("Forward/back arrows : linear velocity incr/decr.");
        puts("Right/left arrows : angular velocity incr/decr.");
        puts("Spacebar : reset linear/angular velocities.");

        while(!quit_requested_)
        {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(key_file_descriptor_, &readfds);
            struct timeval timeout;
            timeout.tv_sec = 0;
            timeout.tv_usec = 50 *1000; //50ms
            int select_ret = ::select(key_file_descriptor_ + 1, &readfds, nullptr, nullptr, &timeout);

            if(select_ret < 0)
            {
                //エラー
                RCLCPP_ERROR(get_logger(), "Select failed : %s", ::strerror(errno));
            }
            else if(select_ret == 0)
            {
                //タイムアウト、何もしない
            }
            else
            {
                if(!FD_ISSET(key_file_descriptor_, &readfds))
                {
                    //エラー
                    RCLCPP_ERROR(get_logger(), "Invalid file descriptor.");
                }

                char c;

                if(::read(key_file_descriptor_, &c, 1) < 0)
                {
                    //エラー
                    RCLCPP_ERROR(get_logger(), "Failed to read character : %s", ::strerror(errno));
                }
                else
                {
                    processKeyboardInput(c);
                }
            }
        }
    }

    void Controller::remoteKeyInputReceived(const std::shared_ptr<sirius_interfaces::msg::ControllerInput> key)
    {
        processKeyboardInput(key->pressed_key);
    }

    void Controller::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        target_odom_pose_x = msg->pose.pose.position.x;
        target_odom_pose_y = msg->pose.pose.position.y;
        target_odom_orientation_z = msg->pose.pose.orientation.z;
        target_odom_orientation_w = msg->pose.pose.orientation.w;
    }

    void Controller::processKeyboardInput(char c)
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        switch(c)
        {
            case sirius_interfaces::msg::ControllerInput::KEYCODE_UP:
            {
                incrementLinearVelocity();
                shift_flag = false;
                break;
            }
            case sirius_interfaces::msg::ControllerInput::KEYCODE_DOWN:
            {
                decrementLinearVelocity();
                shift_flag = false;
                break;
            }
            case sirius_interfaces::msg::ControllerInput::KEYCODE_RIGHT:
            {
                decrementAngularVelocity();
                shift_flag = false;
                break;
            }
            case sirius_interfaces::msg::ControllerInput::KEYCODE_LEFT:
            {
                incrementAngularVelocity();
                shift_flag = false;
                break;
            }
            case ',':
            {
                resetLinearVelocity();
                shift_flag = false;
                break;
            }
            case '.':
            {
                resetAngularVelocity();
                shift_flag = false;
                break;
            }
            case '1':
            {
                publishFace("neutral");
                shift_flag = false;
                break;
            }
            case '2':
            {
                publishFace("happy");
                shift_flag = false;
                break;
            }
            case '3':
            {
                if (shift_flag)
                {
                    publishFace("sad");
                }
                else
                {
                    publishFace("crying");
                }
                shift_flag = false;
                break;
            }
            case '4':
            {
                publishFace("surprised");
                shift_flag = false;
                break;
            }
            case '5':
            {
                publishFace("hurt");
                shift_flag = false;
                break;
            }
            case '8':
            {
                publishFace("wink");
                shift_flag = false;
                break;
            }
            case '0':
            {
                publishFace("auto");
                shift_flag = false;
                break;
            }
            case 'a':
            {
                if (shift_flag)
                {
                    publishTrans(false);
                }
                else
                {
                    publishTrans(true);
                }
                shift_flag = false;
                break;
            }
            case 'b':
            {
                if (shift_flag)
                {
                    publishAudio("restart.wav");
                }
                else
                {
                    publishAudio("bumper.wav");
                }
                shift_flag = false;
                break;
            }
            case 'c':
            {
                cancelGoal();
                shift_flag = false;
                break;
            }
            case 'e':
            {
                if (shift_flag)
                {
                    publishAudio("en.wav");
                }
                else
                {
                    publishAudio("ja.wav");
                }
                shift_flag = false;
                break;
            }
            case 'h':
            {
                if (shift_flag)
                {
                    publishSignal("hazard");
                }
                else
                {
                    publishSignal("thanks");
                }
                shift_flag = false;
                break;
            }
            case 'i':
            {
                publishPoseEstimate();
                shift_flag = false;
                break;
            }
            case 'q':
            {
                if (shift_flag)
                {
                    publishBumpper(false);
                }
                else
                {
                    publishBumpper(true);
                }
                shift_flag = false;
                break;
            }
            case 's':
            {
                if (shift_flag)
                {
                    shift_flag = false;
                }
                else
                {
                    shift_flag = true;
                }
                break;
            }
            case 't': // Toggle Assisted Teleop
            {
                if (shift_flag)
                {
                    stopAssistedTeleop();
                }
                else
                {
                    startAssistedTeleop();
                }
                shift_flag = false;
                break;
            }
        }
    }

    void Controller::disable()
    {
        cmd_->linear.x = 0.0;
        cmd_->angular.z = 0.0;
        
        // Assisted Teleop有効時は cmd_vel_teleop に送信、無効時は cmd_vel に送信
        if(assisted_teleop_active_)
        {
            velocity_teleop_publisher_->publish(*cmd_);
        }
        else
        {
            velocity_publisher_->publish(*cmd_);
        }
    }

    void Controller::incrementLinearVelocity()
    {
        double linear_vel_max = get_parameter("linear_vel_max").get_value<double>();
        double linear_vel_step = get_parameter("linear_vel_step").get_value<double>();

        if (target_linear_vel_ <= linear_vel_max)
        {
            target_linear_vel_ += linear_vel_step;
        }

        RCLCPP_INFO(get_logger(), "Controller : Linear velocity incremented [target: %f | current: %f]", target_linear_vel_, cmd_->linear.x);
    }

    void Controller::decrementLinearVelocity()
    {
        double linear_vel_max = get_parameter("linear_vel_max").get_value<double>();
        double linear_vel_step = get_parameter("linear_vel_step").get_value<double>();

        if (target_linear_vel_ >= -linear_vel_max)
        {
            target_linear_vel_ -= linear_vel_step;
        }

        RCLCPP_INFO(get_logger(), "Controller : Linear velocity decremented [target: %f | current: %f]", target_linear_vel_, cmd_->linear.x);
    }

    void Controller::incrementAngularVelocity()
    {
        double angular_vel_max = get_parameter("angular_vel_max").get_value<double>();
        double angular_vel_step = get_parameter("angular_vel_step").get_value<double>();

        if (target_angular_vel_ <= angular_vel_max)
        {
            target_angular_vel_ += angular_vel_step;
        }

        RCLCPP_INFO(get_logger(), "Controller : Angular velocity incremented [target: %f | current: %f]", target_angular_vel_, cmd_->angular.z);
    }

    void Controller::decrementAngularVelocity()
    {
        double angular_vel_max = get_parameter("angular_vel_max").get_value<double>();
        double angular_vel_step = get_parameter("angular_vel_step").get_value<double>();

        if (target_angular_vel_ >= -angular_vel_max)
        {
            target_angular_vel_ -= angular_vel_step;
        }

        RCLCPP_INFO(get_logger(), "Controller : Angular velocity decrement [target: %f | current: %f]", target_angular_vel_, cmd_->angular.z);
    }

    void Controller::resetLinearVelocity()
    {
        target_linear_vel_ = 0.0;

        RCLCPP_INFO(get_logger(), "Controller : Reset linear velocity (smooth deceleration).");
    }

    void Controller::resetAngularVelocity()
    {
        target_angular_vel_ = 0.0;

        RCLCPP_INFO(get_logger(), "Controller : Reset angular velocity (smooth deceleration).");
    }

    void Controller::smoothVelocityUpdate()
    {
        // タイマー周期: 100ms = 0.1秒
        const double dt = 0.1;
        
        // 並進速度の最大変化量 = 並進加速度 * 時間
        const double max_linear_delta = linear_accel_rate_ * dt;
        // 旋回速度の最大変化量 = 旋回加速度 * 時間
        const double max_angular_delta = angular_accel_rate_ * dt;
        
        // 線形速度の滑らかな更新
        double linear_diff = target_linear_vel_ - cmd_->linear.x;
        if (std::abs(linear_diff) > max_linear_delta)
        {
            // 差分が大きい場合は、加速度制限で更新
            cmd_->linear.x += (linear_diff > 0 ? max_linear_delta : -max_linear_delta);
        }
        else
        {
            // 差分が小さい場合は目標に到達
            cmd_->linear.x = target_linear_vel_;
        }
        
        // 角速度の滑らかな更新
        double angular_diff = target_angular_vel_ - cmd_->angular.z;
        if (std::abs(angular_diff) > max_angular_delta)
        {
            cmd_->angular.z += (angular_diff > 0 ? max_angular_delta : -max_angular_delta);
        }
        else
        {
            cmd_->angular.z = target_angular_vel_;
        }
    }

    void Controller::cancelGoal()
    {
        using action_msgs::msg::GoalStatus;

        if (goal_handle_)
        {
            auto cancel_future = play_audio_client->async_cancel_goal(goal_handle_);
            RCLCPP_INFO(get_logger(), "Controller : Audio playback goal canceled.");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Controller : No active audio goal to cancel.");
        }
    }

    void Controller::publishFace(const std::string &msg)
    {
        face_->data = msg;
        face_publisher_->publish(*face_);

        RCLCPP_INFO(get_logger(), "Controller : Publish face topic.");
    }

    void Controller::publishAudio(const std::string &file_name)
    {
        using PlayAudio = sirius_interfaces::action::PlayAudio;

        if (!play_audio_client->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(get_logger(), "PlayAudio action server not available...");
            return;
        }

        auto goal_msg = PlayAudio::Goal();
        goal_msg.file_name = file_name;

        auto send_goal_options = rclcpp_action::Client<PlayAudio>::SendGoalOptions();

        send_goal_options.goal_response_callback = [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<PlayAudio>> handle) {
            if (!handle) 
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } 
            else 
            {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
                goal_handle_ = handle;
            }
        };

        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<PlayAudio>::WrappedResult & result){
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "PlayAudio action succeeded");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "PlayAudio action failed");
            }
            goal_handle_.reset();
        };

        play_audio_client->async_send_goal(goal_msg, send_goal_options);
    }
    
    void Controller::publishSignal(const std::string &msg)
    {
        signal_->data = msg;
        signal_publisher_->publish(*signal_);

        RCLCPP_INFO(get_logger(), "Controller : Publish signal topic.");
    }

    void Controller::publishTrans(bool state)
    {
        trans_->data = state;
        trans_publisher_->publish(*trans_);

        RCLCPP_INFO(get_logger(), "Controller : Publish trans topic.");
    }

    void Controller::publishBumpper(bool state)
    {
        std_msgs::msg::Bool bumper;
        bumper.data = state;
        bumper_publisher_->publish(bumper);

        if (state) {
            RCLCPP_INFO(get_logger(), "Controller : 緊急停止オン");
        } else {
            RCLCPP_INFO(get_logger(), "Controller : 緊急停止オフ");
        }
    }

    void Controller::startAssistedTeleop()
    {
        using AssistedTeleop = nav2_msgs::action::AssistedTeleop;

        if (assisted_teleop_active_)
        {
            RCLCPP_INFO(get_logger(), "Controller : Assisted Teleop is already active.");
            return;
        }

        if (!assisted_teleop_client->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(get_logger(), "AssistedTeleop action server not available...");
            return;
        }

        auto goal_msg = AssistedTeleop::Goal();
        goal_msg.time_allowance.sec = 300;  // 5分間

        auto send_goal_options = rclcpp_action::Client<AssistedTeleop>::SendGoalOptions();

        send_goal_options.goal_response_callback = [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::AssistedTeleop>> handle) {
            if (!handle) 
            {
                RCLCPP_ERROR(this->get_logger(), "AssistedTeleop goal was rejected by server");
            } 
            else 
            {
                RCLCPP_INFO(this->get_logger(), "AssistedTeleop goal accepted by server");
                assisted_teleop_goal_handle_ = handle;
                assisted_teleop_active_ = true;
                RCLCPP_INFO(this->get_logger(), "Switching to cmd_vel_teleop topic for Assisted Teleop");
            }
        };

        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::AssistedTeleop>::WrappedResult & result){
            assisted_teleop_active_ = false;
            assisted_teleop_goal_handle_.reset();
            RCLCPP_INFO(this->get_logger(), "Switching back to cmd_vel topic (normal mode)");
            
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "AssistedTeleop action succeeded");
            }
            else if (result.code == rclcpp_action::ResultCode::CANCELED)
            {
                RCLCPP_INFO(this->get_logger(), "AssistedTeleop action was canceled");
            }
            else if (result.code == rclcpp_action::ResultCode::ABORTED)
            {
                RCLCPP_ERROR(this->get_logger(), "AssistedTeleop action was aborted");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "AssistedTeleop action failed with unknown result");
            }
        };

        assisted_teleop_client->async_send_goal(goal_msg, send_goal_options);
        
        RCLCPP_INFO(get_logger(), "Controller : Started Assisted Teleop.");
    }

    void Controller::stopAssistedTeleop()
    {
        if (!assisted_teleop_active_ || !assisted_teleop_goal_handle_)
        {
            RCLCPP_INFO(get_logger(), "Controller : No active Assisted Teleop to stop.");
            return;
        }

        RCLCPP_INFO(get_logger(), "Controller : Canceling Assisted Teleop...");
        
        try {
            auto cancel_future = assisted_teleop_client->async_cancel_goal(assisted_teleop_goal_handle_);
            
            // キャンセル結果を短時間待機
            auto future_status = cancel_future.wait_for(std::chrono::seconds(2));
            
            if (future_status == std::future_status::ready) 
            {
                auto cancel_response = cancel_future.get();
                RCLCPP_INFO(get_logger(), "Controller : Assisted Teleop cancel request sent.");
            }
            else 
            {
                RCLCPP_WARN(get_logger(), "Controller : Timeout waiting for cancel response.");
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Controller : Error canceling Assisted Teleop: %s", e.what());
        }
        
        // 状態をリセット
        assisted_teleop_active_ = false;
        assisted_teleop_goal_handle_.reset();
        
        RCLCPP_INFO(get_logger(), "Controller : Assisted Teleop stopped.");
        RCLCPP_INFO(get_logger(), "Switching back to cmd_vel topic (normal mode)");
    }

    void Controller::cancelAllNavigationGoals()
    {
        RCLCPP_INFO(get_logger(), "Controller : Canceling all navigation goals...");
        
        // NavigateToPose のキャンセル
        try {
            if (navigate_to_pose_client->wait_for_action_server(std::chrono::seconds(1))) 
            {
                auto cancel_future = navigate_to_pose_client->async_cancel_all_goals();
                auto future_status = cancel_future.wait_for(std::chrono::seconds(1));
                
                if (future_status == std::future_status::ready) 
                {
                    RCLCPP_INFO(get_logger(), "Controller : NavigateToPose goals canceled");
                }
            }
            else 
            {
                RCLCPP_DEBUG(get_logger(), "NavigateToPose action server not available");
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(get_logger(), "Error canceling NavigateToPose goals: %s", e.what());
        }

        // NavigateThroughPoses のキャンセル
        try {
            if (navigate_through_poses_client->wait_for_action_server(std::chrono::seconds(1))) 
            {
                auto cancel_future = navigate_through_poses_client->async_cancel_all_goals();
                auto future_status = cancel_future.wait_for(std::chrono::seconds(1));
                
                if (future_status == std::future_status::ready) 
                {
                    RCLCPP_INFO(get_logger(), "Controller : NavigateThroughPoses goals canceled");
                }
            }
            else 
            {
                RCLCPP_DEBUG(get_logger(), "NavigateThroughPoses action server not available");
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(get_logger(), "Error canceling NavigateThroughPoses goals: %s", e.what());
        }

        // FollowWaypoints のキャンセル
        try {
            if (follow_waypoints_client->wait_for_action_server(std::chrono::seconds(1))) 
            {
                auto cancel_future = follow_waypoints_client->async_cancel_all_goals();
                auto future_status = cancel_future.wait_for(std::chrono::seconds(1));
                
                if (future_status == std::future_status::ready) 
                {
                    RCLCPP_INFO(get_logger(), "Controller : FollowWaypoints goals canceled");
                }
            }
            else 
            {
                RCLCPP_DEBUG(get_logger(), "FollowWaypoints action server not available");
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(get_logger(), "Error canceling FollowWaypoints goals: %s", e.what());
        }

        RCLCPP_INFO(get_logger(), "Controller : All navigation goals cancellation completed");
    }

    void Controller::publishPoseEstimate()
    {
        RCLCPP_INFO(get_logger(), "Controller : 次のウェイポイント先を現在位置に設定します。");

        initial_pose_->header.stamp = this->now();
        initial_pose_->header.frame_id = "map";
        initial_pose_->pose.pose.position.x = target_odom_pose_x;
        initial_pose_->pose.pose.position.y = target_odom_pose_y;
        initial_pose_->pose.pose.position.z = 0.0;
        initial_pose_->pose.pose.orientation.x = 0.0;
        initial_pose_->pose.pose.orientation.y = 0.0;
        initial_pose_->pose.pose.orientation.z = target_odom_orientation_z;
        initial_pose_->pose.pose.orientation.w = target_odom_orientation_w;

        // 共分散を設定（初期位置の不確かさ）
        initial_pose_->pose.covariance[0] = 0.25;   // x の分散 0.5m
        initial_pose_->pose.covariance[7] = 0.25;   // y の分散 0.5m
        initial_pose_->pose.covariance[35] = 0.14;  // yaw の分散 0.3rad

        initial_pose_pub_->publish(*initial_pose_);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(sirius_controller::Controller)