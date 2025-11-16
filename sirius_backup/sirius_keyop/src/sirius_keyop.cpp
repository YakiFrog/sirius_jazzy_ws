#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sirius_interfaces/msg/keyboard_input.hpp>

#include "sirius_keyop/keyop.hpp"

namespace sirius_keyop
{
  KeyOp::KeyOp(const rclcpp::NodeOptions & options) : rclcpp::Node("sirius_keyop", options), last_zero_vel_sent_(true), quit_requested_(false), key_file_descriptor_(0)
  {
    tcgetattr(key_file_descriptor_, &original_terminal_state_);
    cmd_ = std::make_shared<geometry_msgs::msg::Twist>();
    waypoint_ = std::make_shared<std_msgs::msg::Bool>();

    double linear_vel_step = this->declare_parameter("linear_vel_step", 0.01);
    double linear_vel_max = this->declare_parameter("linear_vel_max", 2.0);
    double angular_vel_step = this->declare_parameter("angular_vel_step", 0.01);
    double angular_vel_max = this->declare_parameter("angular_vel_max", 1.2);

    RCLCPP_INFO(get_logger(), "KeyOp : using linear vel step [%f].", linear_vel_step);
    RCLCPP_INFO(get_logger(), "KeyOp : using linear vel max [%f].", linear_vel_max);
    RCLCPP_INFO(get_logger(), "KeyOp : using angular vel step [%f].", angular_vel_step);
    RCLCPP_INFO(get_logger(), "KeyOp : using angular vel max [%f].", angular_vel_max);

    keyinput_subscriber_ = this->create_subscription<sirius_interfaces::msg::KeyboardInput>("teleop", rclcpp::QoS(1), std::bind(&KeyOp::remoteKeyInputReceived, this, std::placeholders::_1));

    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    waypoint_publisher_ = this->create_publisher<std_msgs::msg::Bool>("waypoint", 1); 
    
    RCLCPP_INFO(get_logger(), "KeyOp: connected.");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&KeyOp::spin, this));

    thread_ = std::thread(&KeyOp::keyboardInputLoop, this);
  }

  KeyOp::~KeyOp()
  {
    disable();
    quit_requested_ = true;
    thread_.join();
    tcsetattr(key_file_descriptor_, TCSANOW, &original_terminal_state_);
  }

  void KeyOp::spin()
  {
    std::lock_guard<std::mutex> lk(cmd_mutex_);

    if((cmd_->linear.x != 0.0) || (cmd_->linear.y != 0.0) || (cmd_->linear.z != 0.0) || (cmd_->angular.x != 0.0) || (cmd_->angular.y != 0.0) || (cmd_->angular.z != 0.0))
    {
      velocity_publisher_->publish(*cmd_);
      last_zero_vel_sent_ = false;
    }
    else if(last_zero_vel_sent_ == false)
    {
      velocity_publisher_->publish(*cmd_);
      last_zero_vel_sent_ = true;
    }
  }

  void KeyOp::keyboardInputLoop()
  {
    struct termios raw;
    std::memcpy(&raw, &original_terminal_state_, sizeof(struct termios));

    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOL] = 2;
    tcsetattr(key_file_descriptor_, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------");
    puts("Forward/back arrows : linear velocity incr/decr.");
    puts("Right/left arrows : angular velocity incr/decr.");
    puts("Spacebar : reset linear/angular velocities.");
    puts("d : disable motors.");
    puts("e : enable motors.");
    puts("w : save waypoint.");

    while(!quit_requested_)
    {
      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(key_file_descriptor_, &readfds);
      struct timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 50 * 1000; //50ms
      int select_ret = ::select(key_file_descriptor_ + 1, &readfds, nullptr, nullptr, &timeout);

      if(select_ret < 0)
      {
        //エラー
        RCLCPP_ERROR(get_logger(), "Select failed: %s", ::strerror(errno));
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
          RCLCPP_ERROR(get_logger(), "Invalid file descriptor. ignoring");
        }

        char c;
        
        if(::read(key_file_descriptor_, &c, 1) < 0)
        {
          //エラー
          RCLCPP_ERROR(get_logger(), "Failed to read character: %s", ::strerror(errno));
        }
        else
        {
          processKeyboardInput(c);
        }
      }
    }
  }

  void KeyOp::remoteKeyInputReceived(const std::shared_ptr<sirius_interfaces::msg::KeyboardInput> key)
  {
    processKeyboardInput(key->pressed_key);
  }

  void KeyOp::processKeyboardInput(char c)
  {
    std::lock_guard<std::mutex> lk(cmd_mutex_);

    switch(c)
    {
      case sirius_interfaces::msg::KeyboardInput::KEYCODE_LEFT:
      {
        incrementAngularVelocity();
        break;
      }
      case sirius_interfaces::msg::KeyboardInput::KEYCODE_RIGHT:
      {
        decrementAngularVelocity();
        break;
      }
      case sirius_interfaces::msg::KeyboardInput::KEYCODE_UP:
      {
        incrementLinearVelocity();
        break;
      }
      case sirius_interfaces::msg::KeyboardInput::KEYCODE_DOWN:
      {
        decrementLinearVelocity();
        break;
      }
      case sirius_interfaces::msg::KeyboardInput::KEYCODE_SPACE:
      {
        resetVelocity();
        break;
      }
      case 'd':
      {
        disable();
        break;
      }
      case 'e':
      {
        enable();
        break;
      }
      case 'w':
      {
        waypoint();
        break;
      }
      default:
      {
        break;
      }
    }
  }

  void KeyOp::disable()
  {
    cmd_->linear.x = 0.0;
    cmd_->angular.z = 0.0;
    velocity_publisher_->publish(*cmd_);

    RCLCPP_INFO(get_logger(), "KeyOp: die, die, die (disabling power to the device's)");
  }

  void KeyOp::enable()
  {
    cmd_->linear.x = 0.0;
    cmd_->angular.z = 0.0;
    velocity_publisher_->publish(*cmd_);

    RCLCPP_INFO(get_logger(), "KeyOp: Enable power to the device subsystem.");
  }

  void KeyOp::waypoint()
  {
    waypoint_->data = true;
    waypoint_publisher_->publish(*waypoint_);

    RCLCPP_INFO(get_logger(), "KeyOp: Save waypoint.");
  }

  void KeyOp::incrementLinearVelocity()
  {
    double linear_vel_max = get_parameter("linear_vel_max").get_value<double>();
    double linear_vel_step = get_parameter("linear_vel_step").get_value<double>();

    if(cmd_->linear.x <= linear_vel_max)
    {
      cmd_->linear.x += linear_vel_step;
    }

    RCLCPP_INFO(get_logger(), "KeyOp: linear velocity incremented [%f|%f]", cmd_->linear.x, cmd_->angular.z);
  }

  void KeyOp::decrementLinearVelocity()
  {
    double linear_vel_max = get_parameter("linear_vel_max").get_value<double>();
    double linear_vel_step = get_parameter("linear_vel_step").get_value<double>();

    if(cmd_->linear.x >= -linear_vel_max)
    {
      cmd_->linear.x -= linear_vel_step;
    }

    RCLCPP_INFO(get_logger(), "KeyOp: linear velocity decremented [%f|%f]", cmd_->linear.x, cmd_->angular.z);
  }

  void KeyOp::incrementAngularVelocity()
  {
    double angular_vel_max = get_parameter("angular_vel_max").get_value<double>();
    double angular_vel_step = get_parameter("angular_vel_step").get_value<double>();

    if(cmd_->angular.z <= angular_vel_max)
    {
      cmd_->angular.z += angular_vel_step;
    }

    RCLCPP_INFO(get_logger(), "KeyOp: angular velocity incremented [%f|%f]", cmd_->linear.x, cmd_->angular.z);
  }

  void KeyOp::decrementAngularVelocity()
  {
    double angular_vel_max = get_parameter("angular_vel_max").get_value<double>();
    double angular_vel_step = get_parameter("angular_vel_step").get_value<double>();

    if(cmd_->angular.z >= -angular_vel_max)
    {
      cmd_->angular.z -= angular_vel_step;
    }

    RCLCPP_INFO(get_logger(), "KeyOp: angular velocity decrement [%f|%f]", cmd_->linear.x, cmd_->angular.z);
  }

  void KeyOp::resetVelocity()
  {
    cmd_->angular.z = 0.0;
    cmd_->linear.x = 0.0;

    RCLCPP_INFO(get_logger(), "KeyOp: reset linear/angular velocities.");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(sirius_keyop::KeyOp)
