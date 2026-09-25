#include "roboteq_ros2_driver/roboteq_ros2_driver.hpp"


#include <chrono> 
#include <functional> 
#include <memory>     
#include <string>     
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include <iostream>
#include <cstdlib>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

// dependencies for ROS
#include <serial/serial.h>
#include <signal.h>
#include <string>
#include <sstream>

#define DELTAT(_nowtime, _thentime) ((_thentime > _nowtime) ? ((0xffffffff - _thentime) + _nowtime) : (_nowtime - _thentime))


// Define following to enable cmdvel debug output
#define _CMDVEL_DEBUG




// Define following to enable odom debug output
#define _ODOM_DEBUG

// Define following to publish additional sensor information; comment out to not publish (TODO: write custom roboteq messages to support other reportable data from MC

// #define _ODOM_SENSORS

// Define following to enable service for returning covariance
// #define _ODOM_COVAR_SERVER

#define NORMALIZE(_z) atan2(sin(_z), cos(_z))

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

serial::Serial controller;
uint32_t millis()
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

namespace Roboteq
{
Roboteq::Roboteq() : Node("roboteq_ros2_driver")
// initialize parameters and variables
{
    pub_odom_tf = this->declare_parameter("pub_odom_tf", true);
    odom_frame = this->declare_parameter("odom_frame", "odom");
    base_frame = this->declare_parameter("base_frame", "base_footprint");
    cmdvel_topic = this->declare_parameter("cmdvel_topic", "cmd_vel");
    odom_topic = this->declare_parameter("odom_topic", "odom");
    port = this->declare_parameter("port", "/dev/roboteq");
    baud = this->declare_parameter("baud", 115200);
    open_loop = this->declare_parameter("open_loop", true);
    wheel_circumference = this->declare_parameter("wheel_circumference", 0.877); //タイヤの円周(m)
    track_width = this->declare_parameter("track_width", 0.40); //トレッド幅(m)
    max_amps = this->declare_parameter("max_amps", 12.4);
    max_rpm = this->declare_parameter("max_rpm", 2500);
    gear_ratio = this->declare_parameter("gear_ratio", 1.0); //ギア比
    pulse = this->declare_parameter("pulse", 229); //一周あたりのパルス数
    max_speed = this->declare_parameter("max_speed", 0.5); //最大速度
    // odom streaming/publish tuning
    odom_stream_interval_ms = this->declare_parameter("odom_stream_interval_ms", 50);
    odom_publish_hz = this->declare_parameter("odom_publish_hz", 50.0);
    // Speed scale factor to calibrate actual robot speed to match commanded speed
    // If robot moves faster than commanded, decrease this value (e.g., 0.625 for 60% faster)
    // If robot moves slower than commanded, increase this value (e.g., 1.2 for 20% slower)
    speed_scale = this->declare_parameter("speed_scale", 1.0);
    kp_soft = this->declare_parameter("kp_soft", 0.0);
    min_speed_threshold = this->declare_parameter("min_speed_threshold", 0.10);
    motor_sign_r = this->declare_parameter("motor_sign_r", 1.0);
    motor_sign_l = this->declare_parameter("motor_sign_l", -1.0);
    encoder_sign_r = this->declare_parameter("encoder_sign_r", -1.0);
    encoder_sign_l = this->declare_parameter("encoder_sign_l", 1.0);
    max_encoder_step_revolutions =
        this->declare_parameter("max_encoder_step_revolutions", 0.25);
    publish_status = this->declare_parameter("publish_status", true);
    status_topic = this->declare_parameter("status_topic", std::string("roboteq/status"));
    status_publish_hz = this->declare_parameter("status_publish_hz", 20.0);
    // ソフトウェア閉ループ車輪速度制御
    closed_loop = this->declare_parameter("closed_loop", false);
    cl_kp = this->declare_parameter("cl_kp", 150.0);
    cl_ki = this->declare_parameter("cl_ki", 300.0);
    cl_kd = this->declare_parameter("cl_kd", 0.0);
    cl_control_hz = this->declare_parameter("cl_control_hz", 50.0);
    cl_max_duty = this->declare_parameter("cl_max_duty", 1000);
    cl_anti_windup = this->declare_parameter("cl_anti_windup", 3.0);
    cl_min_speed = this->declare_parameter("cl_min_speed", 0.15);
    cl_duty_slew = this->declare_parameter("cl_duty_slew", 3000.0);
    cl_duty_lpf_tau = this->declare_parameter("cl_duty_lpf_tau", 0.08);
    cl_feedback_alpha = this->declare_parameter("cl_feedback_alpha", 0.5);

    starttime = 0;
    hstimer = 0;
    mstimer = 0;
    odom_idx = 0;
    odom_encoder_toss = 5;
    odom_encoder_left = 0;
    odom_encoder_right = 0;
    odom_x = 0.0;
    odom_y = 0.0;
    odom_yaw = 0.0;
    odom_last_x = 0.0;
    odom_last_y = 0.0;
    odom_last_yaw = 0.0;
    odom_last_time = 0;
    first_time = true;
    right_rpm_command = 0.0;
    left_rpm_command = 0.0;
    linear_x = 0.0;
    angular_z = 0.0;
    running_ = true;
    
    odom_thread_ = std::thread(&Roboteq::odom_loop, this);

    //odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
    odom_msg = nav_msgs::msg::Odometry();

    serial::Timeout timeout = serial::Timeout::simpleTimeout(50); // タイムアウトを短く設定
    controller.setPort(port);
    controller.setBaudrate(baud);
    controller.setTimeout(timeout);
    // connect to serial port
    connect();
    // configure motor controller
    cmdvel_setup();
    odom_setup();
//
//  odom publisher
//
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1000);
    if (publish_status)
    {
        status_pub =
            this->create_publisher<roboteq_ros2_driver::msg::RoboteqStatus>(status_topic, 10);
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Drive config: wheel_circumference=%.4f pulse=%d gear_ratio=%.3f speed_scale=%.4f "
        "motor_sign=(R=%.0f,L=%.0f) encoder_sign=(R=%.0f,L=%.0f)",
        wheel_circumference, pulse, gear_ratio, speed_scale,
        motor_sign_r, motor_sign_l, encoder_sign_r, encoder_sign_l);
//
// cmd_vel subscriber
//

    cmdvel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        cmdvel_topic, // topic name
        1000,         // QoS history depth
        std::bind(&Roboteq::cmdvel_callback, this, std::placeholders::_1));

    stop_sub = this->create_subscription<std_msgs::msg::Bool>(
        "stop",
        10,
        std::bind(&Roboteq::bumper_callback, this, std::placeholders::_1)
    );
    
    using namespace std::chrono_literals;
    // set odometry publishing loop timer at 10Hz
    timer_ = this->create_wall_timer(10ms,std::bind(&Roboteq::run, this));
    // ソフト閉ループ制御タイマ (closed_loop=false の間は control_loop が即return)
    {
        double chz = (cl_control_hz > 1.0) ? cl_control_hz : 50.0;
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / chz)),
            std::bind(&Roboteq::control_loop, this));
    }
    // enable modifying params at run-time
    odom_baselink_transform_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    /*    
    using namespace std::chrono_literals;
    */
    param_update_timer = this->create_wall_timer(1000ms, std::bind(&Roboteq::update_parameters, this));
    
}

void Roboteq::update_parameters()
{
    this->get_parameter("pub_odom_tf", pub_odom_tf);
    this->get_parameter("odom_frame", odom_frame);
    this->get_parameter("base_frame", base_frame);
    this->get_parameter("cmdvel_topic", cmdvel_topic);
    this->get_parameter("odom_topic", odom_topic);
    this->get_parameter("port", port);
    this->get_parameter("baud", baud);
    this->get_parameter("open_loop", open_loop);
    this->get_parameter("wheel_circumference", wheel_circumference);
    this->get_parameter("track_width", track_width);
    this->get_parameter("max_amps", max_amps);
    this->get_parameter("max_rpm", max_rpm);
    this->get_parameter("gear_ratio", gear_ratio);
    this->get_parameter("pulse", pulse);
    this->get_parameter("max_speed", max_speed);
    // read odom timing params
    int new_stream_ms = odom_stream_interval_ms;
    this->get_parameter("odom_stream_interval_ms", new_stream_ms);
    this->get_parameter("odom_publish_hz", odom_publish_hz);
    this->get_parameter("speed_scale", speed_scale);
    this->get_parameter("kp_soft", kp_soft);
    this->get_parameter("min_speed_threshold", min_speed_threshold);
    this->get_parameter("motor_sign_r", motor_sign_r);
    this->get_parameter("motor_sign_l", motor_sign_l);
    this->get_parameter("encoder_sign_r", encoder_sign_r);
    this->get_parameter("encoder_sign_l", encoder_sign_l);
    this->get_parameter(
        "max_encoder_step_revolutions", max_encoder_step_revolutions);
    this->get_parameter("publish_status", publish_status);
    this->get_parameter("status_topic", status_topic);
    this->get_parameter("status_publish_hz", status_publish_hz);
    bool prev_closed_loop = closed_loop;
    this->get_parameter("closed_loop", closed_loop);
    this->get_parameter("cl_kp", cl_kp);
    this->get_parameter("cl_ki", cl_ki);
    this->get_parameter("cl_kd", cl_kd);
    this->get_parameter("cl_control_hz", cl_control_hz);
    this->get_parameter("cl_max_duty", cl_max_duty);
    this->get_parameter("cl_anti_windup", cl_anti_windup);
    this->get_parameter("cl_min_speed", cl_min_speed);
    this->get_parameter("cl_duty_slew", cl_duty_slew);
    this->get_parameter("cl_duty_lpf_tau", cl_duty_lpf_tau);
    this->get_parameter("cl_feedback_alpha", cl_feedback_alpha);
    if (closed_loop && !prev_closed_loop) {
        cl_integral_r_ = 0.0;
        cl_integral_l_ = 0.0;
        last_duty_r_ = 0.0;
        last_duty_l_ = 0.0;
        filtered_duty_r_ = 0.0;
        filtered_duty_l_ = 0.0;
        // 有効化時に古い目標が残っていると勝手に走るため0にリセット
        std::lock_guard<std::mutex> lock(speed_mutex_);
        target_right_speed_ = 0.0;
        target_left_speed_ = 0.0;
    }
    // If the stream interval changed while running, re-send the stream
    // configuration to the device so it starts using the new rate sooner.
    if (new_stream_ms != odom_stream_interval_ms) {
        odom_stream_interval_ms = new_stream_ms;
        try {
            odom_stream();
            last_sent_odom_stream_ms = odom_stream_interval_ms;
            RCLCPP_INFO(this->get_logger(), "Updated odom stream interval to %d ms and re-sent stream command", odom_stream_interval_ms);
        } catch (const std::exception &e) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Failed to re-send odom stream config: " << e.what());
        }
    }
}

void Roboteq::connect() {
    RCLCPP_INFO_STREAM(this->get_logger(), "Opening serial port on " << port << " at " << baud << "...");
    
    // 接続前に閉じておく
    if (controller.isOpen()) {
        try {
            controller.close();
        } catch (const std::exception &e) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Error closing port: " << e.what());
        }
    }
    
    int retry_count = 0;
    const int max_retries = 5;
    
    while (retry_count < max_retries) {
        try {
            controller.open();
            if (controller.isOpen()) {
                RCLCPP_INFO(this->get_logger(), "Successfully opened serial port");
                
                // シリアルポートの設定を最適化
                controller.setFlowcontrol(serial::flowcontrol_none);  // フロー制御を無効化
                controller.setBytesize(serial::eightbits);  // 8ビット
                controller.setParity(serial::parity_none);  // パリティなし
                controller.setStopbits(serial::stopbits_one);  // ストップビット1
                
                return;
            }
        } catch (const serial::IOException &e) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Serial::IOException: " << e.what());
        } catch (const std::exception &e) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Exception: " << e.what());
        }
        
        retry_count++;
        RCLCPP_WARN_STREAM(this->get_logger(), "Failed to open serial port, retry " << retry_count << " of " << max_retries << "...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port after maximum retries");
}

void Roboteq::bumper_callback(const std_msgs::msg::Bool::SharedPtr stop_msg)
{
    if (stop_msg->data)
    {
        RCLCPP_INFO(this->get_logger(), "Bumper!!!");
        safe_serial_write("!EX 1\r");
    }

    else
    {
        RCLCPP_INFO(this->get_logger(), "Restart!!!");
        // Releasing emergency stop. Reset command values to 0 to prevent the controller
        // from resuming previous non-zero velocities from its memory.
        safe_serial_write("!G 1 0\r");
        safe_serial_write("!G 2 0\r");
        safe_serial_write("!MG\r");
        linear_x = 0.0;
        angular_z = 0.0;
    }
}

// シリアルポートへの安全な書き込み関数を追加
bool Roboteq::safe_serial_write(const std::string &cmd) {
    if (!controller.isOpen()) {
        RCLCPP_WARN(this->get_logger(), "Cannot write to closed port. Attempting to reconnect...");
        try {
            connect();
        } catch (const std::exception &e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to reconnect: " << e.what());
            return false;
        }
    }
    
    try {
        controller.write(cmd);
        return true;
    } catch (const serial::IOException &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Serial write failed: " << e.what());
        return false;
    } catch (const serial::PortNotOpenedException &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Port not opened: " << e.what());
        return false;
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown error during serial write: " << e.what());
        return false;
    }
}

void Roboteq::cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
    // 機体座標系での左右車輪速度 (m/s)。モーター結線の方向は
    // motor_sign_r/l で機体ごとに変換する。
    float right_wheel_speed =
        twist_msg->linear.x + track_width * twist_msg->angular.z / 2.0;
    float left_wheel_speed =
        twist_msg->linear.x - track_width * twist_msg->angular.z / 2.0;

    linear_x = twist_msg->linear.x;
    angular_z = twist_msg->angular.z;

    // 閉ループ時は目標車輪速度(物理m/s, speed_scale前)を保存するだけ。
    // 実際の!G送出は control_loop() がエンコーダ速度をフィードバックして行う。
    if (closed_loop) {
        // デッドバンド補償: 到達不能な低速(デッドバンド以下)を追わせない。
        // 左右の比率を保ったまま最大速度を下限まで底上げする。
        float max_abs = std::max(std::abs(right_wheel_speed), std::abs(left_wheel_speed));
        if (max_abs > 1e-6f && max_abs < static_cast<float>(cl_min_speed)) {
            float boost = static_cast<float>(cl_min_speed) / max_abs;
            right_wheel_speed *= boost;
            left_wheel_speed *= boost;
        }
        std::lock_guard<std::mutex> lock(speed_mutex_);
        target_right_speed_ = right_wheel_speed;
        target_left_speed_ = left_wheel_speed;
        return;
    }

    float min_speed_thresh = (min_speed_threshold > 0.0) ? min_speed_threshold : 0.10f;
    constexpr float EPSILON = 1e-6f;
    float scale = (speed_scale > 0.0) ? speed_scale : 1.0;
    float min_threshold = min_speed_thresh * scale;

    // speed_scaleを先に適用する。
    // これにより、不感帯（デッドバンド）の判定を実際にモーターへ送る最終的な指令値ベースで行える。
    right_wheel_speed *= scale;
    left_wheel_speed *= scale;

    // 指令値がゼロでない場合のみ、低速時の底上げ処理を行う
    if ((std::abs(linear_x) > EPSILON) || (std::abs(angular_z) > EPSILON))
    {
        float max_abs_speed =
            std::max(std::abs(right_wheel_speed), std::abs(left_wheel_speed));
        
        // 最大速度がしきい値（スケーリング済み）未満かつゼロでない場合、比率を維持したまま底上げ
        if (max_abs_speed > EPSILON && max_abs_speed < min_threshold)
        {
            float boost_factor = min_threshold / max_abs_speed;
            right_wheel_speed *= boost_factor;
            left_wheel_speed *= boost_factor;
        }
        // 指令はあるが計算上の車輪速度がほぼゼロになる場合（例：低速旋回のみなど）
        else if (max_abs_speed < EPSILON)
        {
            if (linear_x > EPSILON) {
                right_wheel_speed = min_threshold;
                left_wheel_speed = min_threshold;
            } else if (linear_x < -EPSILON) {
                right_wheel_speed = -min_threshold;
                left_wheel_speed = -min_threshold;
            } else {
                // 旋回指令のみの場合
                if (angular_z > 0) {
                    right_wheel_speed = min_threshold;
                    left_wheel_speed = -min_threshold;
                } else {
                    right_wheel_speed = -min_threshold;
                    left_wheel_speed = min_threshold;
                }
            }
        }
    }

    const float right_speed =
        static_cast<float>(motor_sign_r) * right_wheel_speed;
    const float left_speed =
        static_cast<float>(motor_sign_l) * left_wheel_speed;

    std::stringstream left_cmd;
    std::stringstream right_cmd;
    
    float rpm_r = right_speed / wheel_circumference * 60.0;
    float rpm_l = left_speed / wheel_circumference * 60.0;

    // モーター出力指令 (0-1000 スケール)
    int32_t right_power = rpm_r / max_rpm * 1000.0;
    int32_t left_power = rpm_l / max_rpm * 1000.0;
    
    right_rpm_command = rpm_r;
    left_rpm_command = rpm_l;
    
    right_cmd << "!G 1 " << right_power << "\r";
    left_cmd << "!G 2 " << left_power << "\r";
    // モーターコントローラへコマンドを送信
    #ifndef _CMDVEL_FORCE_RUN
        safe_serial_write(right_cmd.str());
        safe_serial_write(left_cmd.str());
        try {
            if (controller.isOpen()) {
                controller.flush();
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in cmdvel_callback flush: " << e.what());
        }
        
        // 送信したモーターコマンドをログ出力
        // RCLCPP_INFO(this->get_logger(), "cmdvel: linear=%.3f angular=%.3f -> R=%d L=%d (%s)",
        //             linear_x, angular_z, right_power_or_rpm, left_power_or_rpm,
        //             open_loop ? "power" : "rpm");
    #endif
}
void Roboteq::cmdvel_setup()
{
    // stop motors
    safe_serial_write("!G 1 0\r");
    safe_serial_write("!G 2 0\r");
    safe_serial_write("!S 1 0\r");
    safe_serial_write("!S 2 0\r");
    
    if (!controller.isOpen()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port not open, cannot complete setup");
        return;
    }
    
    try {
        controller.flush();

        // disable echo
        safe_serial_write("^ECHOF 1\r");
        controller.flush();

        // enable watchdog timer (1000 ms)
        safe_serial_write("^RWD 1000\r");

        // set motor operating mode (0 for open-loop speed)
        safe_serial_write("^MMOD 1 0\r");
        safe_serial_write("^MMOD 2 0\r");

        // set motor amps limit (A * 10)
        std::stringstream right_ampcmd;
        std::stringstream left_ampcmd;
        right_ampcmd << "^ALIM 1 " << (int)(max_amps * 10) << "\r";
        left_ampcmd << "^ALIM 2 " << (int)(max_amps * 10) << "\r";
        safe_serial_write(right_ampcmd.str());
        safe_serial_write(left_ampcmd.str());

        // set max speed (rpm) for relative speed commands
        std::stringstream right_rpmcmd;
        std::stringstream left_rpmcmd;
        right_rpmcmd << "^MXRPM 1 " << max_rpm << "\r";
        left_rpmcmd << "^MXRPM 2 " << max_rpm << "\r";
        safe_serial_write(right_rpmcmd.str());
        safe_serial_write(left_rpmcmd.str());

        // set max acceleration rate (2000 rpm/s * 10)
        safe_serial_write("^MAC 1 20000\r");
        safe_serial_write("^MAC 2 20000\r");

        // set max deceleration rate (2000 rpm/s * 10)
        safe_serial_write("^MDEC 1 20000\r");
        safe_serial_write("^MDEC 2 20000\r");

        // set open-loop acceleration and deceleration ramps (value in 0.1s to go from 0 to 100% power)
        safe_serial_write("^MXACC 1 12\r");
        safe_serial_write("^MXACC 2 12\r");
        safe_serial_write("^MXDEC 1 12\r");
        safe_serial_write("^MXDEC 2 12\r");

        // set PID parameters (gain * 10)
        safe_serial_write("^KP 1 0\r");
        safe_serial_write("^KP 2 0\r");
        safe_serial_write("^KI 1 0\r");
        safe_serial_write("^KI 2 0\r");
        safe_serial_write("^KD 1 0\r");
        safe_serial_write("^KD 2 0\r");

        // set encoder mode (18 for feedback on motor1, 34 for feedback on motor2)
        safe_serial_write("^EMOD 1 18\r");
        safe_serial_write("^EMOD 2 34\r");

        // set encoder counts (ppr)
        std::stringstream right_enccmd;
        std::stringstream left_enccmd;
        right_enccmd << "^EPPR 1 " << pulse << "\r";
        left_enccmd << "^EPPR 2 " << pulse << "\r";
        safe_serial_write(right_enccmd.str());
        safe_serial_write(left_enccmd.str());

        controller.flush();
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in cmdvel_setup: " << e.what());
    }
}

void Roboteq::cmdvel_loop()
{
}

void Roboteq::cmdvel_run()
{
#ifdef _CMDVEL_FORCE_RUN
    if (open_loop)
    {
    controller.write("!G 1 100\r");
    controller.write("!G 2 100\r");
    }
    else
    {
    std::stringstream right_cmd;
    std::stringstream left_cmd;
    right_cmd << "!S 1 " << (int)(max_rpm * 0.1) << "\r";
    left_cmd << "!S 2 " << (int)(max_rpm * 0.1) << "\r";
    controller.write(right_cmd.str());
    controller.write(left_cmd.str());
    }
    controller.flush();
#endif
}


void Roboteq::odom_setup()
{
    odom_baselink_transform_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    if (pub_odom_tf)
    {
        odom_msg.header.stamp = this->get_clock()->now();
    
        odom_msg.header.frame_id = odom_frame;
        odom_msg.child_frame_id = base_frame;

        // Set up the pose covariance
        for (size_t i = 0; i < 36; i++)
        {
            odom_msg.pose.covariance[i] = 0;
            odom_msg.twist.covariance[i] = 0;
        }

        odom_msg.pose.covariance[7] = 0.001;
        odom_msg.pose.covariance[14] = 1000000;
        odom_msg.pose.covariance[21] = 1000000;
        odom_msg.pose.covariance[28] = 1000000;
        odom_msg.pose.covariance[35] = 1000;

        // Set up the twist covariance
        odom_msg.twist.covariance[0] = 0.001;
        odom_msg.twist.covariance[7] = 0.001;
        odom_msg.twist.covariance[14] = 1000000;
        odom_msg.twist.covariance[21] = 1000000;
        odom_msg.twist.covariance[28] = 1000000;
        odom_msg.twist.covariance[35] = 1000;

        // Set up the transform message: move to odom_publish
    
        tf2::Quaternion q;
        q.setRPY(0, 0, odom_yaw);

        // start encoder streaming
        odom_stream();
        last_sent_odom_stream_ms = odom_stream_interval_ms;
    
        odom_last_time = millis();
    }
#ifdef _ODOM_SENSORS
    current_last_time = millis();
#endif
}

// Odom msg streams

void Roboteq::odom_stream()
{

#ifdef _ODOM_SENSORS
    // start encoder and current output (30 hz)
    // doubling frequency since one value is output at each cycle
    //  controller.write("# C_?CR_?BA_# 17\r");
    // start encoder, current and voltage output (30 hz)
    // tripling frequency since one value is output at each cycle
    safe_serial_write("# C_?CB_?BA_?V_# 11\r");
#else
    // use configured streaming interval (ms) - Roboteq expects the interval
    // in ms in this command (e.g. 50 -> 20Hz). Make sure value is sane.
    int stream_ms = odom_stream_interval_ms;
    if (stream_ms <= 0) stream_ms = 50;
    std::stringstream ss;
    if (publish_status) {
        // Roboteqのストリームはラウンドロビン(1周期に1クエリ)。全テレメトリを
        // 既定5ms間隔で回すと各項目が約29Hzになり、CBも20Hzのodom配信に足りる。
        ss << "# C_?CB_?V_?A_?BA_?BS_?FF_?T_# " << stream_ms << "\r";
    } else {
        ss << "# C_?CB_# " << stream_ms << "\r";
    }
    safe_serial_write(ss.str());
    
    // オプション: より小さなデータフォーマットを使用
    // 注: お使いのRoboteqコントローラがこのコマンドをサポートしているか確認してください
    // safe_serial_write("# _?CR_# 500\r");  // 生カウンタデータ (より小さなデータサイズ)
#endif
    
    try {
        if (controller.isOpen()) {
            controller.flush();
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in odom_stream flush: " << e.what());
    }
}

void Roboteq::odom_loop()
{
    const size_t buffer_size = 1024;  // バッファサイズを大きく
    uint8_t buffer[buffer_size];
    size_t bytes_read = 0;
    std::string line_buffer;
    
    // データ処理のパフォーマンス計測用
    uint32_t last_diagnostic_time = millis();
    size_t total_bytes = 0;
    int packet_count = 0;
    
    // ストリーミング設定再送用のタイミング変数（odom_publishとは別）
    uint32_t stream_last_time = millis();
    
    while (running_) {
        // ストリーミング設定の再送間隔を30秒に長く
        uint32_t nowtime = millis();
        if (DELTAT(nowtime, stream_last_time) >= 30000) {
            odom_stream();
            stream_last_time = nowtime;
            
            // データレート診断を表示
            float elapsed_secs = DELTAT(nowtime, last_diagnostic_time) / 1000.0f;
            if (elapsed_secs > 0 && packet_count > 0) {
                float bytes_per_sec = total_bytes / elapsed_secs;
                RCLCPP_INFO(this->get_logger(), "Odometry data rate: %.2f KB/s, packets: %d", 
                            bytes_per_sec / 1024.0f, packet_count);
                total_bytes = 0;
                packet_count = 0;
                last_diagnostic_time = nowtime;
            }
        }

        if (controller.available()) {
            try {
                // 一度に複数バイトを読み取る
                bytes_read = controller.read(buffer, buffer_size);
                total_bytes += bytes_read;
                
                // 読み取ったデータをより効率的に処理
                for (size_t i = 0; i < bytes_read; i++) {
                    char ch = buffer[i];
                    
                    if (ch == '\r') {
                        packet_count++;
                        
                        // 行の終わり - データを処理
                        if (line_buffer.find('=') != std::string::npos) {
                            process_telemetry(line_buffer);
                        }
                        
                        // バッファをクリア
                        line_buffer.clear();
                    } else if (line_buffer.length() < 64) {  // バッファオーバーフロー防止
                        line_buffer += ch;
                    }
                }
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Exception reading serial data: " << e.what());
            }
        }

        // スリープ時間を短縮（CPUの空回しを防ぎつつも、応答性を保つ）
        std::this_thread::sleep_for(std::chrono::microseconds(100));  // 0.1 ms
    }
}

void Roboteq::process_encoder_data(int32_t right_val, int32_t left_val)
{
    rclcpp::Time now = this->get_clock()->now();

    odom_encoder_right = right_val;
    odom_encoder_left = left_val;

    if (!has_last_encoder_time_) {
        odom_encoder_right_old = right_val;
        odom_encoder_left_old = left_val;
        last_encoder_time_ = now;
        last_odom_update_time_ = now;
        has_last_encoder_time_ = true;
        return;
    }

    double gr = (gear_ratio <= 0.0f) ? 1.0 : gear_ratio;
    double counts_per_wheel_rev = static_cast<double>(pulse) * gr;
    if (counts_per_wheel_rev <= 0.0) {
        return;
    }

    // Keep the absolute counters as integers. Converting a long-running 32-bit
    // counter to float loses single-count precision and creates pose jumps.
    auto wrapped_count_delta = [](int32_t current, int32_t previous) {
        int64_t delta =
            static_cast<int64_t>(current) - static_cast<int64_t>(previous);
        constexpr int64_t COUNTER_RANGE = (int64_t{1} << 32);
        if (delta > INT32_MAX) {
            delta -= COUNTER_RANGE;
        } else if (delta < INT32_MIN) {
            delta += COUNTER_RANGE;
        }
        return delta;
    };

    const int64_t right_count_delta =
        wrapped_count_delta(odom_encoder_right, odom_encoder_right_old);
    const int64_t left_count_delta =
        wrapped_count_delta(odom_encoder_left, odom_encoder_left_old);

    odom_encoder_right_old = odom_encoder_right;
    odom_encoder_left_old = odom_encoder_left;
    last_encoder_time_ = now;

    const double right_diff =
        static_cast<double>(right_count_delta) / counts_per_wheel_rev;
    const double left_diff =
        static_cast<double>(left_count_delta) / counts_per_wheel_rev;

    // A discontinuous controller counter must not teleport the robot or inject
    // an extreme velocity into robot_localization.
    const double max_step =
        max_encoder_step_revolutions > 0.0 ?
        max_encoder_step_revolutions : 0.25;
    if (std::abs(right_diff) > max_step || std::abs(left_diff) > max_step) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Ignoring encoder counter jump: right=%ld left=%ld "
            "(%.3f/%.3f wheel rev)",
            static_cast<long>(right_count_delta),
            static_cast<long>(left_count_delta), right_diff, left_diff);
        return;
    }

    const float d_roll_r = static_cast<float>(encoder_sign_r * right_diff);
    const float d_roll_l = static_cast<float>(encoder_sign_l * left_diff);

    // 車輪回転量から実走行距離・旋回角を算出
    float d_linear = (d_roll_r + d_roll_l) * static_cast<float>(wheel_circumference) / 2.0f;
    float d_angular = (d_roll_r - d_roll_l) * static_cast<float>(wheel_circumference) / static_cast<float>(track_width);

    {
        std::lock_guard<std::mutex> lock(odom_mutex_);

        // Accumulate wheel motion for a stable publish-period velocity. The
        // EKF consumes this twist, so per-packet timing jitter must not leak in.
        odom_roll_right += d_roll_r;
        odom_roll_left += d_roll_l;

        float mid_yaw = odom_yaw + d_angular / 2.0f;
        odom_x += d_linear * std::cos(mid_yaw);
        odom_y += d_linear * std::sin(mid_yaw);
        odom_yaw = NORMALIZE(odom_yaw + d_angular);

        odom_last_x = odom_x;
        odom_last_y = odom_y;
        odom_last_yaw = odom_yaw;

        last_odom_update_time_ = now;
    }
}

void Roboteq::process_telemetry(const std::string &line)
{
    const size_t eq = line.find('=');
    if (eq == std::string::npos) {
        return;
    }
    const std::string key = line.substr(0, eq);
    const std::string val = line.substr(eq + 1);

    std::vector<double> nums;
    size_t start = 0;
    while (start <= val.size()) {
        const size_t sep = val.find(':', start);
        const std::string tok =
            (sep == std::string::npos) ? val.substr(start) : val.substr(start, sep - start);
        if (!tok.empty()) {
            try {
                nums.push_back(std::stod(tok));
            } catch (const std::exception &) {
                // 数値でないトークンは無視
            }
        }
        if (sep == std::string::npos) {
            break;
        }
        start = sep + 1;
    }

    if (key == "CB") {
        if (odom_encoder_toss > 0) {
            --odom_encoder_toss;
            return;
        }
        if (nums.size() >= 2) {
            process_encoder_data(
                static_cast<int32_t>(nums[0]), static_cast<int32_t>(nums[1]));
        }
        return;
    }

    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    if (key == "V") {
        if (nums.size() >= 1) tel_voltage_internal_ = nums[0] * 0.1;
        if (nums.size() >= 2) tel_voltage_ = nums[1] * 0.1;
        if (nums.size() >= 3) tel_voltage_5v_ = nums[2] * 0.001;
    } else if (key == "A") {
        if (nums.size() >= 1) tel_current_ch1_ = nums[0] * 0.1;
        if (nums.size() >= 2) tel_current_ch2_ = nums[1] * 0.1;
    } else if (key == "BA") {
        if (nums.size() >= 1) tel_battery_current_ch1_ = nums[0] * 0.1;
        if (nums.size() >= 2) tel_battery_current_ch2_ = nums[1] * 0.1;
    } else if (key == "BS") {
        if (nums.size() >= 1) tel_rpm_ch1_ = nums[0];
        if (nums.size() >= 2) tel_rpm_ch2_ = nums[1];
    } else if (key == "T") {
        if (nums.size() >= 1) tel_temperature_ = nums[0];
    } else if (key == "FF") {
        if (nums.size() >= 1) tel_fault_flags_ = static_cast<int>(nums[0]);
    } else {
        return;
    }
    telemetry_valid_ = true;
}

void Roboteq::status_publish()
{
    if (!publish_status || !status_pub) {
        return;
    }
    static rclcpp::Time last_publish_time(0, 0, RCL_ROS_TIME);
    static bool first_publish = true;
    const rclcpp::Time current_time = this->get_clock()->now();
    if (first_publish) {
        first_publish = false;
        last_publish_time = current_time;
    }
    double publish_interval = 0.05;
    if (status_publish_hz > 1e-6) {
        publish_interval = 1.0 / status_publish_hz;
    }
    if ((current_time - last_publish_time).seconds() < publish_interval) {
        return;
    }
    last_publish_time = current_time;

    roboteq_ros2_driver::msg::RoboteqStatus msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = base_frame;
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        msg.voltage = static_cast<float>(tel_voltage_);
        msg.voltage_internal = static_cast<float>(tel_voltage_internal_);
        msg.voltage_5v = static_cast<float>(tel_voltage_5v_);
        msg.current_ch1 = static_cast<float>(tel_current_ch1_);
        msg.current_ch2 = static_cast<float>(tel_current_ch2_);
        msg.battery_current_ch1 = static_cast<float>(tel_battery_current_ch1_);
        msg.battery_current_ch2 = static_cast<float>(tel_battery_current_ch2_);
        msg.rpm_ch1 = static_cast<float>(tel_rpm_ch1_);
        msg.rpm_ch2 = static_cast<float>(tel_rpm_ch2_);
        msg.temperature = static_cast<float>(tel_temperature_);
        msg.fault_flags = tel_fault_flags_;
    }
    status_pub->publish(msg);
}

double Roboteq::open_loop_duty(double wheel_speed_mps) const
{
    // 既存のオープンループ指令と同一のマッピング(FFとして使う)
    const double scale = (speed_scale > 0.0) ? speed_scale : 1.0;
    const double ws = wheel_speed_mps * scale;
    const double rpm = ws / wheel_circumference * 60.0;
    const double maxrpm = (max_rpm > 0) ? static_cast<double>(max_rpm) : 1.0;
    return rpm / maxrpm * 1000.0;
}

void Roboteq::control_loop()
{
    if (!closed_loop) {
        return;
    }
    const rclcpp::Time now = this->get_clock()->now();
    static rclcpp::Time last_time(0, 0, RCL_ROS_TIME);
    static bool first = true;
    if (first) {
        first = false;
        last_time = now;
    }
    double dt = (now - last_time).seconds();
    last_time = now;
    if (!(dt > 0.0) || dt > 0.5) {
        dt = 1.0 / ((cl_control_hz > 1.0) ? cl_control_hz : 50.0);
    }

    double target_r, target_l, actual_r, actual_l;
    {
        std::lock_guard<std::mutex> lock(speed_mutex_);
        target_r = target_right_speed_;
        target_l = target_left_speed_;
        actual_r = actual_right_speed_;
        actual_l = actual_left_speed_;
    }

    const double err_r = target_r - actual_r;
    const double err_l = target_l - actual_l;
    // 目標がほぼ0なら積分をリセット（停止時に電流を残さない）
    if (std::abs(target_r) < 0.02 && std::abs(target_l) < 0.02) {
        cl_integral_r_ = 0.0;
        cl_integral_l_ = 0.0;
    }
    // 積分ワインドアップ対策: 積分の寄与が最大400 dutyを超えないようクランプ
    const double i_limit = (cl_ki > 1e-6) ? (400.0 / cl_ki) : cl_anti_windup;
    cl_integral_r_ = std::max(-i_limit, std::min(i_limit, cl_integral_r_ + err_r * dt));
    cl_integral_l_ = std::max(-i_limit, std::min(i_limit, cl_integral_l_ + err_l * dt));

    double duty_r = open_loop_duty(target_r) + cl_kp * err_r + cl_ki * cl_integral_r_;
    double duty_l = open_loop_duty(target_l) + cl_kp * err_l + cl_ki * cl_integral_l_;
    // duty出力のローパス: 高周波の速度ノイズを追わせない(速度振動抑制)
    if (cl_duty_lpf_tau > 1e-6) {
        const double a = 1.0 - std::exp(-dt / cl_duty_lpf_tau);
        filtered_duty_r_ += (duty_r - filtered_duty_r_) * a;
        filtered_duty_l_ += (duty_l - filtered_duty_l_) * a;
    } else {
        filtered_duty_r_ = duty_r;
        filtered_duty_l_ = duty_l;
    }
    duty_r = filtered_duty_r_;
    duty_l = filtered_duty_l_;
    const double dmax = (cl_max_duty > 0) ? static_cast<double>(cl_max_duty) : 1000.0;
    duty_r = std::max(-dmax, std::min(dmax, duty_r));
    duty_l = std::max(-dmax, std::min(dmax, duty_l));
    // duty変化のスルーレート制限: 急なduty低下による回生スパイク(FF=2)を抑制
    if (cl_duty_slew > 1e-6) {
        const double step = cl_duty_slew * dt;
        duty_r = std::max(last_duty_r_ - step, std::min(last_duty_r_ + step, duty_r));
        duty_l = std::max(last_duty_l_ - step, std::min(last_duty_l_ + step, duty_l));
    }
    last_duty_r_ = duty_r;
    last_duty_l_ = duty_l;

    const int ch1 = static_cast<int>(std::lround(motor_sign_r * duty_r));
    const int ch2 = static_cast<int>(std::lround(motor_sign_l * duty_l));

    std::stringstream cmd_r;
    std::stringstream cmd_l;
    cmd_r << "!G 1 " << ch1 << "\r";
    cmd_l << "!G 2 " << ch2 << "\r";
    safe_serial_write(cmd_r.str());
    safe_serial_write(cmd_l.str());
}

void Roboteq::odom_publish()
{
    static rclcpp::Time last_publish_time(0, 0, RCL_ROS_TIME);
    static bool first_publish = true;
    rclcpp::Time current_time = this->get_clock()->now();
    
    if (first_publish) {
        first_publish = false;
        last_publish_time = current_time;
    }
    
    // Odometryメッセージの発行頻度を制限
    double publish_interval = 0.02; // default 50Hz
    if (odom_publish_hz > 1e-6) {
        publish_interval = 1.0 / odom_publish_hz;
    }
    if ((current_time - last_publish_time).seconds() < publish_interval) {
        return;
    }
    
    const double publish_dt =
        (current_time - last_publish_time).seconds();
    last_publish_time = current_time;
    
    float x = 0.0f, y = 0.0f, yaw = 0.0f, v = 0.0f, w = 0.0f;
    float roll_right = 0.0f, roll_left = 0.0f;
    bool encoder_stale = false;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        x = odom_x;
        y = odom_y;
        yaw = odom_yaw;
        roll_right = odom_roll_right;
        roll_left = odom_roll_left;
        odom_roll_right = 0.0f;
        odom_roll_left = 0.0f;
        encoder_stale =
            (current_time - last_odom_update_time_).seconds() > 0.3;
    }

    // ソフト閉ループ用: odomと同じpublish期間から左右車輪速度(m/s, 前進正)を求める。
    // odomのtwistと同じ量を使うことで、閉ループがodom速度を目標に一致させる。
    {
        std::lock_guard<std::mutex> lock(speed_mutex_);
        if (!encoder_stale && publish_dt > 1e-6 && publish_dt <= 1.0) {
            const double v_r = roll_right * wheel_circumference / publish_dt;
            const double v_l = roll_left * wheel_circumference / publish_dt;
            // 車輪速度フィードバックの平滑化係数(パラメータ化)
            const double alpha =
                (cl_feedback_alpha > 0.0 && cl_feedback_alpha <= 1.0) ? cl_feedback_alpha : 0.5;
            actual_right_speed_ += (v_r - actual_right_speed_) * alpha;
            actual_left_speed_ += (v_l - actual_left_speed_) * alpha;
        } else if (encoder_stale) {
            actual_right_speed_ = 0.0;
            actual_left_speed_ = 0.0;
        }
    }

    const float linear_delta =
        (roll_right + roll_left) *
        static_cast<float>(wheel_circumference) / 2.0f;
    const float angular_delta =
        (roll_right - roll_left) *
        static_cast<float>(wheel_circumference) /
        static_cast<float>(track_width);

    // Restore the pre-fae7459 behavior: compute velocity from all encoder
    // movement accumulated during one odometry publish period, not from the
    // timing of an individual serial packet. Use the former five-sample moving
    // average as well, since this twist is integrated by robot_localization.
    if (!encoder_stale && publish_dt > 1e-6 && publish_dt <= 1.0) {
        const float raw_v = linear_delta / static_cast<float>(publish_dt);
        const float raw_w = angular_delta / static_cast<float>(publish_dt);

        velocity_history_v_.push_back(raw_v);
        velocity_history_w_.push_back(raw_w);
        if (velocity_history_v_.size() > 5) {
            velocity_history_v_.erase(velocity_history_v_.begin());
            velocity_history_w_.erase(velocity_history_w_.begin());
        }

        for (const float sample : velocity_history_v_) {
            v += sample;
        }
        for (const float sample : velocity_history_w_) {
            w += sample;
        }
        v /= static_cast<float>(velocity_history_v_.size());
        w /= static_cast<float>(velocity_history_w_.size());
    } else {
        velocity_history_v_.clear();
        velocity_history_w_.clear();
    }

    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(0, 0, yaw);
    geometry_msgs::msg::Quaternion quat;
    quat.x = tf2_quat.x();
    quat.y = tf2_quat.y();
    quat.z = tf2_quat.z();
    quat.w = tf2_quat.w();

    if (pub_odom_tf)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = current_time;
        tf_msg.header.frame_id = odom_frame;
        tf_msg.child_frame_id = base_frame;
        tf_msg.transform.translation.x = x;
        tf_msg.transform.translation.y = y;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = quat;
        odom_baselink_transform_->sendTransform(tf_msg);
    }

    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_frame;
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = quat;

    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = w;

    odom_pub->publish(odom_msg);
}

int Roboteq::run() {
    starttime = millis();
    hstimer = starttime;
    mstimer = starttime;
    lstimer = starttime;
        
    try {
        cmdvel_loop();
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in cmdvel_loop: " << e.what());
    }
    
    // オドメトリをメインスレッドからパブリッシュ
    try {
        odom_publish();
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in odom_publish: " << e.what());
    }

    // Roboteqテレメトリ(電圧/電流/フォルト等)をパブリッシュ
    try {
        status_publish();
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in status_publish: " << e.what());
    }
    
    // デバイスの接続状態をチェック
    if (!controller.isOpen()) {
        RCLCPP_WARN(this->get_logger(), "Device disconnected, attempting to reconnect...");
        try {
            connect();
        } catch (const std::exception &e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to reconnect: " << e.what());
        }
    }

    try {
        cmdvel_run();
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in cmdvel_run: " << e.what());
    }
    
    return 0;
}


Roboteq::~Roboteq()
{
    running_ = false;
    if (odom_thread_.joinable()){
        odom_thread_.join();
    }

    if (controller.isOpen()){
        controller.close();
    }
}

} // end of namespace

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto node = std::make_shared<Roboteq::Roboteq>();
    exec.add_node(node);
    exec.spin();
    printf("stop");
    rclcpp::shutdown();
    return 0;
}
