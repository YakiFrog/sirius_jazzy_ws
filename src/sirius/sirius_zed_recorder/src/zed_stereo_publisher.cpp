#include <videocapture.hpp>

#include <opencv2/opencv.hpp>

// Official ZED Open Capture helper for factory calibration download and rectification.
// OpenCV must be included first because this upstream header is also used standalone.
#include <calibration.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace sirius_zed_recorder
{

sl_oc::video::RESOLUTION parse_resolution(const std::string & value)
{
  if (value == "HD2K") {
    return sl_oc::video::RESOLUTION::HD2K;
  }
  if (value == "HD1080") {
    return sl_oc::video::RESOLUTION::HD1080;
  }
  if (value == "HD720") {
    return sl_oc::video::RESOLUTION::HD720;
  }
  if (value == "VGA") {
    return sl_oc::video::RESOLUTION::VGA;
  }
  throw std::invalid_argument(
          "Unsupported resolution '" + value +
          "'. Use HD2K, HD1080, HD720, or VGA.");
}

sl_oc::video::FPS parse_fps(int value)
{
  switch (value) {
    case 15:
      return sl_oc::video::FPS::FPS_15;
    case 30:
      return sl_oc::video::FPS::FPS_30;
    case 60:
      return sl_oc::video::FPS::FPS_60;
    case 100:
      return sl_oc::video::FPS::FPS_100;
    default:
      throw std::invalid_argument("Unsupported FPS. Use 15, 30, 60, or 100.");
  }
}

class ZedStereoPublisher : public rclcpp::Node
{
public:
  ZedStereoPublisher()
  : Node("zed_stereo_publisher"), last_frame_timestamp_(0), frame_count_(0)
  {
    const auto resolution_name = declare_parameter<std::string>("resolution", "HD720");
    const auto fps = static_cast<int>(declare_parameter<int>("fps", 15));
    jpeg_quality_ = std::clamp(
      static_cast<int>(declare_parameter<int>("jpeg_quality", 90)), 1, 100);
    const auto device_id = static_cast<int>(declare_parameter<int>("device_id", -1));
    frame_id_ = declare_parameter<std::string>(
      "frame_id", "sirius3/zed_camera_link");
    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/camera/stereo_sbs/compressed");
    const auto params_topic = declare_parameter<std::string>(
      "params_topic", "/camera/stereo_params");

    image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      image_topic, rclcpp::SensorDataQoS());
    params_pub_ = create_publisher<std_msgs::msg::String>(
      params_topic, rclcpp::QoS(1).reliable().transient_local());

    sl_oc::video::VideoParams capture_params;
    capture_params.res = parse_resolution(resolution_name);
    capture_params.fps = parse_fps(fps);
    capture_params.verbose = sl_oc::VERBOSITY::INFO;
    camera_ = std::make_unique<sl_oc::video::VideoCapture>(capture_params);

    if (!camera_->initializeVideo(device_id)) {
      throw std::runtime_error(
              "Cannot open the ZED USB camera. Confirm USB3 connection and /dev/video permissions.");
    }
    serial_number_ = camera_->getSerialNumber();

    std::string calibration_file;
    if (!sl_oc::tools::downloadCalibrationFile(serial_number_, calibration_file)) {
      throw std::runtime_error(
              "Cannot obtain factory calibration for ZED SN" +
              std::to_string(serial_number_) + ". Connect to the internet once or copy SN" +
              std::to_string(serial_number_) + ".conf into ~/zed/settings/.");
    }

    int stereo_width = 0;
    int height = 0;
    camera_->getFrameSize(stereo_width, height);
    width_ = stereo_width / 2;
    height_ = height;

    cv::Mat left_camera_matrix;
    cv::Mat right_camera_matrix;
    double baseline_mm = 0.0;
    if (!sl_oc::tools::initCalibration(
        calibration_file, cv::Size(width_, height_),
        left_map_x_, left_map_y_, right_map_x_, right_map_y_,
        left_camera_matrix, right_camera_matrix, &baseline_mm))
    {
      throw std::runtime_error("Invalid ZED factory calibration file: " + calibration_file);
    }

    fx_ = left_camera_matrix.at<double>(0, 0);
    fy_ = left_camera_matrix.at<double>(1, 1);
    cx_ = left_camera_matrix.at<double>(0, 2);
    cy_ = left_camera_matrix.at<double>(1, 2);
    baseline_m_ = std::abs(baseline_mm) / 1000.0;
    if (fx_ <= 0.0 || fy_ <= 0.0 || baseline_m_ <= 0.0) {
      throw std::runtime_error("ZED calibration contains invalid intrinsics or baseline.");
    }

    publish_camera_params();
    params_timer_ = create_wall_timer(1s, [this]() {publish_camera_params();});
    capture_timer_ = create_wall_timer(1ms, [this]() {capture_once();});
    report_started_at_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      get_logger(),
      "CUDA-free ZED recorder ready: SN%u, %dx%d per eye, %d FPS, JPEG=%d",
      serial_number_, width_, height_, fps, jpeg_quality_);
    RCLCPP_INFO(
      get_logger(), "Publishing rectified SBS JPEG on %s and calibration on %s",
      image_topic.c_str(), params_topic.c_str());
  }

private:
  void publish_camera_params()
  {
    std::ostringstream json;
    json << std::fixed << std::setprecision(8)
         << "{\"fx\":" << fx_
         << ",\"fy\":" << fy_
         << ",\"cx\":" << cx_
         << ",\"cy\":" << cy_
         << ",\"baseline\":" << baseline_m_
         << ",\"width\":" << width_
         << ",\"height\":" << height_
         << ",\"serial_number\":" << serial_number_
         << ",\"rectified\":true"
         << ",\"source\":\"zed_open_capture\"}";

    std_msgs::msg::String message;
    message.data = json.str();
    params_pub_->publish(message);
  }

  builtin_interfaces::msg::Time frame_stamp(uint64_t timestamp_ns) const
  {
    const auto ros_now = now();
    const auto ros_now_ns = static_cast<uint64_t>(ros_now.nanoseconds());
    const auto difference = timestamp_ns > ros_now_ns ?
      timestamp_ns - ros_now_ns : ros_now_ns - timestamp_ns;

    // ZED Open Capture normally returns a system timestamp. Fall back to the
    // ROS system clock if a camera/firmware reports a different time domain.
    if (timestamp_ns == 0 || difference > 10ULL * 1000000000ULL) {
      return static_cast<builtin_interfaces::msg::Time>(ros_now);
    }

    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(timestamp_ns / 1000000000ULL);
    stamp.nanosec = static_cast<uint32_t>(timestamp_ns % 1000000000ULL);
    return stamp;
  }

  void capture_once()
  {
    const auto & frame = camera_->getLastFrame(100);
    if (frame.data == nullptr || frame.timestamp == last_frame_timestamp_) {
      return;
    }
    last_frame_timestamp_ = frame.timestamp;

    cv::Mat frame_yuv(frame.height, frame.width, CV_8UC2, frame.data);
    cv::Mat frame_bgr;
    cv::cvtColor(frame_yuv, frame_bgr, cv::COLOR_YUV2BGR_YUYV);
    const auto eye_width = frame_bgr.cols / 2;
    const cv::Mat left_raw = frame_bgr(cv::Rect(0, 0, eye_width, frame_bgr.rows));
    const cv::Mat right_raw = frame_bgr(
      cv::Rect(eye_width, 0, eye_width, frame_bgr.rows));

    cv::Mat left_rectified;
    cv::Mat right_rectified;
    cv::remap(left_raw, left_rectified, left_map_x_, left_map_y_, cv::INTER_LINEAR);
    cv::remap(right_raw, right_rectified, right_map_x_, right_map_y_, cv::INTER_LINEAR);

    cv::Mat stereo_sbs;
    cv::hconcat(left_rectified, right_rectified, stereo_sbs);
    std::vector<unsigned char> jpeg;
    const std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    if (!cv::imencode(".jpg", stereo_sbs, jpeg, encode_params)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Failed to JPEG-encode ZED frame");
      return;
    }

    sensor_msgs::msg::CompressedImage message;
    message.header.stamp = frame_stamp(frame.timestamp);
    message.header.frame_id = frame_id_;
    message.format = "jpeg; stereo=sbs; layout=left-right; rectified=true";
    message.data.assign(jpeg.begin(), jpeg.end());
    image_pub_->publish(std::move(message));
    ++frame_count_;

    const auto current = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration<double>(current - report_started_at_).count();
    if (elapsed >= 5.0) {
      RCLCPP_INFO(
        get_logger(), "Recording source active: %.1f FPS",
        static_cast<double>(frame_count_) / elapsed);
      frame_count_ = 0;
      report_started_at_ = current;
    }
  }

  std::unique_ptr<sl_oc::video::VideoCapture> camera_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr params_pub_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::TimerBase::SharedPtr params_timer_;
  cv::Mat left_map_x_;
  cv::Mat left_map_y_;
  cv::Mat right_map_x_;
  cv::Mat right_map_y_;
  std::string frame_id_;
  int jpeg_quality_;
  int width_;
  int height_;
  unsigned int serial_number_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double baseline_m_;
  uint64_t last_frame_timestamp_;
  std::size_t frame_count_;
  std::chrono::steady_clock::time_point report_started_at_;
};

}  // namespace sirius_zed_recorder

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<sirius_zed_recorder::ZedStereoPublisher>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("zed_stereo_publisher"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
