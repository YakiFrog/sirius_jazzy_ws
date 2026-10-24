// Copyright 2026 Sirius contributors. Licensed under Apache-2.0.
#include <chrono>
#include <thread>
#include "gtest/gtest.h"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"

class Tolerances : public nav2_core::GoalChecker
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override {return false;}
  bool getTolerances(geometry_msgs::msg::Pose & p, geometry_msgs::msg::Twist &) override
  {p.position.x = 0.05; return true;}
};

class SiriusWait : public testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("controller_frequency", 10.0),
      rclcpp::Parameter("WaitPath.sirius_wait_for_clear", true),
      rclcpp::Parameter("WaitPath.sirius_clear_time", 0.06),
      rclcpp::Parameter("WaitPath.sirius_restart_accel", 0.3),
      rclcpp::Parameter("WaitPath.desired_linear_vel", 0.6),
      rclcpp::Parameter("WaitPath.lookahead_dist", 1.0),
      rclcpp::Parameter("WaitPath.use_rotate_to_heading", false),
      rclcpp::Parameter("WaitPath.use_cost_regulated_linear_velocity_scaling", false)});
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("sirius_wait_test", options);
    node->declare_parameter("controller_frequency", 10.0);
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf->setUsingDedicatedThread(true);
    costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("sirius_wait_costmap");
    costmap->on_configure(rclcpp_lifecycle::State());
    costmap->getCostmap()->resizeMap(100, 100, 0.05, -2.5, -2.5);
    clear();
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = costmap->getGlobalFrameID();
    transform.child_frame_id = costmap->getBaseFrameID();
    transform.transform.rotation.w = 1.0;
    tf->setTransform(transform, "test", true);
    controller = std::make_shared<nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController>();
    controller->configure(node, "WaitPath", tf, costmap);
    controller->activate();
    pose.header.frame_id = costmap->getGlobalFrameID();
    pose.pose.orientation.w = 1.0;
    nav_msgs::msg::Path path;
    path.header = pose.header;
    for (int i = 0; i <= 20; ++i) {
      auto point = pose; point.pose.position.x = i * 0.1; path.poses.push_back(point);
    }
    controller->setPlan(path);
  }
  void TearDown() override
  {
    controller->deactivate(); controller->cleanup(); controller.reset();
    costmap->on_cleanup(rclcpp_lifecycle::State()); costmap.reset(); node.reset();
  }
  void clear()
  {
    auto map = costmap->getCostmap();
    std::fill(map->getCharMap(), map->getCharMap() + 10000, nav2_costmap_2d::FREE_SPACE);
  }
  void obstacle()
  {
    for (unsigned int x = 57; x <= 60; ++x) {
      for (unsigned int y = 43; y <= 57; ++y) {
        costmap->getCostmap()->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
  double command()
  {
    pose.header.stamp = node->now();
    return controller->computeVelocityCommands(pose, geometry_msgs::msg::Twist(), &goal).twist.linear.x;
  }
  Tolerances goal;
  geometry_msgs::msg::PoseStamped pose;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::shared_ptr<nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController> controller;
};

TEST_F(SiriusWait, CollisionHoldsAndFlickerDoesNotRestart)
{
  EXPECT_NEAR(command(), 0.03, 1e-6);
  obstacle(); EXPECT_DOUBLE_EQ(command(), 0.0);
  clear(); EXPECT_DOUBLE_EQ(command(), 0.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  obstacle(); EXPECT_DOUBLE_EQ(command(), 0.0);
  clear(); EXPECT_DOUBLE_EQ(command(), 0.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  EXPECT_DOUBLE_EQ(command(), 0.0);  // Not 60 ms of continuous clearance yet.
  std::this_thread::sleep_for(std::chrono::milliseconds(35));
  EXPECT_NEAR(command(), 0.03, 1e-6);
  EXPECT_NEAR(command(), 0.06, 1e-6);
  obstacle(); EXPECT_DOUBLE_EQ(command(), 0.0);  // Emergency stop is never slew limited.
}

TEST_F(SiriusWait, MonitorStopHoldsAndCancelCompletes)
{
  auto publisher = node->create_publisher<nav2_msgs::msg::CollisionMonitorState>("collision_monitor_state", 10);
  publisher->on_activate();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  nav2_msgs::msg::CollisionMonitorState msg;
  msg.action_type = msg.STOP;
  for (int i = 0; i < 20; ++i) {
    publisher->publish(msg); executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_DOUBLE_EQ(command(), 0.0);
  controller->cancel(); EXPECT_DOUBLE_EQ(command(), 0.0);
  EXPECT_TRUE(controller->cancel());
}
