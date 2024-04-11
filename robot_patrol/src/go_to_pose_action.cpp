#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/logging.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/impl/utils.h"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_patrol/action/go_to_pose.hpp"

template <class T>
inline int
sgn(T v) {
    return (v > T(0)) - (v < T(0));
}

class GoToPose : public rclcpp::Node {
public:
  using GTP = robot_patrol::action::GoToPose;
  using GoalHandleGTP = rclcpp_action::ServerGoalHandle<GTP>;

  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("go_to_pose_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GTP>(
        this, "go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    this->publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    this->subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&GoToPose::odom_callback, this, _1));
  }

private:
  rclcpp_action::Server<GTP>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;

  geometry_msgs::msg::Pose2D current_pos_;
  geometry_msgs::msg::Pose2D desired_pos_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GTP::Goal> goal) {
    RCLCPP_INFO(
        this->get_logger(),
        "Received goal request with pos:\nX = %f, Y = %f and theta = %f",
        goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
    desired_pos_ = goal->goal_pos;
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGTP> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGTP> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGTP> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GTP::Feedback>();
    auto result = std::make_shared<GTP::Result>();
    auto cmd_vel = geometry_msgs::msg::Twist();
    cmd_vel.linear.x = 0.15;

    rclcpp::Rate loop_rate(20);

    while (dist_to_goal() > 0.02) {
      if (goal_handle->is_canceling()) {
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      cmd_vel.angular.z = direction() / M_PI;
      RCLCPP_INFO(this->get_logger(), "dir %f", direction() / M_PI);

      if (dist_to_goal() < 0.15) {
        cmd_vel.linear.x = dist_to_goal() / 2;
      }

      publisher_->publish(cmd_vel);
      feedback->current_pos = this->current_pos_;
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    publisher_->publish(cmd_vel);

    float error_angle = 180;

    while (std::abs(error_angle) > 0.5) {
      error_angle = desired_pos_.theta - current_pos_.theta;
      RCLCPP_INFO(this->get_logger(),
                  "desired pos : %f, current pos %f, error %f",
                  desired_pos_.theta, current_pos_.theta, error_angle);
      feedback->current_pos = this->current_pos_;
      goal_handle->publish_feedback(feedback);
      cmd_vel.angular.z = (error_angle / 180) + sgn<float>(error_angle) * 0.2;
      RCLCPP_INFO(this->get_logger(), "cmd vel %f, %f", error_angle / 180,  sgn<float>(error_angle) * 0.2);
      publisher_->publish(cmd_vel);
      loop_rate.sleep();
    }
    // Check if goal is done
    if (rclcpp::ok()) {
      result->status = true;
      cmd_vel.angular.z = 0.0;
      publisher_->publish(cmd_vel);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    current_pos_.theta = tf2::impl::getYaw(q);
    current_pos_.theta *= 180 / M_PI;
  }

  float dist_to_goal() const {
    return std::sqrt((this->desired_pos_.x - this->current_pos_.x) *
                         (this->desired_pos_.x - this->current_pos_.x) +
                     (this->desired_pos_.y - this->current_pos_.y) *
                         (this->desired_pos_.y - this->current_pos_.y));
  }

  float direction() const {
    return std::atan2(this->desired_pos_.y - this->current_pos_.y,
                      this->desired_pos_.x - this->current_pos_.x);
  }
}; // class GoToPose

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPose>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}