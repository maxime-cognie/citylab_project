#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/logging.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MyActionClient : public rclcpp::Node
{
public:
  using GoToPose = robot_patrol::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ClientGoalHandle<GoToPose>;

  explicit MyActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("my_action_client", node_options), goal_done_(false)
  {
    this->client_ptr_ = rclcpp_action::create_client<GoToPose>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "go_to_pose");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MyActionClient::send_goal, this));
  }

  bool is_goal_done() const
  {
    return this->goal_done_;
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = GoToPose::Goal();
    goal_msg.goal_pos.x = 0.7;
    goal_msg.goal_pos.y = 0.3;
    goal_msg.goal_pos.theta = 0.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<GoToPose>::SendGoalOptions();
                
    send_goal_options.goal_response_callback =
      std::bind(&MyActionClient::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
      std::bind(&MyActionClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
      std::bind(&MyActionClient::result_callback, this, _1);
      
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<GoToPose>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void goal_response_callback(const GoalHandleGoToPose::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleGoToPose::SharedPtr,
    const std::shared_ptr<const GoToPose::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(), "Feedback received: current pos: \nx: %f\ny: %f\n theta: %f", 
        feedback->current_pos.x, feedback->current_pos.y, feedback->current_pos.theta);
  }

  void result_callback(const GoalHandleGoToPose::WrappedResult & result)
  {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Success");
        return;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
  }
};  // class MyActionClient

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<MyActionClient>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_client);

  while (!action_client->is_goal_done() && rclcpp::ok()) {
    if (action_client->is_goal_done()){
        RCLCPP_INFO(rclcpp::get_logger("my_action_client"), "test");
    }
    executor.spin();
  }

  rclcpp::shutdown();
  return 0;
}