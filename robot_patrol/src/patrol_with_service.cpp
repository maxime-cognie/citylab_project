#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using GetDirection = robot_patrol::srv::GetDirection;

class PatrolService : public rclcpp::Node {
public:
  PatrolService() : Node("robot_patrol_client") {
    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions scan_sub_option_;
    scan_sub_option_.callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    this->dir_client_ = this->create_client<GetDirection>("direction_service");
    this->timer_ = this->create_wall_timer(
        100ms, std::bind(&PatrolService::timer_callback, this),
        this->timer_callback_group_);
    this->vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    this->scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&PatrolService::scan_callback, this, std::placeholders::_1),
        scan_sub_option_);

    this->cmd_vel_.linear.x = 0.1;
    this->dir_ang_vel_["left"] = 0.5;
    this->dir_ang_vel_["fornt"] = 0.0;
    this->dir_ang_vel_["right"] = -0.5;
  }

private:
  void timer_callback() {
    while (!dir_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }

    auto request = std::make_shared<GetDirection::Request>();

    request->laser_data = last_laser_;

    auto result_future = dir_client_->async_send_request(request);

    if (result_future.wait_for(1s) == std::future_status::ready) {
      auto result = result_future.get();
      this->cmd_vel_.angular.z = this->dir_ang_vel_[result->direction];
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to call service /direction_service");
    }

    this->vel_pub_->publish(cmd_vel_);
  };

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    this->last_laser_ = *msg;
  };

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::CallbackGroup::SharedPtr scan_callback_group_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Client<GetDirection>::SharedPtr dir_client_;

  sensor_msgs::msg::LaserScan last_laser_;
  std::map<std::string, float> dir_ang_vel_;
  geometry_msgs::msg::Twist cmd_vel_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<PatrolService> robot_patrol =
      std::make_shared<PatrolService>();

  rclcpp::executors::MultiThreadedExecutor executors;
  executors.add_node(robot_patrol);
  executors.spin();

  rclcpp::shutdown();
  return 0;
}