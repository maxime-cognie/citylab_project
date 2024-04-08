#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("robot_patrol"), direction_(0.0) {
    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions scan_sub_option_;
    scan_sub_option_.callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    timer_ =
        this->create_wall_timer(100ms, std::bind(&Patrol::timer_callback, this),
                                this->timer_callback_group_);
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&Patrol::scan_callback, this, std::placeholders::_1),
        scan_sub_option_);

    cmd_vel_.linear.x = 0.1;
  }

private:
  void timer_callback() {
    cmd_vel_.angular.z = direction_ / 2;
    vel_pub_->publish(cmd_vel_);
  };

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float front_range = msg->range_max;
    float max_range = 0;
    int max_range_index = 0;
    RCLCPP_INFO(this->get_logger(), "front %f", msg->ranges[659]);

    for (int i = 0; i < 165; i++) {
        if (msg->ranges[i] > msg->range_min &&
            msg->ranges[i] < front_range &&
            i < 55) {
            front_range = msg->ranges[i];
        }
        if (msg->ranges[659 - i] > msg->range_min &&
            msg->ranges[659 - i] < front_range &&
            i < 55) {
            front_range = msg->ranges[659 - i];
        }
        if (msg->ranges[i] < msg->range_max &&
            msg->ranges[i] > max_range) {
            max_range = msg->ranges[i];
            max_range_index = i;  
        }
        if (msg->ranges[659 - i] < msg->range_max &&
            msg->ranges[659 - i] > max_range) {
            max_range = msg->ranges[659 - i];
            max_range_index = - i;
        }
    }
    if (front_range > 0.5) {
      direction_ = 0.0;
    } else {
      // convert the index of the largest ray in the scan array to its
      // corresponding angle (between -pi/2 and pi/2)
      direction_ = (max_range_index * M_PI / 2) / 165;
    }
  };

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::CallbackGroup::SharedPtr scan_callback_group_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  double direction_;
  geometry_msgs::msg::Twist cmd_vel_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<Patrol> robot_patrol = std::make_shared<Patrol>();

  rclcpp::executors::MultiThreadedExecutor executors;
  executors.add_node(robot_patrol);
  executors.spin();

  rclcpp::shutdown();
  return 0;
}