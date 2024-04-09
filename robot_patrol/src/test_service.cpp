#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "robot_patrol/srv/get_direction.hpp"

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>

using namespace std::chrono_literals;
using GetDirection = robot_patrol::srv::GetDirection;

class TestClient : public rclcpp::Node {
private:
  rclcpp::Client<GetDirection>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool service_done_ = false;

  void timer_callback() {
    while (!client_->wait_for_service(1s)) {
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

    service_done_ = false;
    auto result_future = client_->async_send_request(
        request, std::bind(&TestClient::response_callback, this,
                           std::placeholders::_1));
  }

  void
  response_callback(rclcpp::Client<GetDirection>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
        auto result = future.get();
      RCLCPP_INFO(this->get_logger(), "Result: %s", result->direction.c_str());
      service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

public:
  TestClient() : Node("test_client") {
    client_ = this->create_client<GetDirection>("direction_service");
    timer_ = this->create_wall_timer(
        1s, std::bind(&TestClient::timer_callback, this));
  }

  bool is_service_done() const { return this->service_done_; }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto test_client = std::make_shared<TestClient>();
  while (!test_client->is_service_done()) {
    rclcpp::spin_some(test_client);
  }

  rclcpp::shutdown();
  return 0;
}