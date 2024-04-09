#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <map>
#include <memory>

using GetDirection = robot_patrol::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("node_server") {
    dir_srv_ = this->create_service<GetDirection>(
        "direction_service",
        std::bind(&DirectionService::direction_callback, this, _1, _2));
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr dir_srv_;

  void direction_callback(const std::shared_ptr<GetDirection::Request> request,
                          std::shared_ptr<GetDirection::Response> response) {
    int section_size = request->laser_data.ranges.size() / 6;
    int front_section_start_1 = 0;
    int front_section_start_2 =
        request->laser_data.ranges.size() - section_size / 2 - 1;
    int left_section_start = section_size / 2 - 1;
    int right_section_start = front_section_start_2 - section_size;

    // map a section to the total dist
    std::map<std::string, float> dir_map;
    dir_map["front"] =
        sum_section(*request, front_section_start_1, section_size / 2) +
        sum_section(*request, front_section_start_2, section_size / 2);
    dir_map["left"] = sum_section(*request, left_section_start, section_size);
    dir_map["right"] = sum_section(*request, right_section_start, section_size);
    if (dir_map["front"] < 100) {
      auto max_dist = std::max_element(
          dir_map.begin(), dir_map.end(),
          [](const auto &l, const auto &r) { return l.second < r.second; });
      response->direction = max_dist->first;
    } else {
      response->direction = "front";
    }
  }

  // sum all the ranges from a particular section of the laser scan data
  // section start at index 'start' and end at index 'start' + 'shift'
  float sum_section(const GetDirection::Request &req, const int start,
                    const int shift) {
    float sum = 0.0;
    for (int i = start; i < start + shift; i++) {
      // adds up only ranges between range min and range max
      if (req.laser_data.ranges[i] >= req.laser_data.range_min) {
        if (req.laser_data.ranges[i] <= req.laser_data.range_max) {
          sum += req.laser_data.ranges[i];
        } else {
          sum += req.laser_data.range_max;
        }
      }
    }
    return sum;
  }
};

int main(int argc, const char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}
