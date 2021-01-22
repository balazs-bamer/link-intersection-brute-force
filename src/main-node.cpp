//#include <urdf/model.h>
#include "rclcpp/rclcpp.hpp"

class NodeLinkIntersectionBruteForce : public rclcpp::Node {
public:
  NodeLinkIntersectionBruteForce() : Node("linkIntersectionBruteForce") {
    RCLCPP_INFO(get_logger(), "log still works");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NodeLinkIntersectionBruteForce>();
  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
