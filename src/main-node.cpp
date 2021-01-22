//#include <urdf/model.h>
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"
#include <string>

class NodeLinkIntersectionBruteForce final : public rclcpp::Node {
private:
  static constexpr int csUrdfArgumentIndex = 1;

  urdf::Model mModel;

public:
  NodeLinkIntersectionBruteForce(int aArgc, char** aArgv) : Node("linkIntersectionBruteForce") {
    mModel.initFile(std::string{aArgv[csUrdfArgumentIndex]});
    RCLCPP_INFO(get_logger(), mModel.getName());
  }
};

int main(int aArgc, char **aArgv) {
  rclcpp::init(aArgc, aArgv);
  auto node = std::make_shared<NodeLinkIntersectionBruteForce>(aArgc, aArgv);
  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
