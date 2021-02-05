//#include <urdf/model.h>

#include "TypedLog.h"
#include "DumpModel.h"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"
#include <string>
#include <map>

class NodeLinkIntersectionBruteForce final : public rclcpp::Node {
private:
  static constexpr int csUrdfArgumentIndex = 1;

  urdf::Model mModel;
  std::map<std::string, std::string> mParentLinkTree;

public:
  NodeLinkIntersectionBruteForce(int, char** aArgv) : Node("linkIntersectionBruteForce") {
    mModel.initFile(std::string{aArgv[csUrdfArgumentIndex]});
    mModel.initTree(mParentLinkTree);
    mModel.initRoot(mParentLinkTree);
  }

  void dumpModelInfo() {
    ::dumpModelInfo(mModel, mParentLinkTree);
  }

private:
};

namespace LogTopics {
nowtech::log::TopicInstance system;
nowtech::log::TopicInstance dumpModel;
}

// args as in launch:
// /home/balazs/munka/cuda-trajectory-planner/ros-workspace/install/link-intersection-brute-force/share/link-intersection-brute-force/px150_coll.urdf --ros-args -r __node:=linkIntersectionBruteForce -r __ns:=/cudaTrajectoryPlanner
int main(int aArgc, char **aArgv) {
  rclcpp::init(aArgc, aArgv);
  nowtech::log::LogFormatConfig logConfig;
  LogSender::init();
  Log::init(logConfig);
  Log::registerTopic(LogTopics::system, "system");
  Log::registerTopic(LogTopics::dumpModel, "dumpUrdf");
  Log::registerCurrentTask("main");
  auto node = std::make_shared<NodeLinkIntersectionBruteForce>(aArgc, aArgv);
  node.get()->dumpModelInfo();
  rclcpp::spin_some(node);
  Log::unregisterCurrentTask();
  Log::done();
  rclcpp::shutdown();
  return 0;
}
