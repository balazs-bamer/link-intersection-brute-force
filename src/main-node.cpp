//#include <urdf/model.h>

#include "TypedLog.h"
#include "DumpModel.h"
#include "EigenMeshModel.h"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"
#include <string>
#include <map>
#include <unordered_set>
#include <fstream>
#include <regex>

class NodeLinkIntersectionBruteForce final : public rclcpp::Node {
private:
  static constexpr int                    csArgumentIndexUrdf           = 1;
  static constexpr int                    csArgumentIndexForbiddenLinks = 2;
  static constexpr int                    csArgumentIndexMeshLocation   = 3;
  inline static constexpr char            csForbiddenRegex[] = "^[a-zA-Z_][a-zA-Z_/\\-0-9]*$";

  urdf::Model                             mModel;
  std::map<std::string, std::string>      mParentLinkTree;
  std::unique_ptr<fragor::EigenMeshModel> mEigenModel;
  std::unordered_set<std::string>         mForbiddenLinks;

public:
  NodeLinkIntersectionBruteForce(int, char** aArgv) : Node("linkIntersectionBruteForce") {
    mModel.initFile(std::string{aArgv[csArgumentIndexUrdf]});
    mModel.initTree(mParentLinkTree);
    mModel.initRoot(mParentLinkTree);
    readForbiddenLinks(aArgv[csArgumentIndexForbiddenLinks]);
    mEigenModel = std::make_unique<fragor::EigenMeshModel>(mModel, mParentLinkTree, mForbiddenLinks, aArgv[csArgumentIndexMeshLocation]);
  }

  void dumpModelInfo() {
    ::dumpModelInfo(mModel, mParentLinkTree);
  }

private:
  void readForbiddenLinks(char const * const aFilename) {
    std::regex regex(csForbiddenRegex);
    std::ifstream in{aFilename};
    while(true) {
      std::string line;
      std::getline(in, line);
      if(std::regex_match(line, regex)) {
        mForbiddenLinks.insert(line);
      }
      else { // nothing to do
      }
      if(!in.good()) {
        break;
      }
      else { // nothing to do
      }
    }
  }
};

namespace LogTopics {
nowtech::log::TopicInstance system;
nowtech::log::TopicInstance dumpModel;
}

// args as in launch:
// /home/balazs/munka/cuda-trajectory-planner/ros-workspace/install/link-intersection-brute-force/share/link-intersection-brute-force/px150_coll.urdf /home/balazs/munka/cuda-trajectory-planner/ros-workspace/install/link-intersection-brute-force/share/link-intersection-brute-force/forbidden-links.txt /home/balazs/munka/cuda-trajectory-planner/ros-workspace/src/link-intersection-brute-force/ --ros-args -r __node:=linkIntersectionBruteForce -r __ns:=/cudaTrajectoryPlanner
int main(int aArgc, char **aArgv) {
  rclcpp::init(aArgc, aArgv);
  nowtech::log::LogFormatConfig logConfig;
  LogSender::init();
  Log::init(logConfig);
  Log::registerTopic(LogTopics::system, "system");
  Log::registerTopic(LogTopics::dumpModel, "dumpUrdf");
  Log::registerCurrentTask("main");
  for(int i = 0; i < aArgc; ++i) {
    Log::n() << i << aArgv[i] << Log::end;
  }
  auto node = std::make_shared<NodeLinkIntersectionBruteForce>(aArgc, aArgv);
//  node.get()->dumpModelInfo();
  rclcpp::spin_some(node);
  Log::unregisterCurrentTask();
  Log::done();
  rclcpp::shutdown();
  return 0;
}
