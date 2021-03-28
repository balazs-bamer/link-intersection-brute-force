//#include <urdf/model.h>

#include "TypedLog.h"
#include "DumpModel.h"
#include "EigenMeshModel.h"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"
#include "StlWriter.h"
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
  inline static char                      csMeshNamePrefix[] = "transformedMesh";
  inline static char                      csMeshNameSuffix[] = ".stl";

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

  void dumpMeshDistinct() {
    for(fragor::Id i = 0; i < mEigenModel->size(); ++i) {
      std::string filename{csMeshNamePrefix};
      filename += std::to_string(i);
      filename += csMeshNameSuffix;
      std::ofstream out{filename, std::ios::binary};
      std::vector<fragor::HomVertex> mesh;
      mEigenModel->transformLimbMesh(i, &mesh);
      ::writeStlBinary(out, mEigenModel->getName(i), mesh);
    }
  }

  void dumpMeshUnified() {
    mEigenModel->setJointPosition(0, -0.02f); // right finger
    mEigenModel->setJointPosition(1, 0.03f);  // left finger
    mEigenModel->setJointPosition(2, 1.3f);   // wrist roll (gripper)
    mEigenModel->setJointPosition(3, 0.3f);   // wrist pitch (wrist)
    mEigenModel->setJointPosition(4, 0.7f);   // elbow
    mEigenModel->setJointPosition(5, -1.0f);  // shoulder
    mEigenModel->setJointPosition(6, 0.4f);   // waist
    std::string filename{csMeshNamePrefix};
    filename += csMeshNameSuffix;
    std::ofstream out{filename, std::ios::binary};
    std::vector<fragor::HomVertex> mesh;
    for(fragor::Id i = 0; i < mEigenModel->size(); ++i) {
      mEigenModel->transformLimbMesh(i, &mesh);
    }
    ::writeStlBinary(out, csMeshNamePrefix, mesh);
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
  auto options = node->get_node_options();
  auto args = options.arguments();
  for(auto &i : args) {
    Log::n() << "arg: " << i << Log::end;
  }
  auto const overrides = options.parameter_overrides();
  for(auto &i : overrides) {
    Log::n() << "paraOver: " << i.get_type_name() << i.get_name() << i.as_string() << Log::end;
  }
//  node.get()->dumpModelInfo();
  node.get()->dumpMeshUnified();
  rclcpp::spin_some(node);
  Log::unregisterCurrentTask();
  Log::done();
  rclcpp::shutdown();
  return 0;
}
