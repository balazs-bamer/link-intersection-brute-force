//#include <urdf/model.h>

#include "TypedLog.h"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"
#include <string>
#include <map>

class NodeLinkIntersectionBruteForce final : public rclcpp::Node {
private:
  static constexpr int csUrdfArgumentIndex = 1;

  urdf::Model mModel;
  std::map<std::string, std::string> mParentLinkTree;
  static constexpr char csJointTypeNames[][16u] = {
    "UNKNOWN", "REVOLUTE", "CONTINUOUS", "PRISMATIC", "FLOATING", "PLANAR", "FIXED"
  };

public:
  NodeLinkIntersectionBruteForce(int, char** aArgv) : Node("linkIntersectionBruteForce") {
    mModel.initFile(std::string{aArgv[csUrdfArgumentIndex]});
    Log::i(LogTopics::dumpUrdf) << mModel.getName() << Log::end;
    mModel.initTree(mParentLinkTree);
    mModel.initRoot(mParentLinkTree);
    Log::i(LogTopics::dumpUrdf) << mModel.getRoot()->name << Log::end;
    dumpInfo(mModel.getRoot());
    for(auto &item : mParentLinkTree) {
      Log::i(LogTopics::dumpUrdf) << item.second << " -> " << item.first << Log::end;
      auto link = mModel.getLink(item.first);
      dumpInfo(link);
      dumpInfo(link->parent_joint);
      Log::i(LogTopics::dumpUrdf) << Log::end;
    }
  }

private:
  rclcpp::Logger l() const {
    return get_logger();
  }

  void dumpInfo(urdf::LinkConstSharedPtr aLink) {
    Log::i(LogTopics::dumpUrdf) << "---link---" << Log::end;
    if(aLink->collision) {
      if(aLink->collision->geometry->type != urdf::Geometry::MESH) {
        throw std::invalid_argument("Only meshes are supported as geometry.");
      }
      else { // nothing to do
      }
      auto coll = aLink->collision.get();
      auto geomsh{coll->geometry};
      auto geom = geomsh.get();
      auto mesh = dynamic_cast<urdf::Mesh*>(geom);
      Log::i(LogTopics::dumpUrdf) << "mesh: x:" << mesh->scale.x << " y:" << mesh->scale.y << " z:" << mesh->scale.z << " filename: " << mesh->filename << Log::end;
      dumpInfo(aLink->collision->origin);
    }
    else {
      Log::i(LogTopics::dumpUrdf) << "no collision" << Log::end;
    }
    Log::i(LogTopics::dumpUrdf) << Log::end;
  }

  void dumpInfo(urdf::JointConstSharedPtr aJoint) {
    Log::i(LogTopics::dumpUrdf) << "---joint---" << Log::end;
    Log::i(LogTopics::dumpUrdf) << csJointTypeNames[aJoint->type] << Log::end;
    if(aJoint->type != urdf::Joint::REVOLUTE
    && aJoint->type != urdf::Joint::CONTINUOUS
    && aJoint->type != urdf::Joint::PRISMATIC
    && aJoint->type != urdf::Joint::FIXED) {
      throw std::invalid_argument("Only fixed, revolute and prismatic joints are supported");
    }
    else { // nothing to do
    }
    Log::i(LogTopics::dumpUrdf) << "axis x:" << aJoint->axis.x << " y:" << aJoint->axis.y << " z:" << aJoint->axis.z << Log::end;
    dumpInfo(aJoint->parent_to_joint_origin_transform);
    if(aJoint->limits) {
      Log::i(LogTopics::dumpUrdf) << "limit l:" << aJoint->limits->lower << " u:" << aJoint->limits->upper << Log::end;
    }
    else {
      Log::i(LogTopics::dumpUrdf) << "no limit" << Log::end;
    }
  }

  void dumpInfo(urdf::Pose const &aPose) {
    Log::i(LogTopics::dumpUrdf) << "origin rotation w:" << aPose.rotation.w << " x:" << aPose.rotation.x << " y:" << aPose.rotation.y << " z:" << aPose.rotation.z << Log::end;
    Log::i(LogTopics::dumpUrdf) << "origin position x:" << aPose.position.x << " y:" << aPose.position.y << " z:" << aPose.position.z << Log::end;
  }
};

// args as in launch:
// /home/balazs/munka/cuda-trajectory-planner/ros-workspace/install/link-intersection-brute-force/share/link-intersection-brute-force/px150_coll.urdf --ros-args -r __node:=linkIntersectionBruteForce -r __ns:=/cudaTrajectoryPlanner
int main(int aArgc, char **aArgv) {
  rclcpp::init(aArgc, aArgv);
  nowtech::log::LogFormatConfig logConfig;
  LogSender::init();
  Log::init(logConfig);
  Log::registerTopic(LogTopics::system, "system");
  Log::registerTopic(LogTopics::dumpUrdf, "dumpUrdf");
  Log::registerCurrentTask("main");
  auto node = std::make_shared<NodeLinkIntersectionBruteForce>(aArgc, aArgv);
  rclcpp::spin_some(node);
  Log::unregisterCurrentTask();
  Log::done();
  rclcpp::shutdown();
  return 0;
}

constexpr char NodeLinkIntersectionBruteForce::csJointTypeNames[][16u];
