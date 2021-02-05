//
// Created by balazs on 2021. 02. 05..
//

#include "DumpModel.h"
#include "TypedLog.h"

constexpr char cgJointTypeNames[][16u] = {
  "UNKNOWN", "REVOLUTE", "CONTINUOUS", "PRISMATIC", "FLOATING", "PLANAR", "FIXED"
};

void dumpModelInfo(urdf::Pose const &aPose) {
  Log::i(LogTopics::dumpModel) << "origin rotation w:" << aPose.rotation.w << " x:" << aPose.rotation.x << " y:" << aPose.rotation.y << " z:" << aPose.rotation.z << Log::end;
  Log::i(LogTopics::dumpModel) << "origin position x:" << aPose.position.x << " y:" << aPose.position.y << " z:" << aPose.position.z << Log::end;
}

void dumpModelInfo(urdf::LinkConstSharedPtr aLink) {
  Log::i(LogTopics::dumpModel) << "---link---" << Log::end;
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
    Log::i(LogTopics::dumpModel) << "mesh: x:" << mesh->scale.x << " y:" << mesh->scale.y << " z:" << mesh->scale.z << " filename: " << mesh->filename << Log::end;
    dumpModelInfo(aLink->collision->origin);
  }
  else {
    Log::i(LogTopics::dumpModel) << "no collision" << Log::end;
  }
  Log::i(LogTopics::dumpModel) << Log::end;
}

void dumpModelInfo(urdf::JointConstSharedPtr aJoint) {
  Log::i(LogTopics::dumpModel) << "---joint---" << Log::end;
  Log::i(LogTopics::dumpModel) << cgJointTypeNames[aJoint->type] << Log::end;
  if(aJoint->type != urdf::Joint::REVOLUTE
     && aJoint->type != urdf::Joint::CONTINUOUS
     && aJoint->type != urdf::Joint::PRISMATIC
     && aJoint->type != urdf::Joint::FIXED) {
    throw std::invalid_argument("Only fixed, revolute and prismatic joints are supported");
  }
  else { // nothing to do
  }
  Log::i(LogTopics::dumpModel) << "axis x:" << aJoint->axis.x << " y:" << aJoint->axis.y << " z:" << aJoint->axis.z << Log::end;
  dumpModelInfo(aJoint->parent_to_joint_origin_transform);
  if(aJoint->limits) {
    Log::i(LogTopics::dumpModel) << "limit l:" << aJoint->limits->lower << " u:" << aJoint->limits->upper << Log::end;
  }
  else {
    Log::i(LogTopics::dumpModel) << "no limit" << Log::end;
  }
}

void dumpModelInfo (urdf::Model &aModel, std::map<std::string, std::string> &aParentLinkTree) {
  Log::i(LogTopics::dumpModel) << aModel.getName() << Log::end;
  Log::i(LogTopics::dumpModel) << aModel.getRoot()->name << Log::end;
  dumpModelInfo(aModel.getRoot());
  for(auto &item : aParentLinkTree) {
    Log::i(LogTopics::dumpModel) << item.second << " -> " << item.first << Log::end;
    auto link = aModel.getLink(item.first);
    dumpModelInfo(link);
    dumpModelInfo(link->parent_joint);
    Log::i(LogTopics::dumpModel) << Log::end;
  }
}
