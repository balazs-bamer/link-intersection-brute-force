//
// Created by balazs on 2021. 02. 05..
//

#ifndef LINK_INTERSECTION_BRUTE_FORCE_EIGENMESHMODEL_H
#define LINK_INTERSECTION_BRUTE_FORCE_EIGENMESHMODEL_H

#include "urdf/model.h"
#include <Eigen/Dense>
#include <unordered_map>
#include <unordered_set>
#include <optional>

namespace fragor {

using Id = int32_t;

class EigenMeshJointLink {
private:
  struct ChildTransform {
    std::shared_ptr<EigenMeshJointLink>            mChild;
    Eigen::Matrix4f                                mTransform;
  };

  std::optional<urdf::JointSharedPtr>              mJoint;
  std::optional<std::weak_ptr<EigenMeshJointLink>> mParent;               // used to travel backwards for composition
  urdf::LinkSharedPtr                              mFirstLink;
  std::vector<Eigen::Vector3f>                     mMesh;
  std::unordered_map<Id, ChildTransform>           mChildrenWithTransform;
  // These are identity for root.
  Eigen::Matrix4f                                  mTranslation1;         // For itself.
  Eigen::Matrix4f                                  mRotation2;            // For itself or optionally including the rotate joint.
  Eigen::Matrix4f                                  mTranslation3;         // For optional prismatic joint.
  Eigen::Matrix4f                                  mCumulativeTransform;  // everything in the tree up to and including this
};

// FIX:   1. trasnsl by pos  2. rot by rot.
// ROT:   1. trasnsl by pos  2. rot by (rot + act)
// PRIS:  1. trasnsl by pos  2. rot by rot. 3. transl by

class EigenMeshModel final {
private:
  std::shared_ptr<EigenMeshJointLink> mRoot;

public:
  EigenMeshModel(urdf::Model const &aModel, std::map<std::string, std::string> const &aParentLinkTree, std::unordered_set<std::string> const aForbiddenLinks);
};

}

#endif //LINK_INTERSECTION_BRUTE_FORCE_EIGENMESHMODEL_H
