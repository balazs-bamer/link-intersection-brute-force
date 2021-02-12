//
// Created by balazs on 2021. 02. 05..
//

#ifndef LINK_INTERSECTION_BRUTE_FORCE_EIGENMESHMODEL_H
#define LINK_INTERSECTION_BRUTE_FORCE_EIGENMESHMODEL_H

#include "urdf/model.h"
#include <Eigen/Dense>
#include <unordered_map>
#include <unordered_set>

namespace fragor {

using Id = int32_t;

class EigenMeshJointLink {
private:
  struct ChildTransform {
    std::shared_ptr<EigenMeshJointLink> const mChild;
    Eigen::Matrix4f const                     mTransform;        // Won't change since fixed joints lead to the child.

    ChildTransform(std::shared_ptr<EigenMeshJointLink> const aChild, Eigen::Matrix4f const aTransform)
    : mChild(aChild), mTransform(aTransform) {}
  };
  inline static constexpr char           csStlNamePrefix[] = "package://";

  Id const                               mActualLocalRootId;     // +
  urdf::LinkConstSharedPtr const         mFirstLink;             // +
  urdf::JointSharedPtr     const         mJoint;                 // +
  std::weak_ptr<EigenMeshJointLink>      mParent;                // + Used to travel backwards for composition.
  std::vector<Eigen::Vector3f>           mMesh;                  // Cumulative, in the mesh coordinate space of the local root link.
  std::unordered_map<Id, ChildTransform> mChildrenWithTransform; // All the cumulative transforms from the local root link of this mesh to the actual child's holding link.
  // These are identity for root.
  Eigen::Matrix4f                        mTranslation1;          // For itself.
  Eigen::Matrix4f                        mRotation2;             // For itself or optionally including the rotate joint.
  Eigen::Matrix4f                        mTranslation3;          // For optional prismatic joint.
  Eigen::Matrix4f                        mCumulativeTransform;   // Everything in the tree up to and including this. This enables

public:
  EigenMeshJointLink(urdf::Model const &aModel,
                     Id const aActualLocalRootId,
                     std::unordered_map<Id, Id> const &aLinkId2parentLinkId,
                     std::unordered_set<Id> const &aCurrentLimbIds,
                     std::unordered_map<Id, std::string> const &aId2name);
  void addChild(std::shared_ptr<EigenMeshJointLink> aChild);
  void addParent(std::shared_ptr<EigenMeshJointLink> aParent);
};

// FIX:   1. trasnsl by pos  2. rot by rot.
// ROT:   1. trasnsl by pos  2. rot by (rot + act)
// PRIS:  1. trasnsl by pos  2. rot by rot. 3. transl by

class EigenMeshModel final {
private:
  std::shared_ptr<EigenMeshJointLink>                         mRoot;
  std::unordered_map<Id, std::shared_ptr<EigenMeshJointLink>> mId2limb;
  std::unordered_map<Id, Id>                                  mId2subRoot;

public:
  EigenMeshModel(urdf::Model const &aModel, std::map<std::string, std::string> const &aParentLinkTree, std::unordered_set<std::string> const aForbiddenLinks);
};

}
#endif //LINK_INTERSECTION_BRUTE_FORCE_EIGENMESHMODEL_H
