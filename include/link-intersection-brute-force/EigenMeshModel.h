//
// Created by balazs on 2021. 02. 05..
//

#ifndef LINK_INTERSECTION_BRUTE_FORCE_EIGENMESHMODEL_H
#define LINK_INTERSECTION_BRUTE_FORCE_EIGENMESHMODEL_H

#include "urdf/model.h"
#include <Eigen/Dense>
#include <unordered_map>
#include <unordered_set>
#include <deque>

namespace fragor {

using Id = int32_t;
using Transform   = Eigen::Transform<float, 3, Eigen::Affine>; // TODO check if bottom row remains 0 0 0 1
using Translation = Eigen::Translation<float, 3>;
using Quaternion  = Eigen::Quaternion<float>;
using HomVertex   = Eigen::Vector4f;                           // Last coordinate is 1.

class EigenMeshJointLink {
private:
  struct ChildTransform {
    std::shared_ptr<EigenMeshJointLink> mChild;                    // +
    Transform                           mAllFixedTransforms;       // +
    Translation                         mPossibleUnitDisplacement; // + Prism direction for prismatic joint.
    Quaternion                          mPossibleUnitRotation;     // + Rotation axis with 1 radian unit for rotate joint.
    // Transforms come so from local parent to non-fixed joint child:
    // 1. All of fixed transform across the fixed joints.
    // 2. Constant displacement for this joint.
    // 3. Constant rotation for this joint.
    // 4. Actual rotation for rotate joint. | Actual joint displacement for prismatic joint. - These on urdf::Joint.axis
  };
  static constexpr float                 csEpsilon         = 1e-6f;
  static constexpr float                 csHomogeneousOne  = 1.0f;
  inline static constexpr char           csStlNamePrefix[] = "package://";

  Id const                               mActualLocalRootId;     // +
  urdf::LinkConstSharedPtr const         mLocalRootLink;         // +
  urdf::JointSharedPtr     const         mJoint;                 // + Joint holding this stuff.
  std::weak_ptr<EigenMeshJointLink>      mParent;                // + Used to travel backwards for composition.
  std::vector<HomVertex>                 mMesh;                  // + Cumulative, in the mesh coordinate space of the local root link.
  std::unordered_map<Id, ChildTransform> mChildrenWithTransform; // + Maps from the child's local root id to the actual struct.
  // These are identity for root.
  Eigen::Matrix4f                        mCumulativeTransform;   // Everything in the tree up to and including this local root? TODO consider

public:
  EigenMeshJointLink(urdf::Model const &aModel,
                     Id const aActualLocalRootId,
                     std::unordered_map<Id, Id> const &aLinkId2parentLinkId,
                     std::unordered_set<Id> const &aCurrentLimbIds,
                     std::unordered_map<Id, std::string> const &aId2name,
                     std::string const &aMeshRootDirectory);
  void addChild(std::shared_ptr<EigenMeshJointLink> aChild) { mChildrenWithTransform[aChild->getLocalRootId()].mChild = aChild; }
  void addParent(std::shared_ptr<EigenMeshJointLink> aParent) { mParent = aParent; }
  Id   getLocalRootId() const { return mActualLocalRootId; }

private:
  void                  readMesh(Transform const &aTransform,
                                 std::string const aFilename,
                                 std::deque<HomVertex> &aMeshes,
                                 std::string const &aMeshRootDirectory);
  void                  collectMesh(urdf::CollisionSharedPtr aCollision, std::deque<HomVertex> aMeshes, std::string const &aMeshRootDirectory);
  static Transform      createChildFixedTransform(urdf::JointSharedPtr aJoint);
  static Transform      createFixedTransforms(urdf::Model const &aModel,
                                              Id const aLeafId,
                                              Id const aActualLocalRootId,
                                              std::unordered_map<Id, Id> const &aLinkId2parentLinkId,
                                              std::unordered_map<Id, std::string> const &aId2name);
  static ChildTransform createChildTransform(urdf::Model const &aModel,
                                             Id const aChildId,
                                             Id const aActualLocalRootId,
                                             std::unordered_map<Id, Id> const &aLinkId2parentLinkId,
                                             std::unordered_map<Id, std::string> const &aId2name);
};

class EigenMeshModel final {
private:
  std::shared_ptr<EigenMeshJointLink>                         mRoot;
  std::unordered_map<Id, std::shared_ptr<EigenMeshJointLink>> mId2limb;
  std::unordered_map<Id, Id>                                  mId2subRoot;

public:
  EigenMeshModel(urdf::Model const &aModel,
                 std::map<std::string, std::string> const &aParentLinkTree,
                 std::unordered_set<std::string> const aForbiddenLinks,
                 std::string const &aMeshRootDirectory);
};

}
#endif //LINK_INTERSECTION_BRUTE_FORCE_EIGENMESHMODEL_H
