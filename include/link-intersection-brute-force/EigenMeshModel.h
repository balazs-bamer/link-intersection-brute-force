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

class Limb {
private:
  struct ChildTransform {
    bool                                mValid = false;
    float                               mJointLimitLow;
    float                               mJointLimitHigh;
    float                               mJointLimitEffort;
    float                               mJointLimitVelocity;
    Transform                           mAllFixedTransforms;
    Translation                         mPossibleUnitDisplacement; // Prism direction for prismatic joint.
    Quaternion                          mPossibleUnitRotation;     // Rotation axis with 1 radian unit for rotate joint.
    float                               mActualJointPosition;      // Initially average of low and high limit.
    // Transforms come so from local parent to non-fixed joint child:
    // 1. All of fixed transform across the fixed joints.
    // 2. Constant displacement for this joint.
    // 3. Constant rotation for this joint.
    // 4. Actual rotation for rotate joint. | Actual joint displacement for prismatic joint. - These on urdf::Joint.axis
  };
  static constexpr float                 csEpsilon         = 1e-6f;
  static constexpr float                 csHomogeneousOne  = 1.0f;
  inline static constexpr char           csStlNamePrefix[] = "package://";

  // TODO later incorporate inertia calculated for the whole limb.
  Id const                                      mActualLocalRootId;
  ChildTransform                                mOwnTransform;
  urdf::LinkConstSharedPtr const                mLocalRootLink;
  urdf::JointSharedPtr     const                mJoint;                 // Joint holding this stuff.
  Limb const *                                  mParent;                // Used to travel backwards for composition.
  std::vector<HomVertex>                        mMesh;                  // Cumulative, in the mesh coordinate space of the local root link.
  std::unordered_map<Id, Limb const*>           mChildren;              // Maps from the child's local root id to the actual object
  std::unordered_map<Id, ChildTransform>        mChildTransforms;       // Maps from the child's local root id to its transform.

public:
  Limb(urdf::Model const &aModel,
       Id const aActualLocalRootId,
       std::unordered_map<Id, Id> const &aLinkId2parentLinkId,
       std::unordered_set<Id> const &aCurrentLimbIds,
       std::unordered_map<Id, std::string> const &aId2name,
       std::string const &aMeshRootDirectory);
  ~Limb() = default;
  void                     addChild(Limb * const aChild);
  void                     addParent(Limb const * const aParent) { mParent = aParent; }
  Id                       getLocalRootId() const                { return mActualLocalRootId; }
  ChildTransform const&    getTransform() const                  { return mOwnTransform; }
  urdf::LinkConstSharedPtr getLocalRootLink() const              { return mLocalRootLink; }
  void                     setJointPosition(float const aPos)    { mOwnTransform.mActualJointPosition = aPos; }
  void                     transformMesh(std::vector<HomVertex> * const aResult) const;

private:
  void                  readMesh(Transform const &aTransform,
                                 std::string const aFilename,
                                 std::deque<HomVertex> &aMeshes,
                                 std::string const &aMeshRootDirectory,
                                 urdf::Vector3 const &aScale);
    void                  collectMesh(urdf::CollisionSharedPtr aCollision, std::deque<HomVertex> &aMeshes, std::string const &aMeshRootDirectory);
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
  static constexpr Id   csNoParent = -1;

  Limb const*                        mRoot;
  std::vector<Id>                    mParentOfIndex;  // For each index, its parent is in the array. For root, -1.
  std::vector<std::unique_ptr<Limb>> mLimbs;          // The indices in these vectors are DIFFERENT then those used
                                                      // in Limbs and during construction.

public:
  EigenMeshModel(urdf::Model const &aModel,
                 std::map<std::string, std::string> const &aParentLinkTree,
                 std::unordered_set<std::string> const aForbiddenLinks,
                 std::string const &aMeshRootDirectory);

  int32_t size() const                          { return mLimbs.size(); }
  Id      getRootIndex() const                  { return mLimbs.size() - 1; }
  Id      operator[](Id const aIndex) const     { return mParentOfIndex[aIndex]; }
  float getJointPosition(Id const aIndex) const;
  void  setJointPosition(Id const aIndex, float const aPosition);
  float getLimitLow(Id const aIndex) const      { return mLimbs[aIndex]->getTransform().mJointLimitLow; }
  float getLimitHigh(Id const aIndex) const     { return mLimbs[aIndex]->getTransform().mJointLimitHigh; }
  float getLimitEffort(Id const aIndex) const   { return mLimbs[aIndex]->getTransform().mJointLimitEffort; }
  float getLimitVelocity(Id const aIndex) const { return mLimbs[aIndex]->getTransform().mJointLimitVelocity; }
  std::string getName(Id const aIndex) const    { return mLimbs[aIndex]->getLocalRootLink()->name; }

  void  transformLimbMesh(Id const aIndex, std::vector<HomVertex> * const aResult);
};

}
#endif //LINK_INTERSECTION_BRUTE_FORCE_EIGENMESHMODEL_H
