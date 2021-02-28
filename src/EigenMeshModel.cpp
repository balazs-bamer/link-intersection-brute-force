//
// Created by balazs on 2021. 02. 05..
//

#include "EigenMeshModel.h"
#include "TypedLog.h"
#include "stl_reader.h"      // TODO mention https://github.com/sreiter/stl_reader.git
#include <list>
#include <stdexcept>
#include <cstring>

namespace fragor {

fragor::Transform homogenize(fragor::Translation const &aVector) {
  fragor::Transform result{aVector};
  return result;
}

fragor::Transform homogenize(fragor::Quaternion const &aQuaternion) {
  fragor::Transform result{aQuaternion.normalized().toRotationMatrix()};
  return result;
}

}

fragor::Limb::Limb(urdf::Model const &aModel,
                   Id const aActualLocalRootId,
                   std::unordered_map<Id, Id> const &aLinkId2parentLinkId,
                   std::unordered_set<Id> const &aCurrentLimbIds,
                   std::unordered_map<Id, std::string> const &aId2name,
                   std::string const &aMeshRootDirectory)
                                               : mActualLocalRootId(aActualLocalRootId)
                                               , mLocalRootLink(aModel.getLink(aId2name.at(aActualLocalRootId)))
                                               , mJoint(mLocalRootLink->parent_joint) {
  for(auto &i : aCurrentLimbIds) { Log::n() << '-' << aId2name.at(i) << Log::end; } // TODO remove

  std::deque<HomVertex> allTransformedMeshes;
  auto remainingLinks = aCurrentLimbIds;
  std::list<Id> links2process;
  remainingLinks.erase(aActualLocalRootId);
  links2process.push_back(aActualLocalRootId);
  while(!links2process.empty()) {
    auto actualLinkId = *(links2process.begin());
    links2process.pop_front();
    auto link = aModel.getLink(aId2name.at(actualLinkId));
    std::deque<HomVertex> meshes;
    for(auto &i : link->collision_array) {
      collectMesh(i, meshes, aMeshRootDirectory);
    }
    // Some parts will be calculated more than once, but don't care, since there are only few of them.
    auto transform = createFixedTransforms(aModel, actualLinkId, aActualLocalRootId, aLinkId2parentLinkId, aId2name);
    std::transform(meshes.begin(), meshes.end(), std::back_inserter(allTransformedMeshes), [&transform](HomVertex const& aVertex) -> HomVertex {
      return transform * aVertex;
    });
    for (auto &i : aLinkId2parentLinkId) {
      if (i.second == actualLinkId) {
        if(remainingLinks.contains(i.first)) {
          remainingLinks.erase(i.first);
          links2process.push_back(i.first);
        }
        else if(!aCurrentLimbIds.contains(i.first)) {  // Found a child with non-fixed joint.
          mChildTransforms.emplace(std::make_pair(i.first, createChildTransform(aModel, i.first, aActualLocalRootId, aLinkId2parentLinkId, aId2name)));
        }
      } else { // nothing to do
      }
    }
  }
  mMesh.reserve(allTransformedMeshes.size());
  std::copy(allTransformedMeshes.begin(), allTransformedMeshes.end(), std::back_inserter(mMesh));
  Log::n() << Log::end;
}

void fragor::Limb::addChild(Limb * const aChild) {
  auto childId = aChild->getLocalRootId();
  mChildren[childId] = aChild;
  aChild->mOwnTransform = mChildTransforms[childId];
}

void fragor::Limb::readMesh(fragor::Transform const &aTransform,
                            std::string const aFilename,
                            std::deque<HomVertex> &aMeshes,
                            std::string const &aMeshRootDirectory,
                            urdf::Vector3 const &aScale) {
  stl_reader::StlMesh<float, int32_t> mesh(aMeshRootDirectory + aFilename);
  Eigen::Vector3f minCoord{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
  Eigen::Vector3f maxCoord{std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };
  for(int32_t indexTriangle = 0; indexTriangle < static_cast<int32_t>(mesh.num_tris()); ++indexTriangle) {
    for(int32_t indexCorner = 0; indexCorner < 3; ++indexCorner) {
      float const * const coords = mesh.tri_corner_coords(indexTriangle, indexCorner);
      Eigen::Vector4f in;
      in(0) = coords[0] * aScale.x;
      minCoord(0) = std::min(minCoord(0), coords[0]);
      maxCoord(0) = std::max(maxCoord(0), coords[0]);
      in(1) = coords[1] * aScale.y;
      minCoord(1) = std::min(minCoord(1), coords[1]);
      maxCoord(1) = std::max(maxCoord(1), coords[1]);
      in(2) = coords[2] * aScale.z;
      minCoord(2) = std::min(minCoord(2), coords[2]);
      maxCoord(2) = std::max(maxCoord(2), coords[2]);
      in(3) = csHomogeneousOne;
      aMeshes.push_back(aTransform * in);
    }
  }
  Log::n() << "AABB min:" << minCoord(0) << minCoord(1) << minCoord(2) << Log::end;
  Log::n() << "AABB max:" << maxCoord(0) << maxCoord(1) << maxCoord(2) << Log::end;
}

void fragor::Limb::collectMesh(urdf::CollisionSharedPtr aCollision, std::deque<fragor::HomVertex> &aMeshes, std::string const &aMeshRootDirectory) {
  auto coll = aCollision.get();
  if(coll != nullptr) {
    auto &pose = coll->origin;
    Translation translation { static_cast<float>(pose.position.x),
                              static_cast<float>(pose.position.y),
                              static_cast<float>(pose.position.z) };
    Quaternion rotation { static_cast<float>(pose.rotation.w),
                          static_cast<float>(pose.rotation.x),
                          static_cast<float>(pose.rotation.y),
                          static_cast<float>(pose.rotation.z) };
    Transform transform = homogenize(translation) * homogenize(rotation);
    auto geomSh{coll->geometry};
    auto mesh = dynamic_cast<urdf::Mesh *>(geomSh.get());
    if (mesh != nullptr) {
      std::string filename = mesh->filename;
      if (filename.starts_with(csStlNamePrefix)) {
        filename.erase(0u, std::strlen(csStlNamePrefix));
      } else { // nothing to do
      }
      readMesh(transform, filename, aMeshes, aMeshRootDirectory, mesh->scale);
      Log::n() << filename << aMeshes.size() << Log::end;
    } else {
      Log::n() << "NONE" << Log::end;
    }
  }
  else { // TODO find out how to carry on with transforms if this is missing.
  }
}

fragor::Transform fragor::Limb::createChildFixedTransform(urdf::JointSharedPtr aJoint) {
  fragor::Transform result;
  Log::n() << "joint fixed:" << aJoint->name << Log::end;
  auto &pose = aJoint->parent_to_joint_origin_transform;
  Log::n() << "pose position: " << pose.position.x << pose.position.y << pose.position.z << Log::end;
  Log::n() << "pose rotation: " << pose.rotation.w << pose.rotation.x << pose.rotation.y << pose.rotation.z << Log::end;
  Translation translation{static_cast<float>(pose.position.x),
                          static_cast<float>(pose.position.y),
                          static_cast<float>(pose.position.z)};
  Quaternion rotation{static_cast<float>(pose.rotation.w),
                      static_cast<float>(pose.rotation.x),
                      static_cast<float>(pose.rotation.y),
                      static_cast<float>(pose.rotation.z)};
  if(rotation.vec().norm() < csEpsilon) {
    result = homogenize(translation);
  }
  else {
    result = homogenize(translation) * homogenize(rotation);
  }
  Log::n() << result(0,0) << result(0, 1) << result(0,2) << result(0,3) << Log::end;
  Log::n() << result(1,0) << result(1, 1) << result(1,2) << result(1,3) << Log::end;
  Log::n() << result(2,0) << result(2, 1) << result(2,2) << result(2,3) << Log::end;
  Log::n() << result(3,0) << result(3, 1) << result(3,2) << result(3,3) << Log::end;
  return result;
}

fragor::Transform fragor::Limb::createFixedTransforms(urdf::Model const &aModel,
                                                      Id const aLeafId,
                                                      Id const aActualLocalRootId,
                                                      std::unordered_map<Id, Id> const &aLinkId2parentLinkId,
                                                      std::unordered_map<Id, std::string> const &aId2name) {
  Transform result;
  result.setIdentity();
  auto iterateLinkId = aLeafId;
  while(iterateLinkId != aActualLocalRootId) {
    auto child = aModel.getLink(aId2name.at(iterateLinkId));
    auto fixedJoint = child->parent_joint;
    result = createChildFixedTransform(fixedJoint) * result;
    Log::n() << "=>" << aId2name.at(iterateLinkId) << Log::end;
    iterateLinkId = aLinkId2parentLinkId.at(iterateLinkId);
  }
  Log::n() << "=." << Log::end;
  return result;
}

fragor::Limb::ChildTransform
fragor::Limb::createChildTransform(urdf::Model const &aModel,
                                   Id const aChildId,
                                   Id const aActualLocalRootId,
                                   std::unordered_map<Id, Id> const &aLinkId2parentLinkId,
                                   std::unordered_map<Id, std::string> const &aId2name) {
  ChildTransform result;
  auto movingJoint = aModel.getLink(aId2name.at(aChildId))->parent_joint;
  auto &axis = movingJoint->axis;
  Eigen::Vector3f vector{static_cast<float>(axis.x), static_cast<float>(axis.y), static_cast<float>(axis.z)};
  Log::n() << "axis: " << axis.x << axis.y << axis.z << Log::end;
  vector.normalize();
  if (movingJoint->type == urdf::Joint::PRISMATIC) {
    result.mPossibleUnitDisplacement = Translation{vector};
    result.mPossibleUnitRotation = Quaternion{0.0f, 1.0f, 0.0f, 0.0f};
  } else if (movingJoint->type == urdf::Joint::REVOLUTE) {
    result.mPossibleUnitDisplacement = Translation{0.0f, 0.0f, 0.0f};
    result.mPossibleUnitRotation = Quaternion{1.0f, vector.x(), vector.y(), vector.z()};
  } else { // nothing to do
  }
  result.mAllFixedTransforms =
    createFixedTransforms(aModel, aLinkId2parentLinkId.at(aChildId), aActualLocalRootId, aLinkId2parentLinkId, aId2name)
    * createChildFixedTransform(movingJoint);
  result.mJointLimitLow = movingJoint->limits->lower;
  result.mJointLimitHigh = movingJoint->limits->upper;
  result.mJointLimitEffort = movingJoint->limits->effort;
  result.mJointLimitVelocity = movingJoint->limits->velocity;
  result.mActualJointPosition = (result.mJointLimitLow + result.mJointLimitHigh) / 2.0f;
  result.mValid = true;
  return result;
}

void fragor::Limb::transformMesh(std::vector<HomVertex> * const aResult) const {
  Transform transform;
  transform.setIdentity();
  auto limb = this;
  while(limb->mParent != nullptr) {
    Translation translation{mOwnTransform.mActualJointPosition * mOwnTransform.mPossibleUnitDisplacement.x(),
      mOwnTransform.mActualJointPosition * mOwnTransform.mPossibleUnitDisplacement.y(),
      mOwnTransform.mActualJointPosition * mOwnTransform.mPossibleUnitDisplacement.z()};
    Quaternion  quaternion{mOwnTransform.mActualJointPosition * mOwnTransform.mPossibleUnitRotation.w(),
                           mOwnTransform.mPossibleUnitRotation.x(),
                           mOwnTransform.mPossibleUnitRotation.y(),
                           mOwnTransform.mPossibleUnitRotation.z()};
    transform = mOwnTransform.mAllFixedTransforms * homogenize(quaternion) * homogenize(translation) * transform;
    limb = limb->mParent;
  }
  aResult->reserve(mMesh.size());
  std::transform(mMesh.begin(), mMesh.end(), std::back_inserter(*aResult), [&transform](HomVertex const &aItem){ return transform * aItem; });
}

fragor::EigenMeshModel::EigenMeshModel(urdf::Model const &aModel,
                                       std::map<std::string, std::string> const &aParentLinkTree,
                                       std::unordered_set<std::string> const aForbiddenLinks,
                                       std::string const &aMeshRootDirectory) {
  Id nextId = 1;
  std::unordered_map<Id, std::unique_ptr<Limb>> id2limb;
  std::unordered_map<Id, Id>                    id2parentId;
  std::unordered_map<Id, Id>                    linkId2parentLinkId;
  std::unordered_map<Id, std::string>           id2name;
  std::unordered_map<std::string, Id>           name2id;
  Id rootId = 0;
  std::string rootName = aModel.getRoot()->name;
  id2name.emplace(rootId, rootName);
  name2id.emplace(rootName, rootId);
  for(auto &i: aParentLinkTree) {
    auto childName = i.first;
    id2name.emplace(nextId, childName);
    name2id.emplace(childName, nextId);
    ++nextId;
  }
  for(auto &i: aParentLinkTree) {
    if(!aForbiddenLinks.contains(i.first) && !aForbiddenLinks.contains(i.second)) {
      linkId2parentLinkId.emplace(name2id[i.first], name2id[i.second]);
    }
    else { // nothing to do
    }
  }
  std::unordered_set<Id>     links2process;
  links2process.insert(rootId);
  while(!links2process.empty()) {
    auto actualLocalRootId = *(links2process.begin());
    links2process.erase(actualLocalRootId);
    std::unordered_set<Id> currentLimbIds;
    currentLimbIds.insert(actualLocalRootId);
    bool wasIncrement;
    do {
      wasIncrement = false;
      for (auto i = linkId2parentLinkId.begin(); i != linkId2parentLinkId.end(); ++i) {                // O(n^2) but who cares? Not many links in an URDF.
        if(currentLimbIds.contains(i->second)               // second is parent
        && !currentLimbIds.contains(i->first)
        && !links2process.contains(i->first)
        && !aForbiddenLinks.contains(id2name[i->first])) {  // first is child
          auto jointType = aModel.getLink(id2name[i->first])->parent_joint->type;
          if(jointType == urdf::Joint::FIXED) {
            currentLimbIds.insert(i->first);
            wasIncrement = true;
          }
          else if(jointType == urdf::Joint::PRISMATIC || jointType == urdf::Joint::REVOLUTE) {
            links2process.insert(i->first);
          }
          else { // We trim the parent tree here to avoid considering other joint types in further processing.
            i = linkId2parentLinkId.erase(i);
          }
        }
        else { // nothing to do
        }
      }
    } while(wasIncrement);
    // now currentLimbIds contains a set of links which are fixed together and will appear as one mesh
    std::unique_ptr<Limb> limbUnique = std::make_unique<Limb>(aModel, actualLocalRootId, linkId2parentLinkId, currentLimbIds, id2name, aMeshRootDirectory);
    auto limbPointer = limbUnique.get();
    id2limb.emplace(limbPointer->getLocalRootId(), std::move(limbUnique));
    if(actualLocalRootId == rootId) {
      mRoot = limbPointer;
      limbPointer->addParent(nullptr);
    }
    else {
      auto possibleParentId = linkId2parentLinkId[actualLocalRootId];
      while(!id2limb.contains(possibleParentId)) {
        possibleParentId = linkId2parentLinkId[possibleParentId];
      }
      auto parentPointer = id2limb[possibleParentId].get();
      parentPointer->addChild(limbPointer);
      limbPointer->addParent(parentPointer);
      id2parentId.emplace(limbPointer->getLocalRootId(), parentPointer->getLocalRootId());
      Log::n() << "  -=-  " << parentPointer->getLocalRootId() << " ->" << limbPointer->getLocalRootId() << Log::end;
    }
  }
  // TODO remove log
  Log::n() << "  -id2limb-  " << id2limb.size() << Log::end;
  Log::n() << "  -id2parentId-  " << id2parentId.size() << Log::end;
  for(auto &i : id2limb) {
    Log::n() << i.second->getLocalRootLink()->name << Log::end;
  }
  mParentOfIndex.reserve(id2limb.size());
  mLimbs.reserve(id2limb.size());
  std::unordered_map<Id, Id> id2seq;
  Id seq = 0u;
  for(auto &i : id2limb) {
    mParentOfIndex[seq] = csNoParent;
    id2seq.emplace(i.first, seq);
    mLimbs.emplace(mLimbs.end(), std::move(i.second));
    ++seq;
  }
  for(auto &i: id2parentId) {
    mParentOfIndex[id2seq[i.first]] = id2seq[i.second];
  }
  // TODO remove log
  Log::n() << "  =mLimbs=  " << mLimbs.size() << Log::end;
  for(Id i = 0; i < seq; ++i) {
    Log::n() << i << "->" << mLimbs[i]->getLocalRootLink()->name << Log::end;
    Log::n() << i << "->" << mParentOfIndex[i] << Log::end;
  }
}

float fragor::EigenMeshModel::getJointPosition(Id const aIndex) const {
  if(aIndex >= getRootIndex()) {
    throw std::invalid_argument("Root limb has no posiiton.");
  }
  else {
    return mLimbs[aIndex]->getTransform().mActualJointPosition;
  }
}

void fragor::EigenMeshModel::setJointPosition(Id const aIndex, float const aPosition) {
  if(aIndex >= getRootIndex()) {
    throw std::invalid_argument("Root limb has no position.");
  }
  else {
    mLimbs[aIndex]->setJointPosition(aPosition);
  }
}

void fragor::EigenMeshModel::transformLimbMesh(Id const aIndex, std::vector<HomVertex> * const aResult) {
  if(aResult != nullptr) {
    mLimbs[aIndex]->transformMesh(aResult);
  }
  else { // nothing to do
  }
}
