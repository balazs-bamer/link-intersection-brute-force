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
    auto coll = link->collision.get();
    std::deque<HomVertex> meshes;
// perhaps needed, but not for this ROS2 implementation   collectMesh(link->collision, meshes);
    for(auto &i : link->collision_array) {
      collectMesh(i, meshes, aMeshRootDirectory);
    }
    // Some parts will be calculated more than once, but don't care, since there are only few of them.
    auto transform = createFixedTransforms(aModel, actualLinkId, aActualLocalRootId, aLinkId2parentLinkId, aId2name);
    std::transform(meshes.begin(), meshes.end(), allTransformedMeshes.begin(), [&transform](HomVertex const& aVertex) -> HomVertex {
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
  std::copy(allTransformedMeshes.begin(), allTransformedMeshes.end(), mMesh.begin());
  Log::n() << Log::end;
}

void fragor::Limb::addChild(std::shared_ptr<Limb> const &aChild) {
  auto childId = aChild->getLocalRootId();
  mChildren[childId] = aChild;
  aChild->mOwnTransform = mChildTransforms[childId];
}

void fragor::Limb::readMesh(fragor::Transform const &aTransform,
                            std::string const aFilename,
                            std::deque<HomVertex> &aMeshes,
                            std::string const &aMeshRootDirectory) {
  stl_reader::StlMesh<float, int32_t> mesh(aMeshRootDirectory + aFilename);
  for(int32_t indexTriangle = 0; indexTriangle < mesh.num_tris(); ++indexTriangle) {
    for(int32_t indexCorner = 0; indexCorner < 3; ++indexCorner) {
      float const * const coords = mesh.tri_corner_coords(indexTriangle, indexCorner);
      Eigen::Vector4f in;
      for(int32_t i = 0; i < 3; ++i) {
        in(i) = coords[i];
      }
      in(3) = csHomogeneousOne;
      aMeshes.push_back(aTransform * in);
    }
  }

}

void fragor::Limb::collectMesh(urdf::CollisionSharedPtr aCollision, std::deque<fragor::HomVertex> aMeshes, std::string const &aMeshRootDirectory) {
  auto coll = aCollision.get();
  if(coll != nullptr) {
    auto &pose = coll->origin;
    Translation translation { pose.position.x, pose.position.y, pose.position.z };
    Quaternion rotation { pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z };
    Transform transform = homogenize(translation) * homogenize(rotation);
    auto geomSh{coll->geometry};
    auto mesh = dynamic_cast<urdf::Mesh *>(geomSh.get());
    if (mesh != nullptr) {
      std::string filename = mesh->filename;
      if (filename.starts_with(csStlNamePrefix)) {
        filename.erase(0u, std::strlen(csStlNamePrefix));
      } else { // nothing to do
      }
      readMesh(transform, filename, aMeshes, aMeshRootDirectory);
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
  auto &pose = aJoint->parent_to_joint_origin_transform;
  Translation translation{pose.position.x, pose.position.y, pose.position.z};
  Quaternion rotation{pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z};
  if(rotation.vec().norm() < csEpsilon) {
    result.setIdentity();
  }
  else {
    result = homogenize(translation) * homogenize(rotation);
  }
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
  Eigen::Vector3f vector{axis.x, axis.y, axis.z};
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
  result.mValid = true;
  return result;
}

fragor::EigenMeshModel::EigenMeshModel(urdf::Model const &aModel,
                                       std::map<std::string, std::string> const &aParentLinkTree,
                                       std::unordered_set<std::string> const aForbiddenLinks,
                                       std::string const &aMeshRootDirectory) {
  Id nextId = 1;
  std::unordered_map<Id, Id> linkId2parentLinkId;
  std::unordered_map<Id, std::string> id2name;
  std::unordered_map<std::string, Id> name2id;
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
// TODO probably remove            mId2localRootId.emplace(i->first, actualLocalRootId);
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
    std::shared_ptr<Limb> limb = std::make_shared<Limb>(aModel, actualLocalRootId, linkId2parentLinkId, currentLimbIds, id2name, aMeshRootDirectory);
    mId2limb.emplace(limb->getLocalRootId(), limb);
    if(actualLocalRootId == rootId) {
      mRoot = limb;
    }
    else {
      auto possibleParentId = linkId2parentLinkId[actualLocalRootId];
      while(!mId2limb.contains(possibleParentId)) {
        possibleParentId = linkId2parentLinkId[possibleParentId];
      }
      auto parent = mId2limb[possibleParentId];
      parent->addChild(limb);
      limb->addParent(parent);
      mId2parentId.emplace(limb->getLocalRootId(), parent->getLocalRootId());
      Log::n() << "  -=-  " << parent->getLocalRootId() << " ->" << limb->getLocalRootId() << Log::end;
    }
  }
  Log::n() << "  -mId2limb-  " << mId2limb.size() << Log::end;
  Log::n() << "  -mId2parentId-  " << mId2parentId.size() << Log::end;
  for(auto &i : mId2limb) {
    Log::n() << i.second->getLocalRootLink()->name << Log::end;
  }
}
