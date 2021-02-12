//
// Created by balazs on 2021. 02. 05..
//

#include "EigenMeshModel.h"
#include "TypedLog.h"
#include <list>
#include <stdexcept>
#include <cstring>

fragor::EigenMeshJointLink::EigenMeshJointLink(urdf::Model const &aModel,
                                               Id const aActualLocalRootId,
                                               std::unordered_map<Id, Id> const &aLinkId2parentLinkId,
                                               std::unordered_set<Id> const &aCurrentLimbIds,
                                               std::unordered_map<Id, std::string> const &aId2name)
                                               : mActualLocalRootId(aActualLocalRootId)
                                               , mFirstLink(aModel.getLink(aId2name.at(aActualLocalRootId)))
                                               , mJoint(mFirstLink->parent_joint) {
  auto remainingLinks = aCurrentLimbIds;
  std::list<Id> links2process;
  remainingLinks.erase(aActualLocalRootId);
  links2process.push_back(aActualLocalRootId);
  while(!links2process.empty()) {
    auto actualLinkId = *(links2process.begin());
    links2process.pop_front();
    auto coll = aModel.getLink(aId2name.at(actualLinkId))->collision.get();
    std::string filename;
    if(coll != nullptr) {
      auto geomSh{coll->geometry};
      auto mesh = dynamic_cast<urdf::Mesh *>(geomSh.get());
      if (mesh != nullptr) {
        filename = mesh->filename;
        if(filename.starts_with(csStlNamePrefix)) {
          filename.erase(0u, std::strlen(csStlNamePrefix));
        }
        else { // nothing to do
        }
        Log::n() << aId2name.at(actualLinkId) << filename << Log::end;
      } else {
        Log::n() << aId2name.at(actualLinkId) << "NONE" << Log::end;
      }
    }
    else { // TODO check how to carry on with transformation chain
    }
    for (auto &i : aLinkId2parentLinkId) {
      if (i.second == actualLinkId) {
        if(remainingLinks.contains(i.first)) {
          remainingLinks.erase(i.first);
          links2process.push_back(i.first);
        }
        else {  // Found a child with non-fixed joint.
          // TODO process
        }
      } else { // nothing to do
      }
    }
  }
  Log::n() << Log::end;
}

void fragor::EigenMeshJointLink::addChild(std::shared_ptr<EigenMeshJointLink> aChild) {

}

void fragor::EigenMeshJointLink::addParent(std::shared_ptr<EigenMeshJointLink> aParent) {
  mParent = aParent;
}

fragor::EigenMeshModel::EigenMeshModel(urdf::Model const &aModel, std::map<std::string, std::string> const &aParentLinkTree, std::unordered_set<std::string> const aForbiddenLinks) {
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
    linkId2parentLinkId.emplace(name2id[i.first], name2id[i.second]);
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
            mId2subRoot.emplace(i->first, actualLocalRootId);
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
    std::shared_ptr<EigenMeshJointLink> limb = std::make_shared<EigenMeshJointLink>(aModel, actualLocalRootId, linkId2parentLinkId, currentLimbIds, id2name);
    for(auto &i : currentLimbIds) {
      mId2limb.emplace(i, limb);
    }
    if(actualLocalRootId == rootId) {
      mRoot = limb;
    }
    else {
      auto parent = mId2limb[linkId2parentLinkId[actualLocalRootId]];
      parent->addChild(limb);
      limb->addParent(parent);
    }
  }
}
