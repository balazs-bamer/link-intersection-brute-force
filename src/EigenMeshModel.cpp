//
// Created by balazs on 2021. 02. 05..
//

#include "EigenMeshModel.h"
#include "TypedLog.h"
#include <list>

fragor::EigenMeshModel::EigenMeshModel(urdf::Model const &aModel, std::map<std::string, std::string> const &aParentLinkTree, std::unordered_set<std::string> const aForbiddenLinks) {
  Id nextId = 0;
  std::unordered_map<std::string, Id> name2id;
  std::list<std::string>     linksToProcess;
  linksToProcess.push_back(aModel.getRoot()->name);
  while(!linksToProcess.empty()) {
    auto actualName = *(linksToProcess.begin());
    linksToProcess.pop_front();
    name2id.emplace(actualName, nextId++);
    std::unordered_set<std::string> currentLinkNames;
    currentLinkNames.insert(actualName);
    bool wasIncrement;
    do {
      wasIncrement = false;
      for (auto &i : aParentLinkTree) {    // O(n^2) but who cares? Not many links in an URDF.
        if(currentLinkNames.contains(i.second) // second is parent
        && !currentLinkNames.contains(i.first)
        && aModel.getLink(i.first)->parent_joint->type == urdf::Joint::FIXED
        && !aForbiddenLinks.contains(i.first)) {
          currentLinkNames.insert(i.first);
          name2id.emplace(i.first, nextId++);
          wasIncrement = true;
        }
        else { // nothing to do
        }
      }
    } while(wasIncrement);
    // now currentLinkNames contains a set of links which are fixed together and will appear as one mesh
    for(auto &linkName : currentLinkNames) {
      Log::n() << linkName << Log::end;
    }
    Log::n() << Log::end;
  }
}
