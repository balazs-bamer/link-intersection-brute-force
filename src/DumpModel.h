//
// Created by balazs on 2021. 02. 05..
//

#ifndef LINK_INTERSECTION_BRUTE_FORCE_DUMPMODEL_H
#define LINK_INTERSECTION_BRUTE_FORCE_DUMPMODEL_H

#include "urdf/model.h"

void dumpModelInfo (urdf::Model &aModel, std::map<std::string, std::string> &aParentLinkTree);

#endif //LINK_INTERSECTION_BRUTE_FORCE_DUMPMODEL_H
