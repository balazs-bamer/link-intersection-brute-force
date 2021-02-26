//
// Created by balazs on 2021. 02. 26..
//

#ifndef LINK_INTERSECTION_BRUTE_FORCE_STLWRITER_H
#define LINK_INTERSECTION_BRUTE_FORCE_STLWRITER_H

#include <string>
#include <ostream>
#include <Eigen/Dense>

// Sets triangle normal to 0 in output. It assumes aMesh contains a multiple of 9 items.
void writeStlText(std::ostream &aOut, std::string const &aName, std::vector<Eigen::Vector4f> const &aMesh);

#endif //LINK_INTERSECTION_BRUTE_FORCE_STLWRITER_H
