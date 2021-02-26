//
// Created by balazs on 2021. 02. 26..
//

#include <string>
#include <ostream>
#include <Eigen/Dense>

void writeStlText(std::ostream &aOut, std::string const &aName, std::vector<Eigen::Vector4f> const &aMesh) {
  aOut << "solid " << aName << '\n';
  for(auto i = aMesh.begin(); i != aMesh.end(); ++i) {
    aOut << "facet normal 0.0e+000 0.0e+000 0.0e+000\n  outer loop\n";
    aOut << "    vertex " << std::scientific << i->x() << ' ' << (++i)->y() << ' ' << (++i)->z();
    aOut << "    vertex " << std::scientific << (++i)->x() << ' ' << (++i)->y() << ' ' << (++i)->z();
    aOut << "    vertex " << std::scientific << (++i)->x() << ' ' << (++i)->y() << ' ' << (++i)->z();
    aOut << "  endloop\nendfacet\n";
  }
  aOut << "endsolid " << aName << '\n';
}
