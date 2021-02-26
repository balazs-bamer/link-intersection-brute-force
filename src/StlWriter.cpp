//
// Created by balazs on 2021. 02. 26..
//

#include <string>
#include <ostream>
#include <Eigen/Dense>

void writeStlText(std::ostream &aOut, std::string const &aName, std::vector<Eigen::Vector4f> const &aMesh) {
  aOut << "solid " << aName << '\n';
  for(size_t i = 0; i < aMesh.size(); ) {
    aOut << "facet normal 0.0e+000 0.0e+000 0.0e+000\n  outer loop\n";
    aOut << "    vertex " << std::scientific << aMesh[i++].x() << ' ' << aMesh[i++].y() << ' ' << aMesh[i++].z() << '\n';
    aOut << "    vertex " << std::scientific << aMesh[i++].x() << ' ' << aMesh[i++].y() << ' ' << aMesh[i++].z() << '\n';
    aOut << "    vertex " << std::scientific << aMesh[i++].x() << ' ' << aMesh[i++].y() << ' ' << aMesh[i++].z() << '\n';
    aOut << "  endloop\nendfacet\n";
  }
  aOut << "endsolid " << aName << '\n';
}
