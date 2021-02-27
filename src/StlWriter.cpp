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
    for(size_t j = 0; j < 3; ++j) {
      aOut << "    vertex " << std::scientific << aMesh[i].x() << ' ' << aMesh[i].y() << ' ' << aMesh[i].z() << '\n';
      i++;
    }
    aOut << "  endloop\nendfacet\n";
  }
  aOut << "endsolid " << aName << '\n';
}

void writeStlBinary(std::ostream &aOut, std::string const &aName, std::vector<Eigen::Vector4f> const &aMesh) {
  constexpr std::streamsize cBufferSize = 80;
  char buffer[cBufferSize];
  std::string prefix{"name:"};
  std::memcpy(buffer, prefix.c_str(), prefix.size());
  std::memcpy(buffer + prefix.size(), aName.c_str(), std::min(aName.size(), cBufferSize - prefix.size()));
  aOut.write(buffer, cBufferSize);
  uint32_t count = aMesh.size() / 3u;
  std::memcpy(buffer, &count, sizeof(count));
  aOut.write(buffer, sizeof(count));
  size_t index = 0u;
  for(uint32_t which = 0u; which < count; ++which) {
    float value = 0.0f;
    std::memcpy(buffer, &value, sizeof(value));
    aOut.write(buffer, sizeof(value));
    aOut.write(buffer, sizeof(value));
    aOut.write(buffer, sizeof(value));
    for(uint32_t vertex = 0u; vertex < 3u; ++vertex) {
      value = aMesh[index].x();
      std::memcpy(buffer, &value, sizeof(value));
      aOut.write(buffer, sizeof(value));
      value = aMesh[index].y();
      std::memcpy(buffer, &value, sizeof(value));
      aOut.write(buffer, sizeof(value));
      value = aMesh[index].z();
      std::memcpy(buffer, &value, sizeof(value));
      aOut.write(buffer, sizeof(value));
      ++index;
    }
    uint16_t attribute = 0u;
    std::memcpy(buffer, &attribute, sizeof(attribute));
    aOut.write(buffer, sizeof(attribute));
  }
}
