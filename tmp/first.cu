/*
 * first.cu
 *
 *  Created on: 2021 jan. 10
 *      Author: balazs
 */

#include "stl_reader.h"
#include <array>
#include <deque>
#include <chrono>
#include <random>
#include <vector>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <Eigen/Dense>


using Triangle = std::array<Eigen::Vector3f, 3u>;
using CudaTriangle = Eigen::Vector3f*;
using CudaConstTriangle = Eigen::Vector3f const*;
using TrianglesDeque = std::deque<Triangle>;
using TrianglesVector = std::vector<Eigen::Vector3f>;
using Result = std::vector<int8_t>;

constexpr float   cgEpsilon        = 0.00001f;       // TODO consider if uniform epsilon suits all needs.
constexpr uint32_t cgSignumZero     = 0u;
constexpr uint32_t cgSignumPlus     = 1u;
constexpr uint32_t cgSignumMinus    = 2u;
constexpr uint32_t cgSignumShift0   = 0u;
constexpr uint32_t cgSignumShift1   = 2u;
constexpr uint32_t cgSignumShift2   = 4u;
constexpr uint32_t cgSignumAllZero  = (cgSignumZero  << cgSignumShift0) | (cgSignumZero  << cgSignumShift1) | (cgSignumZero  << cgSignumShift2);
constexpr uint32_t cgSignumAllPlus  = (cgSignumPlus  << cgSignumShift0) | (cgSignumPlus  << cgSignumShift1) | (cgSignumPlus  << cgSignumShift2);
constexpr uint32_t cgSignumAllMinus = (cgSignumMinus << cgSignumShift0) | (cgSignumMinus << cgSignumShift1) | (cgSignumMinus << cgSignumShift2);

constexpr uint32_t cgSignumSelect0a = (cgSignumPlus  << cgSignumShift0) | (cgSignumMinus << cgSignumShift1) | (cgSignumMinus << cgSignumShift2);
constexpr uint32_t cgSignumSelect0b = (cgSignumMinus << cgSignumShift0) | (cgSignumPlus  << cgSignumShift1) | (cgSignumPlus  << cgSignumShift2);
constexpr uint32_t cgSignumSelect0c = (cgSignumZero  << cgSignumShift0) | (cgSignumPlus  << cgSignumShift1) | (cgSignumPlus  << cgSignumShift2);
constexpr uint32_t cgSignumSelect0d = (cgSignumZero  << cgSignumShift0) | (cgSignumMinus << cgSignumShift1) | (cgSignumMinus << cgSignumShift2);
constexpr uint32_t cgSignumSelect0e = (cgSignumPlus  << cgSignumShift0) | (cgSignumZero  << cgSignumShift1) | (cgSignumZero  << cgSignumShift2);
constexpr uint32_t cgSignumSelect0f = (cgSignumMinus << cgSignumShift0) | (cgSignumZero  << cgSignumShift1) | (cgSignumZero  << cgSignumShift2);
constexpr uint32_t cgSignumSelect0g = (cgSignumZero  << cgSignumShift0) | (cgSignumPlus  << cgSignumShift1) | (cgSignumMinus << cgSignumShift2);
constexpr uint32_t cgSignumSelect0h = (cgSignumZero  << cgSignumShift0) | (cgSignumMinus << cgSignumShift1) | (cgSignumPlus  << cgSignumShift2);

constexpr uint32_t cgSignumSelect1a = (cgSignumPlus  << cgSignumShift1) | (cgSignumMinus << cgSignumShift0) | (cgSignumMinus << cgSignumShift2);
constexpr uint32_t cgSignumSelect1b = (cgSignumMinus << cgSignumShift1) | (cgSignumPlus  << cgSignumShift0) | (cgSignumPlus  << cgSignumShift2);
constexpr uint32_t cgSignumSelect1c = (cgSignumZero  << cgSignumShift1) | (cgSignumPlus  << cgSignumShift0) | (cgSignumPlus  << cgSignumShift2);
constexpr uint32_t cgSignumSelect1d = (cgSignumZero  << cgSignumShift1) | (cgSignumMinus << cgSignumShift0) | (cgSignumMinus << cgSignumShift2);
constexpr uint32_t cgSignumSelect1e = (cgSignumPlus  << cgSignumShift1) | (cgSignumZero  << cgSignumShift0) | (cgSignumZero  << cgSignumShift2);
constexpr uint32_t cgSignumSelect1f = (cgSignumMinus << cgSignumShift1) | (cgSignumZero  << cgSignumShift0) | (cgSignumZero  << cgSignumShift2);
constexpr uint32_t cgSignumSelect1g = (cgSignumZero  << cgSignumShift1) | (cgSignumPlus  << cgSignumShift0) | (cgSignumMinus << cgSignumShift2);
constexpr uint32_t cgSignumSelect1h = (cgSignumZero  << cgSignumShift1) | (cgSignumMinus << cgSignumShift0) | (cgSignumPlus  << cgSignumShift2);

// Otherwise select 2, no need for checking and thus no constants.

constexpr uint32_t cgSignumCircumferenceA = (cgSignumZero << cgSignumShift0) | (cgSignumPlus << cgSignumShift1) | (cgSignumPlus << cgSignumShift2);
constexpr uint32_t cgSignumCircumferenceB = (cgSignumZero << cgSignumShift0) | (cgSignumZero << cgSignumShift1) | (cgSignumPlus << cgSignumShift2);
constexpr uint32_t cgSignumCircumferenceC = (cgSignumPlus << cgSignumShift0) | (cgSignumZero << cgSignumShift1) | (cgSignumPlus << cgSignumShift2);
constexpr uint32_t cgSignumCircumferenceD = (cgSignumPlus << cgSignumShift0) | (cgSignumZero << cgSignumShift1) | (cgSignumZero << cgSignumShift2);
constexpr uint32_t cgSignumCircumferenceE = (cgSignumPlus << cgSignumShift0) | (cgSignumPlus << cgSignumShift1) | (cgSignumZero << cgSignumShift2);
constexpr uint32_t cgSignumCircumferenceF = (cgSignumZero << cgSignumShift0) | (cgSignumPlus << cgSignumShift1) | (cgSignumZero << cgSignumShift2);

__device__
constexpr uint32_t calculateSignum(float const distances[3]) {
  uint32_t result = 0u;
  for(int32_t i = 0; i < 3; ++i) {
    int32_t tmp = cgSignumZero;
    if(distances[i] > cgEpsilon) {
      tmp = cgSignumPlus;
    }
    else if(distances[i] < -cgEpsilon) {
      tmp = cgSignumMinus;
    }
    else { // nothing to do
    }
    result |= tmp << (cgSignumShift1 * i);
  }
  return result;
}

__device__
void calculateNormals(Eigen::Vector2f const aShape[3], Eigen::Vector2f aNormals[3]) {
  Eigen::Vector2f side = aShape[1] - aShape[0];
  aNormals[0](0) = -side(1);
  aNormals[0](1) = side(0);
  float correction = 1.0f;
  if(aNormals[0].dot(aShape[2]) < 0.0f) {
    correction = -1.0f;                     // They shall point towards the interior.
    aNormals[0] *= correction;
  }
  else { // nothing to do
  }
  side = aShape[2] - aShape[1];
  aNormals[1](0) = -side(1) * correction;
  aNormals[1](1) = side(0) * correction;
  side = aShape[0] - aShape[2];
  aNormals[2](0) = -side(1) * correction;
  aNormals[2](1) = side(0) * correction;
}

__device__
bool doesTouchOther(uint32_t const aSignums) noexcept {
  bool result = (aSignums == cgSignumAllPlus
  || aSignums == cgSignumCircumferenceA
  || aSignums == cgSignumCircumferenceB
  || aSignums == cgSignumCircumferenceC
  || aSignums == cgSignumCircumferenceD
  || aSignums == cgSignumCircumferenceE
  || aSignums == cgSignumCircumferenceF);
  return result;
}

__device__
bool checkCornerOnPerimeterAndInterior(Eigen::Vector2f aShape1[3], Eigen::Vector2f aShape2[3]) {
  Eigen::Vector2f normals1[3]; // i : i->(i+1)%3
  Eigen::Vector2f normals2[3];
  calculateNormals(aShape1, normals1); // Normal vectors point to the center.
  calculateNormals(aShape2, normals2);
  bool result = false;
  for(int32_t indexCorner = 0; indexCorner < 3; ++indexCorner) {
    float distancesCornerFromEachSideOfShape1[3];
    float distancesCornerFromEachSideOfShape2[3];
    for(int32_t indexSide = 0; indexSide < 3; ++indexSide) {
      distancesCornerFromEachSideOfShape2[indexSide] = normals2[indexSide].dot(aShape1[indexCorner] - aShape2[indexSide]);
      distancesCornerFromEachSideOfShape1[indexSide] = normals1[indexSide].dot(aShape2[indexCorner] - aShape1[indexSide]);
    }
    result = result || doesTouchOther(calculateSignum(distancesCornerFromEachSideOfShape2));
    result = result || doesTouchOther(calculateSignum(distancesCornerFromEachSideOfShape1));   // True if one corner is on the sides, corners or interior of the other triangle.
    // Don't break out since it won't use on CUDA.
  }
  return result;
}

__device__
bool checkTrueIntersecitonOfSides(Eigen::Vector2f aShape1[3], Eigen::Vector2f aShape2[3]) {
  bool result = false;
  for(int32_t indexSide1 = 0; indexSide1 < 3; ++indexSide1) {
    for(int32_t indexSide2 = 0; indexSide2 < 3; ++indexSide2) {
      Eigen::Vector2f &side1a = aShape1[indexSide1];
      Eigen::Vector2f &side1b = aShape1[(indexSide1 + 1) % 3];
      Eigen::Vector2f &side2a = aShape2[indexSide2];
      Eigen::Vector2f &side2b = aShape2[(indexSide2 + 1) % 3];
      // Manually solve linear EQ to make sure we have as few branches as possible.
      uint32_t nonzeroAindex = (fabs(side1a(0) - side1b(0)) > cgEpsilon ? 0 : 1);
      float a = side1a(nonzeroAindex) - side1b(nonzeroAindex);
      float b = side2b(nonzeroAindex) - side2a(nonzeroAindex);
      float c = side1a(1 - nonzeroAindex) - side1b(1 - nonzeroAindex);
      float d = side2b(1 - nonzeroAindex) - side2a(1 - nonzeroAindex);
      float determinant = a * d - b * c;
      if(fabs(determinant) > cgEpsilon) {
        float k = side1a(nonzeroAindex) - side2a(nonzeroAindex);
        float l = side1a(1 - nonzeroAindex) - side2a(1 - nonzeroAindex);
        float v = (l * a - c * k) / determinant;
        float u = (k - b * v) / a;
        result = result || (u >= 0.0f && u <= 1.0f && v >= 0.0f && v <= 1.0f);  // The intersection is inside of both sides.
      }
      else { // nothing to do, because the lines are parallel but can't touch each other.
      }
    }
  }
  return result;
}

__device__
bool hasCommonPoint(CudaConstTriangle const aShape1, CudaConstTriangle const aShape2, Eigen::Vector3f const &aShape1normal, Eigen::Vector3f const &aShape2normal) { // coplanar
  Eigen::Vector3f normal;
  if(aShape1normal.dot(aShape2normal) > 0.0f) {
    normal = aShape1normal + aShape2normal;
  }
  else {
    normal = aShape1normal - aShape2normal;
  }
  int32_t indexX = 1;
  int32_t indexY = 2;
  float abs1 = fabs(normal(1)); // Looking for the biggest projection. TODO fabsf for CUDA
  if(abs1 > fabs(normal(0))) {
    indexX = 0;
  }
  else { // nothing to do
  }
  if(fabs(normal(2)) > abs1) {
    indexX = 0;
    indexY = 1;
  }
  else { // nothing to do
  }
  Eigen::Vector2f shape1[3];
  Eigen::Vector2f shape2[3];
  for(int32_t i = 0; i < 3; ++i) {
    shape1[i](0) = aShape1[i](indexX);  // Project 3D triangle to axis-parallel plane.
    shape1[i](1) = aShape1[i](indexY);
    shape2[i](0) = aShape2[i](indexX);
    shape2[i](1) = aShape2[i](indexY);
  }
  bool result = checkCornerOnPerimeterAndInterior(shape1, shape2);
  if(!result) { // Common point may only occur now when sides truly intersect each other.
    result = checkTrueIntersecitonOfSides(shape1, shape2);
  }
  else { // nothing to do
  }
  return result;
}

__device__
void calculateIntersectionParameter(
  CudaConstTriangle const aShape
, Eigen::Vector3f const &aIntersectionVector
, float const aDistanceCornerNfromOtherPlane[3]
, uint32_t const aSignumShapeFromOtherPlane
, float &aIntersectionParameterA
, float &aIntersectionParameterB) noexcept {
  int32_t indexCommon, indexA, indexB;
  if(aSignumShapeFromOtherPlane == cgSignumSelect0a
  || aSignumShapeFromOtherPlane == cgSignumSelect0b
  || aSignumShapeFromOtherPlane == cgSignumSelect0c
  || aSignumShapeFromOtherPlane == cgSignumSelect0d
  || aSignumShapeFromOtherPlane == cgSignumSelect0e
  || aSignumShapeFromOtherPlane == cgSignumSelect0f
  || aSignumShapeFromOtherPlane == cgSignumSelect0g
  || aSignumShapeFromOtherPlane == cgSignumSelect0h) {
    indexCommon = 0; indexA = 1; indexB = 2;
  }
  else if(aSignumShapeFromOtherPlane == cgSignumSelect1a
  || aSignumShapeFromOtherPlane == cgSignumSelect1b
  || aSignumShapeFromOtherPlane == cgSignumSelect1c
  || aSignumShapeFromOtherPlane == cgSignumSelect1d
  || aSignumShapeFromOtherPlane == cgSignumSelect1e
  || aSignumShapeFromOtherPlane == cgSignumSelect1f
  || aSignumShapeFromOtherPlane == cgSignumSelect1g
  || aSignumShapeFromOtherPlane == cgSignumSelect1h) {
    indexCommon = 1; indexA = 0; indexB = 2;
  }
  else {
    indexCommon = 2; indexA = 1; indexB = 0;
  }
  float vertexProjections[3];
  for(int32_t i = 0; i < 3; ++i) {
    vertexProjections[i] = aIntersectionVector.dot(aShape[i]);
  }
  aIntersectionParameterA =
   vertexProjections[indexA]
 + (vertexProjections[indexCommon] - vertexProjections[indexA])
 * aDistanceCornerNfromOtherPlane[indexA]
 / (aDistanceCornerNfromOtherPlane[indexA] - aDistanceCornerNfromOtherPlane[indexCommon]);
  aIntersectionParameterB =
   vertexProjections[indexB]
 + (vertexProjections[indexCommon] - vertexProjections[indexB])
 * aDistanceCornerNfromOtherPlane[indexB]
 / (aDistanceCornerNfromOtherPlane[indexB] - aDistanceCornerNfromOtherPlane[indexCommon]);
}

__global__
void hasCommonPoint(int32_t aPairCount, CudaConstTriangle const aShapes, int32_t const * const aIndices, int8_t * const aResult) {
  int32_t indexThread = blockIdx.x*blockDim.x + threadIdx.x;
  if(indexThread < aPairCount) {
    bool result = false;
    CudaConstTriangle shape1 = aShapes + aIndices[indexThread * 2] * 3;
    CudaConstTriangle shape2 = aShapes + aIndices[indexThread * 2 + 1] * 3;
    Eigen::Vector3f shape1normal = (shape1[1] - shape1[0]).cross(shape1[2] - shape1[0]);
    Eigen::Vector3f shape2normal = (shape2[1] - shape2[0]).cross(shape2[2] - shape2[0]);
    shape1normal.normalize();
    shape2normal.normalize();
    float distanceCornerNofShape1FromPlane2[3];
    float distanceCornerNofShape2FromPlane1[3];
    for(int32_t i = 0; i < 3; ++i) {
      distanceCornerNofShape1FromPlane2[i] = shape2normal.dot(shape1[i] - shape2[0]);
      distanceCornerNofShape2FromPlane1[i] = shape1normal.dot(shape2[i] - shape1[0]);
    }
    uint32_t signumShape1FromPlane2 = calculateSignum(distanceCornerNofShape1FromPlane2); // These contain info about relation of each point and the other plane.
    uint32_t signumShape2FromPlane1 = calculateSignum(distanceCornerNofShape2FromPlane1);
    if(signumShape1FromPlane2 == cgSignumAllPlus || signumShape1FromPlane2 == cgSignumAllMinus || signumShape2FromPlane1 == cgSignumAllPlus || signumShape2FromPlane1 == cgSignumAllMinus) {
      // Nothing to do: one triangle is completely on the one side of the other's plane
    }
    else {
      Eigen::Vector3f intersectionVector = shape1normal.cross(shape2normal);
      if(intersectionVector.norm() > cgEpsilon && signumShape1FromPlane2 != cgSignumAllZero && signumShape2FromPlane1 != cgSignumAllZero) { // Real intersection, planes are not identical, and both triangles touch the common line.
        intersectionVector.normalize();
        float intersectionParameterAshape1;
        float intersectionParameterBshape1;
        float intersectionParameterAshape2;
        float intersectionParameterBshape2;
        calculateIntersectionParameter(shape1, intersectionVector, distanceCornerNofShape1FromPlane2, signumShape1FromPlane2, intersectionParameterAshape1, intersectionParameterBshape1); // The two parameters will contain the locations of the touching point.
        calculateIntersectionParameter(shape2, intersectionVector, distanceCornerNofShape2FromPlane1, signumShape2FromPlane1, intersectionParameterAshape2, intersectionParameterBshape2);
        if(intersectionParameterAshape1 > intersectionParameterBshape1) {
          auto tmp = intersectionParameterAshape1;
          intersectionParameterAshape1 = intersectionParameterBshape1;
          intersectionParameterBshape1 = tmp;
        }
        else { // nothing to do
        }
        if(intersectionParameterAshape2 > intersectionParameterBshape2) {
          auto tmp = intersectionParameterAshape2;
          intersectionParameterAshape2 = intersectionParameterBshape2;
          intersectionParameterBshape2 = tmp;
        }
        else { // nothing to do
        }
        if((intersectionParameterAshape1 - cgEpsilon <= intersectionParameterAshape2) && (intersectionParameterAshape2 <= intersectionParameterBshape1 + cgEpsilon) // Epsilons make possible to check for triangles with corner-corner or corner-edge touch.
        || (intersectionParameterAshape1 - cgEpsilon <= intersectionParameterBshape2) && (intersectionParameterBshape2 <= intersectionParameterBshape1 + cgEpsilon)
        || (intersectionParameterAshape2 - cgEpsilon <= intersectionParameterAshape1) && (intersectionParameterAshape1 <= intersectionParameterBshape2 + cgEpsilon)
        || (intersectionParameterAshape2 - cgEpsilon <= intersectionParameterBshape1) && (intersectionParameterBshape1 <= intersectionParameterBshape2 + cgEpsilon)) {
          result = true;
        }
        else { // nothing to do
        }
      }
      else {
        result = hasCommonPoint(shape1, shape2, shape1normal, shape2normal); // Coplanar triangles
      }
    }
    aResult[indexThread] = (result ? 1 : 0);
  }
  else { // nothing to do
  }
}

Eigen::Matrix3f randomTransform(std::default_random_engine &aGenerator) {
  std::uniform_real_distribution<float> distribution(0.0f, 1.0f);
  Eigen::Matrix3f result;
  for(int32_t i = 0; i < 9; ++i) {
    result(i / 3, i % 3) = distribution(aGenerator);
  }
  return result;
}

TrianglesDeque readTriangles(char const * const aFilename) {
  TrianglesDeque result;
  stl_reader::StlMesh<float, int32_t> mesh(aFilename);
  for(int32_t indexTriangle = 0; indexTriangle < mesh.num_tris(); ++indexTriangle) {
      Triangle triangle;
      for(int32_t indexCorner = 0; indexCorner < 3; ++indexCorner) {
          float const * const coords = mesh.tri_corner_coords(indexTriangle, indexCorner);
          Eigen::Vector3f in;
          for(int32_t i = 0; i < 3; ++i) {
            in(i) = coords[i];
          }
          triangle[indexCorner] = in;
      }
      result.push_back(triangle);
  }
  return result;
}

void writeTriangles(TrianglesDeque const &aTriangles, char const * const aFilename) {
  std::ofstream out(aFilename);
  out << "solid Exported from Blender-2.82 (sub 7)\n";
  for(auto const & triangle : aTriangles) {
//    Eigen::Vector3f normal = (triangle[1] - triangle[0]).cross(aShape2[2] - aShape2[0]);
//  shape1normal.normalize();
    out << "facet normal 0.000000 0.000000 0.000000\nouter loop\n";
    for(auto const & vertex : triangle) {
      out << "vertex " << vertex(0) << ' ' << vertex(1) << ' ' << vertex(2) << '\n';
    }
    out << "endloop\nendfacet\n";
  }
  out << "endsolid Exported from Blender-2.82 (sub 7)\n";
}

void check(TrianglesVector const &aTriangles, Result &aResult) {
  int32_t count = aTriangles.size() / 3;
  int32_t pairCount = count * (count - 1) / 2;
  int32_t bytesTriangles = count * 3 * sizeof(Eigen::Vector3f);
  int32_t bytesIndices = pairCount * 2 * sizeof(int32_t);

  CudaTriangle trianglesCuda;
  cudaMalloc(&trianglesCuda, bytesTriangles);
  cudaMemcpy(trianglesCuda, aTriangles.data(), bytesTriangles, cudaMemcpyHostToDevice);

  std::vector<int32_t> indicesTmp;
  indicesTmp.reserve(pairCount * 2);
  int32_t unified = 0;
  for(int32_t i = 0; i < count; ++i) {
    for(int32_t j = i + 1; j < count; ++j) {
      indicesTmp[unified++] = i;
      indicesTmp[unified++] = j;
    }
  }
  int32_t *indicesCuda;
  cudaMalloc(&indicesCuda, bytesIndices);
  cudaMemcpy(indicesCuda, indicesTmp.data(), bytesIndices, cudaMemcpyHostToDevice);

  int8_t *resultCuda;
  cudaMalloc(&resultCuda, pairCount);

  hasCommonPoint<<<(pairCount + 255) / 256, 256>>>(pairCount, trianglesCuda, indicesCuda, resultCuda);

  cudaMemcpy(aResult.data(), resultCuda, pairCount, cudaMemcpyDeviceToHost);

  cudaFree(indicesCuda);
  cudaFree(resultCuda);
  cudaFree(trianglesCuda);
}

int main(int argc, char **argv) {
  int ret = 0;
  size_t iterations = 1;
  if(argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <filenameIn> <count>\n";
    ret = 1;
  }
  else {
    try {
      iterations = std::stoul(argv[2]);
      if(iterations == 0u) {
        throw std::invalid_argument("Count must be > 0");
      }
      else { // nothing to do
      }
      std::default_random_engine generator;
      generator.seed((std::chrono::high_resolution_clock::now() - std::chrono::high_resolution_clock::time_point::min()).count());
      auto triangles = readTriangles(argv[1]);
      TrianglesVector trianglesTransform;
      trianglesTransform.reserve(triangles.size() * 3);
      Result result;
      auto count = triangles.size();
      auto pairCount = count * (count - 1u) / 2u;
      result.reserve(pairCount);
      std::fill_n(result.begin(), pairCount, 0);

      auto start = std::chrono::high_resolution_clock::now();
      for(size_t i = 0; i < iterations; ++i) {
        auto transform = randomTransform(generator);
        trianglesTransform.clear();
        for(auto &item : triangles) {
          std::transform(item.begin(), item.end(), std::back_inserter(trianglesTransform), [&transform](auto &item) -> Eigen::Vector3f {
            Eigen::Vector3f result = transform * item;
            return result;
          });
        }
        check(trianglesTransform, result);
      }
      auto finish = std::chrono::high_resolution_clock::now();

      size_t unified = 0;
      for(int32_t i = 0; i < count; ++i) {
        for(int32_t j = i + 1; j < count; ++j) {
          if(result[unified]) {
            std::cout << "Has common point: " << i << ' ' << j << "\n";
          }
          else { // nothing to do
          }
          ++unified;
        }
      }
      double totalTime = std::chrono::duration_cast<std::chrono::duration<double>>(finish - start).count();

      std::cout << "Average time: " << totalTime / iterations << '\n';
    }
    catch(std::exception &e) {
      std::cerr << "exception: " << e.what() << '\n';
      ret = 2;
    }
  }
  return ret;
}
