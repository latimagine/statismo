/*
 * Copyright (c) 2015 University of Basel
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the project's author nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __STATISMO_BUILD_MODEL_UTILS_H_
#define __STATISMO_BUILD_MODEL_UTILS_H_

#include "statismo/ITK/itkUtils.h"

#include <itkCompensatedSummation.h>
#include <itkIdentityTransform.h>
#include <itkTransformMeshFilter.h>

#include <iostream>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <random>

namespace statismo::cli
{
  namespace details {
    template <class MeshType, class LandmarkBasedTransformInitializerType, class TransformType, class FilterType>
std::vector<typename MeshType::Pointer>
static SuperimposeMeshes(const std::vector<typename MeshType::Pointer>& originalMeshes,
                  typename MeshType::Pointer              referenceMesh,
                  const std::set<unsigned>&                      landmarkIndices)
{
  std::vector<typename MeshType::Pointer> translatedMeshes;
  translatedMeshes.reserve(originalMeshes.size());
  for (auto origMesh : originalMeshes)
  {
    using LandmarkContainerType = typename LandmarkBasedTransformInitializerType::LandmarkPointContainer;
    LandmarkContainerType                                                          movingLandmarks;
    LandmarkContainerType                                                          fixedLandmarks;
    auto                                                    movingMesh = origMesh;

    if (movingMesh->GetNumberOfPoints() != referenceMesh->GetNumberOfPoints() ||
        movingMesh->GetNumberOfCells() != referenceMesh->GetNumberOfCells())
    {
      itkGenericExceptionMacro(<< "All meshes must have the same number of Edges & Vertices");
    }

    // Only use a subset of the meshes' points for the alignment since we don't have that many degrees of freedom
    // anyways and since calculating a SVD with too many points is expensive
    for (auto idx : landmarkIndices)
    {
      movingLandmarks.push_back(movingMesh->GetPoint(idx));
      fixedLandmarks.push_back(referenceMesh->GetPoint(idx));
    }

    // only rotate & translate the moving mesh to best fit with the fixed mesh; there's no scaling taking place.
    auto landmarkBasedTransformInitializer =
      LandmarkBasedTransformInitializerType::New();
    landmarkBasedTransformInitializer->SetFixedLandmarks(fixedLandmarks);
    landmarkBasedTransformInitializer->SetMovingLandmarks(movingLandmarks);
    
    auto transform = TransformType::New();
    transform->SetIdentity();
    landmarkBasedTransformInitializer->SetTransform(transform);
    landmarkBasedTransformInitializer->InitializeTransform();

    auto filter = FilterType::New();
    filter->SetInput(movingMesh);
    filter->SetTransform(transform);
    filter->Update();

    translatedMeshes.push_back(filter->GetOutput());
  }
  return translatedMeshes;
}


template <class MeshType>
float
static CalculateMeshDistance(typename MeshType::Pointer mesh1, typename MeshType::Pointer mesh2)
{
  if (mesh1->GetNumberOfPoints() != mesh2->GetNumberOfPoints() ||
      mesh1->GetNumberOfCells() != mesh2->GetNumberOfCells())
  {
    itkGenericExceptionMacro(<< "Both meshes must have the same number of Edges & Vertices");
  }

  float                                                fDifference{0.0f};
  using IteratorType = typename MeshType::PointsContainer::Iterator;
  IteratorType                                         point1 = mesh1->GetPoints()->Begin();
  IteratorType                                         point2 = mesh2->GetPoints()->Begin();
  for (; point1 != mesh1->GetPoints()->End(); ++point1, ++point2)
  {
    fDifference += point1->Value().SquaredEuclideanDistanceTo(point2->Value());
  }
  fDifference /= (mesh1->GetNumberOfPoints() * MeshType::PointDimension);
  return fDifference;
}

template <class MeshType>
typename MeshType::Pointer
static CalculateMeanMesh(const std::vector<typename MeshType::Pointer>& meshes)
{
  if (meshes.size() == 0)
  {
    itkGenericExceptionMacro(<< "Can't calculate the mean since no meshes were provided.");
  }

  using CompensatedSummationType = ::itk::CompensatedSummation<typename MeshType::PixelType> ;
  using MeshPointsVectorType = std::vector<CompensatedSummationType>                   ;

  auto firstMesh = meshes.front();

  // prepare for summation
  MeshPointsVectorType vMeshPoints;
  unsigned             uDataSize = firstMesh->GetNumberOfPoints() * MeshType::PointDimension;
  vMeshPoints.reserve(uDataSize);
  for (int i = 0; i < uDataSize; ++i)
  {
    CompensatedSummationType sum;
    vMeshPoints.push_back(sum);
  }

  for (const auto& mesh : meshes)
  {
    if (vMeshPoints.size() != mesh->GetNumberOfPoints() * MeshType::PointDimension)
    {
      itkGenericExceptionMacro(<< "All meshes must have the same number of Edges");
    }

    auto           sum = std::begin(vMeshPoints);
    auto pointData = mesh->GetPoints()->Begin();
    
    // sum up all meshes
    for (; pointData != mesh->GetPoints()->End(); ++pointData)
    {
      auto point = pointData->Value();
      for (auto pointIter = point.Begin(); pointIter != point.End();
           ++pointIter, ++sum)
      {
        (*sum) += *pointIter;
      }
    }
  }

  float                      fInvNumberOfMeshes = 1.0f / meshes.size();
  auto meanMesh = statismo::itk::CloneMesh<MeshType>(firstMesh);

  // write the data to the mean mesh
  auto sum = std::begin(vMeshPoints);
  for (auto pointData = meanMesh->GetPoints()->Begin();
       pointData != meanMesh->GetPoints()->End();
       ++pointData)
  {
    for (auto pointIter = pointData->Value().Begin();
         pointIter != pointData->Value().End();
         ++pointIter, ++sum)
    {
      *pointIter = sum->GetSum() * fInvNumberOfMeshes;
    }
  }

  return meanMesh;
}
  }


template <class MeshType, class LandmarkBasedTransformInitializerType, class TransformType, class FilterType>
typename MeshType::Pointer
static CalculateProcrustesMeanMesh(const std::vector<typename MeshType::Pointer>& meshes,
                            unsigned                                maxIterations,
                            unsigned                                nrOfLandmarks,
                            float                                   breakIfChangeBelow)
{
  // the initial mesh to which all others will be aligned to is the first one in the list here. Any other mesh could be
  // chosen as well
  auto referenceMesh = meshes.front();

  std::random_device r;
  std::default_random_engine eng{r()};
  std::uniform_int_distribution<> urd(0, referenceMesh->GetNumberOfPoints() - 1);

  std::set<unsigned> pointNumbers;
  while (pointNumbers.size() < std::min(nrOfLandmarks, static_cast<unsigned>(referenceMesh->GetNumberOfPoints())))
  {
    pointNumbers.insert(urd(eng));
  }

  float fPreviousDifference = -1;

  for (unsigned i = 0; i < maxIterations; ++i)
  {
    // calculate the difference to the previous iteration's mesh and break if the difference is very small
    auto translatedMeshes =
      details::SuperimposeMeshes<MeshType, LandmarkBasedTransformInitializerType, TransformType, FilterType>(
        meshes, referenceMesh, pointNumbers);
    auto meanMesh = details::CalculateMeanMesh<MeshType>(translatedMeshes);
    auto                      fDifference = details::CalculateMeshDistance<MeshType>(meanMesh, referenceMesh);
    auto                      fDifferenceDelta = std::abs(fDifference - fPreviousDifference);
    fPreviousDifference = fDifference;
    referenceMesh = meanMesh;

    if (fDifferenceDelta < breakIfChangeBelow)
    {
      break;
    }
  }
  return referenceMesh;
}

static std::vector<std::string>
GetFileList(const std::string& path)
{
  std::vector<std::string> fileList;
  std::ifstream file;
  try
  {
    file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    file.open(path.c_str(), std::ifstream::in);
    std::string line;
    while (getline(file, line))
    {
      if (line != "")
      {
        // reading files with windows EOL on linux results in the \r not being removed from the line ending
        if (*std::rbegin(line) == '\r')
        {
          line.erase(line.length() - 1, 1);
        }
        fileList.push_back(line);
      }
    }
  }
  catch (const std::ifstream::failure& e)
  {
    if (file.eof() == false)
    {
      throw std::ifstream::failure("Failed to read the file '" + path + "'.");
    }
  }

  return fileList;
}

}

#endif