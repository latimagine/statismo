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

#ifndef __STATISMO_BUILD_POSTERIOR_MODEL_UTILS_H_
#define __STATISMO_BUILD_POSTERIOR_MODEL_UTILS_H_

#include "statismo/ITK/itkPosteriorModelBuilder.h"

#include <vector>
#include <string>
#include <fstream>

namespace statismo::cli
{
namespace details
{
template <class DataType>
static std::vector<typename DataType::PointType>
ReadLandmarksFile(const std::string & path)
{
  std::vector<typename DataType::PointType> vLandmarks;

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
        // typedef boost::tokenizer<boost::escaped_list_separator<char> > TokenizerType;
        // TokenizerType t(line);
        // TODO: Replace with a real csv tokenizer that can handle coma in escaped string
        auto                         t = statismo::utils::Split<','>(line);
        typename DataType::PointType p;
        auto                         pointIter = p.Begin();
        // The first element is the description/name and will be ignored
        for (auto it = ++std::begin(t); it != std::end(t); ++it, ++pointIter)
        {
          try
          {
            auto fCoordValue = statismo::utils::LexicalCast<float>(*it);
            if (pointIter == p.End())
            {
              // ignore the last point if it is equal to 0 in the 2D case (and it is really is the last point)
              if (p.Size() == 2 && ++it == t.end())
              {
                if (fCoordValue == 0)
                {
                  break;
                }
                else
                {
                  itkGenericExceptionMacro(<< "The last point in the 2D case has to be 0 in the following line: '"
                                           << line << "' (file: " << path << ")");
                }
              }
              else
              {
                itkGenericExceptionMacro(<< "Too many point components were found in this line: '" << line
                                         << "' (file: " << path << ")");
              }
            }
            *pointIter = fCoordValue;
          }
          catch (const std::bad_cast & e)
          {
            itkGenericExceptionMacro(<< "Could not parse '" << (*it) << "' to a float in this line: '" << line
                                     << "' in the file '" << path << "'");
          }
        }
        if (pointIter != p.End())
        {
          itkGenericExceptionMacro(<< "Not enough point components were found in this line: '" << line
                                   << "' (file: " << path << ")");
        }
        vLandmarks.push_back(p);
      }
    }
  }
  catch (const std::ifstream::failure & e)
  {
    if (file.eof() == false)
    {
      throw std::ifstream::failure("Failed to read a file: '" + path + "'");
    }
  }

  return vLandmarks;
}
} // namespace details

template <class DataType, class StatisticalModelType>
static typename StatisticalModelType::Pointer
BuildPosteriorDeformationModel(typename StatisticalModelType::Pointer model,
                               const std::string &                    fixedLandmarksFileName,
                               const std::string &                    movingLandmarksFileName,
                               double                                 dLandmarksVariance)
{
  auto fixedLandmarks = details::ReadLandmarksFile<DataType>(fixedLandmarksFileName);
  auto movingLandmarks = details::ReadLandmarksFile<DataType>(movingLandmarksFileName);

  if (fixedLandmarks.size() != movingLandmarks.size())
  {
    itkGenericExceptionMacro(<< "There have to be an equal number of fixed and moving Landmarks.")
  }

  typename StatisticalModelType::PointValueListType constraints;
  auto                                              fixedIt = std::begin(fixedLandmarks);
  for (auto movingIt = std::begin(movingLandmarks); movingIt != std::end(movingLandmarks); ++movingIt, ++fixedIt)
  {
    typename DataType::PixelType disp;
    for (unsigned i = 0; i < fixedIt->Size(); ++i)
    {
      disp[i] = (*movingIt)[i] - (*fixedIt)[i];
    }
    constraints.emplace_back(*fixedIt, disp);
  }

  using PosteriorModelBuilderType = ::itk::PosteriorModelBuilder<DataType>;

  auto posteriorModelBuilder = PosteriorModelBuilderType::New();
  return posteriorModelBuilder->BuildNewModelFromModel(model, constraints, dLandmarksVariance, false);
}


template <class DataType, class StatisticalModelType>
static typename StatisticalModelType::Pointer
BuildPosteriorShapeModel(typename StatisticalModelType::Pointer model,
                         typename DataType::Pointer             mesh,
                         double                                 dVariance)
{
  if (mesh->GetNumberOfPoints() != model->GetRepresenter()->GetReference()->GetNumberOfPoints())
  {
    itkGenericExceptionMacro(<< "The provided Mesh is not in correspondence.")
  }

  typename StatisticalModelType::PointValueListType constraints;

  auto fixedIt = model->GetRepresenter()->GetReference()->GetPoints()->Begin();
  for (auto movingIt = mesh->GetPoints()->Begin(); movingIt != mesh->GetPoints()->End(); ++movingIt, ++fixedIt)
  {
    constraints.emplace_back(fixedIt->Value(), movingIt->Value());
  }

  using PosteriorModelBuilderType = ::itk::PosteriorModelBuilder<DataType>;
  auto posteriorModelBuilder = PosteriorModelBuilderType::New();
  return posteriorModelBuilder->BuildNewModelFromModel(model, constraints, dVariance, false);
}

template <class DataType, class StatisticalModelType, class PointsLocatorType>
static typename StatisticalModelType::Pointer
BuildPosteriorShapeModel(typename StatisticalModelType::Pointer model,
                         const std::string &                    fixedLandmarksFileName,
                         const std::string &                    movingLandmarksFileName,
                         double                                 dLandmarksVariance)
{
  auto fixedLandmarks = details::ReadLandmarksFile<DataType>(fixedLandmarksFileName);
  auto movingLandmarks = details::ReadLandmarksFile<DataType>(movingLandmarksFileName);

  if (fixedLandmarks.size() != movingLandmarks.size())
  {
    itkGenericExceptionMacro(<< "There have to be an equal number of fixed and moving Landmarks.")
  }

  typename DataType::Pointer pReference = model->GetRepresenter()->GetReference();
  auto                       pPointLocator = PointsLocatorType::New();
  pPointLocator->SetPoints(pReference->GetPoints());
  pPointLocator->Initialize();

  const auto *                                      referenceMeshPoints = pReference->GetPoints();
  typename StatisticalModelType::PointValueListType constraints;

  for (unsigned i = 0; i < fixedLandmarks.size(); ++i)
  {
    constraints.emplace_back(referenceMeshPoints->at(pPointLocator->FindClosestPoint(fixedLandmarks[i])),
                             movingLandmarks[i]);
  }

  using PosteriorModelBuilderType = ::itk::PosteriorModelBuilder<DataType>;
  auto posteriorModelBuilder = PosteriorModelBuilderType::New();
  return posteriorModelBuilder->BuildNewModelFromModel(model, constraints, dLandmarksVariance, false);
}

} // namespace statismo::cli

#endif