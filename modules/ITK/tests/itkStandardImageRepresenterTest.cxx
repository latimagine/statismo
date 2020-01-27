/*
 * This file is part of the statismo library.
 *
 * Author: Marcel Luethi (marcel.luethi@unibas.ch)
 *
 * Copyright (c) 2011 University of Basel
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
#include "StatismoUnitTest.h"
#include "statismo/core/Exceptions.h"
#include "statismo/core/GenericRepresenterValidator.h"
#include "statismo/ITK/itkStandardImageRepresenter.h"

#include <string>

namespace
{
using ScalarImageType = itk::Image<float, 2>;
using VectorImageType = itk::Image<itk::Vector<float, 2>, 2>;

std::string g_dataDir;

template <typename T>
typename T::Pointer
LoadImage(const std::string & filename)
{
  auto reader = itk::ImageFileReader<T>::New();
  reader->SetFileName(filename);
  reader->Update();

  typename T::Pointer img = reader->GetOutput();
  img->DisconnectPipeline();
  return img;
}

int
TestRepresenterForScalarImage()
{
  using RepresenterType = itk::StandardImageRepresenter<float, 2>;
  using RepresenterValidatorType = GenericRepresenterValidator<RepresenterType>;

  auto referenceFilename = g_dataDir + "/hand_images/hand-1.vtk";
  auto testDatasetFilename = g_dataDir + "/hand_images/hand-2.vtk";

  auto representer = RepresenterType::New();
  auto reference = LoadImage<ScalarImageType>(referenceFilename);
  representer->SetReference(reference);

  // choose a test dataset, a point and its associate pixel value

  auto                       testDataset = LoadImage<ScalarImageType>(testDatasetFilename);
  ScalarImageType::IndexType idx;
  idx.Fill(0);
  ScalarImageType::PointType testPt;
  reference->TransformIndexToPhysicalPoint(idx, testPt);
  auto testValue = testDataset->GetPixel(idx);

  RepresenterValidatorType validator(representer, testDataset, std::make_pair(testPt, testValue));

  return (validator.RunAllTests() ? EXIT_SUCCESS : EXIT_FAILURE);
}

int
TestRepresenterForVectorImage()
{
  using RepresenterType = itk::StandardImageRepresenter<itk::Vector<float, 2>, 2>;
  using RepresenterValidatorType = GenericRepresenterValidator<RepresenterType>;

  auto referenceFilename = g_dataDir + "/hand_dfs/df-hand-1.vtk";
  auto testDatasetFilename = g_dataDir + "/hand_dfs/df-hand-2.vtk";

  auto representer = RepresenterType::New();
  auto reference = LoadImage<VectorImageType>(referenceFilename);
  representer->SetReference(reference);

  // choose a test dataset, a point and its associate pixel value

  auto                       testDataset = LoadImage<VectorImageType>(testDatasetFilename);
  VectorImageType::IndexType idx;
  idx.Fill(0);
  VectorImageType::PointType testPt;
  reference->TransformIndexToPhysicalPoint(idx, testPt);
  VectorImageType::PixelType testValue = testDataset->GetPixel(idx);

  RepresenterValidatorType validator(representer, testDataset, std::make_pair(testPt, testValue));

  return (validator.RunAllTests() ? EXIT_SUCCESS : EXIT_FAILURE);
}
} // namespace

int
itkStandardImageRepresenterTest(int argc, char ** argv) // NOLINT
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " datadir" << std::endl;
    return EXIT_FAILURE;
  }

  g_dataDir = argv[1];

  auto res = statismo::Translate([]() {
    return statismo::test::RunAllTests("itkStandardImageRepresenterTest",
                                       {
                                         { "TestRepresenterForScalarImage", TestRepresenterForScalarImage },
                                         { "TestRepresenterForVectorImage", TestRepresenterForVectorImage },
                                       });
  });

  return !statismo::CheckResultAndAssert(res, EXIT_SUCCESS);
}
