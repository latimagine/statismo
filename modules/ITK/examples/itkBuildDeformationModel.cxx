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

#include "statismo/ITK/itkDataManager.h"
#include "statismo/ITK/itkPCAModelBuilder.h"
#include "statismo/ITK/itkStandardImageRepresenter.h"
#include "statismo/ITK/itkIO.h"
#include "statismo/ITK/itkStatisticalModel.h"
#include "statismo/ITK/itkUtils.h"

#include <itkImageFileReader.h>

#include <iostream>

/*
 * This example shows the ITK Wrapping of statismo can be used to build a deformation model.
 */

namespace
{
using ImageType3D = itk::Image<float, 3>;
using VectorImageType3D = itk::Image<itk::Vector<float, 3>, 3>;
using RepresenterType3D = itk::StandardImageRepresenter<itk::Vector<float, 3>, 3>;

using ImageType2D = itk::Image<float, 2>;
using VectorImageType2D = itk::Image<itk::Vector<float, 2>, 2>;
using RepresenterType2D = itk::StandardImageRepresenter<itk::Vector<float, 2>, 2>;

template <class RepresenterType, class ImageType>
void
_DoRunExample(const char * dir, const char * modelname, double noiseVariance)
{
  using ModelBuilderType = itk::PCAModelBuilder<ImageType>;
  using DataManagerType = itk::DataManager<ImageType>;
  using ImageFileReaderType = itk::ImageFileReader<ImageType>;

  auto filenames = statismo::itk::GetDirFiles(dir, ".vtk");
  assert(!filenames.empty());

  // we take an arbitrary dataset as the reference, as they have all the same resolution anyway
  auto refReader = ImageFileReaderType::New();
  refReader->SetFileName((std::string(dir) + "/" + filenames[0]));
  refReader->Update();

  auto representer = RepresenterType::New();
  representer->SetReference(refReader->GetOutput());

  auto dataManager = DataManagerType::New();
  dataManager->SetRepresenter(representer);

  for (const auto & file : filenames)
  {
    auto fullpath = (std::string(dir) + "/") + file;
    auto reader = ImageFileReaderType::New();
    reader->SetFileName(fullpath);
    reader->Update();

    typename ImageType::Pointer df = reader->GetOutput();
    dataManager->AddDataset(df, fullpath.c_str());
  }

  auto pcaModelBuilder = ModelBuilderType::New();
  auto model = pcaModelBuilder->BuildNewModel(dataManager->GetData(), noiseVariance);
  itk::StatismoIO<ImageType>::SaveStatisticalModel(model, modelname);
}
} // namespace

int
main(int argc, char * argv[])
{
  if (argc < 4)
  {
    std::cerr << "usage " << argv[0] << " dimension deformationFieldDir modelname [noiseVariance = 0]" << std::endl;
    return 1;
  }

  unsigned int dimension = std::stoul(argv[1]);
  const char * dir = argv[2];
  const char * modelname = argv[3];

  double noiseVariance = 0;
  if (argc > 4)
  {
    noiseVariance = std::stof(argv[4]);
  }

  if (dimension == 2)
  {
    _DoRunExample<RepresenterType2D, VectorImageType2D>(dir, modelname, noiseVariance);
  }
  else if (dimension == 3)
  {
    _DoRunExample<RepresenterType3D, VectorImageType3D>(dir, modelname, noiseVariance);
  }
  else
  {
    std::cerr << "invalid dimension" << std::endl;
    return 1;
  }

  std::cout << "Model building completed successfully with a noise variance of " << noiseVariance << "." << std::endl;
  return 0;
}
