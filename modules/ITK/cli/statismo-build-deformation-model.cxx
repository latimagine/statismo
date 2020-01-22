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

#include "utils/statismoBuildModelUtils.h"

#include "statismo/ITK/itkDataManager.h"
#include "statismo/ITK/itkPCAModelBuilder.h"
#include "statismo/ITK/itkStandardImageRepresenter.h"
#include "statismo/ITK/itkIO.h"
#include "statismo/ITK/itkStatisticalModel.h"

#include "lpo.h"

#include <itkImageFileReader.h>

namespace po = lpo;
using namespace std;

namespace
{

struct _ProgramOptions
{
  bool     bComputeScores{ true };
  string   strDataListFile;
  string   strOutputFileName;
  float    fNoiseVariance{ 0.0 };
  unsigned uNumberOfDimensions{ 0 };
};

bool
_IsOptionsConflictPresent(const _ProgramOptions & opt)
{
  return opt.strDataListFile.empty() || opt.strOutputFileName.empty() || (opt.strDataListFile == opt.strOutputFileName);
}

template <unsigned Dimensions>
void
_BuildAndSaveDeformationModel(const _ProgramOptions & opt)
{
  using VectorPixelType = itk::Vector<float, Dimensions>;
  using ImageType = itk::Image<VectorPixelType, Dimensions>;
  using RepresenterType = itk::StandardImageRepresenter<VectorPixelType, Dimensions>;
  using DataManagerType = itk::DataManager<ImageType>;
  using ImageReaderType = itk::ImageFileReader<ImageType>;
  using ModelBuilderType = itk::PCAModelBuilder<ImageType>;

  auto representer = RepresenterType::New();
  auto dataManager = DataManagerType::New();
  auto fileNames = statismo::cli::GetFileList(opt.strDataListFile);

  if (fileNames.empty())
  {
    itkGenericExceptionMacro(<< "No Data was loaded and thus the model can't be built.");
  }

  bool firstPass{ true };
  for (const auto & file : fileNames)
  {
    auto reader = ImageReaderType::New();
    reader->SetFileName(file);
    reader->Update();

    if (firstPass)
    {
      representer->SetReference(reader->GetOutput());
      dataManager->SetRepresenter(representer);
      firstPass = false;
    }

    dataManager->AddDataset(reader->GetOutput(), reader->GetFileName().c_str());
  }

  auto pcaModelBuilder = ModelBuilderType::New();
  auto model = pcaModelBuilder->BuildNewModel(dataManager->GetData(), opt.fNoiseVariance, opt.bComputeScores);
  itk::StatismoIO<ImageType>::SaveStatisticalModel(model, opt.strOutputFileName);
}
} // namespace

int
main(int argc, char ** argv)
{
  _ProgramOptions                                         poParameters;
  po::program_options<std::string, float, unsigned, bool> parser{ argv[0], "Program help:" };

  parser
    .add_opt<std::string>({ "data-list",
                            "l",
                            "File containing a list of meshes to build the deformation model from",
                            &poParameters.strDataListFile,
                            "" },
                          true)
    .add_opt<unsigned>(
      { "dimensionality", "d", "Dimensionality of the input image", &poParameters.uNumberOfDimensions, 3, 2, 3 }, true)
    .add_opt<float>({ "noise", "n", "Noise variance of the PPCA model", &poParameters.fNoiseVariance, 0.0f, 0.0f })
    .add_opt<bool>({ "scores", "s", "Compute scores (default true)", &poParameters.bComputeScores, true })
    .add_pos_opt<std::string>({ "Name of the output file", &poParameters.strOutputFileName });

  if (!parser.parse(argc, argv))
  {
    return EXIT_FAILURE;
  }

  if (_IsOptionsConflictPresent(poParameters))
  {
    cerr << "A conflict in the options exists or insufficient options were set." << endl;
    cout << parser << endl;
    return EXIT_FAILURE;
  }

  try
  {
    if (poParameters.uNumberOfDimensions == 2)
    {
      _BuildAndSaveDeformationModel<2>(poParameters);
    }
    else
    {
      _BuildAndSaveDeformationModel<3>(poParameters);
    }
  }
  catch (const ifstream::failure & e)
  {
    cerr << "Could not read the data-list:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }
  catch (const itk::ExceptionObject & e)
  {
    cerr << "Could not build the model:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
