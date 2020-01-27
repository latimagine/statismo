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

// Add new kernels in this file (and document their usage in the statismo-build-gp-model.md file)
#include "utils/statismoBuildGPModelKernels.h"

#include "statismo/core/Utils.h"
#include "statismo/ITK/itkDataManager.h"
#include "statismo/ITK/itkLowRankGPModelBuilder.h"
#include "statismo/ITK/itkStandardImageRepresenter.h"
#include "statismo/ITK/itkStandardMeshRepresenter.h"
#include "statismo/ITK/itkIO.h"
#include "statismo/ITK/itkStatisticalModel.h"

#include "lpo.h"

#include <itkImageFileReader.h>
#include <itkMeshFileReader.h>

#include <iostream>
#include <memory>

namespace po = lpo;
using namespace std;

namespace
{

struct ProgramOptions
{
  string         strOptionalModelPath;
  string         strReferenceFile;
  string         strKernel;
  string         strType;
  vector<string> vKernelParameters;
  float          fKernelScale{ 0.0 };
  int            iNrOfBasisFunctions{ 0 };
  unsigned       uNumberOfDimensions{ 0 };
  string         strOutputFileName;
};

string
GetAvailableKernelsStr()
{
  string ret;

  for (const auto & p : statismo::cli::s_kernelMap)
  {
    ret += p.first + ",";
  }

  ret.pop_back();
  return ret;
}

bool
IsOptionsConflictPresent(ProgramOptions & opt)
{
  statismo::utils::ToLower(opt.strKernel);
  statismo::utils::ToLower(opt.strType);

  return (opt.strType != "shape" && opt.strType != "deformation") ||
         (opt.strReferenceFile.empty() && opt.strOptionalModelPath.empty()) ||
         (!opt.strReferenceFile.empty() && !opt.strOptionalModelPath.empty()) ||
         opt.strOutputFileName == opt.strReferenceFile || (opt.strType == "shape" && opt.uNumberOfDimensions != 3);
}

template <class DataType, class RepresenterType, class DataReaderType, bool IS_SHAPE_MODEL, unsigned DIM>
void
BuildAndSaveModel(const ProgramOptions & opt)
{
  auto it = statismo::cli::s_kernelMap.find(opt.strKernel);
  if (it == std::end(statismo::cli::s_kernelMap))
  {
    itkGenericExceptionMacro(<< "The kernel '" << opt.strKernel
                             << "' isn't available. Available kernels: " << GetAvailableKernelsStr());
  }

  using PointType = typename DataType::PointType;
  using MatrixPointerType = std::unique_ptr<statismo::ScalarValuedKernel<PointType>>;
  using KernelPointerType = std::unique_ptr<statismo::MatrixValuedKernel<PointType>>;
  using RawModelType = statismo::StatisticalModel<DataType>;
  using RawModelPointerType = statismo::UniquePtrType<RawModelType>;
  using DatasetPointerType = typename RepresenterType::DatasetPointerType;

  MatrixPointerType kernel;
  if constexpr ((std::is_same_v<DataType, statismo::cli::DataTypeShape>)) // NOLINT
  {
    kernel = std::move(it->second.createKernelShape(opt.vKernelParameters));
  }
  else if constexpr ((std::is_same_v<DataType, statismo::cli::DataType2DDeformation>)) // NOLINT
  {
    kernel = std::move(it->second.createKernel2DDeformation(opt.vKernelParameters));
  }
  else if constexpr ((std::is_same_v<DataType, statismo::cli::DataType3DDeformation>)) // NOLINT
  {
    kernel = std::move(it->second.createKernel3DDeformation(opt.vKernelParameters));
  }

  KernelPointerType unscaledKernel(new statismo::UncorrelatedMatrixValuedKernel<PointType>(kernel.get(), DIM));
  KernelPointerType scaledKernel(new statismo::ScaledKernel<PointType>(unscaledKernel.get(), opt.fKernelScale));

  KernelPointerType   statModelKernel;
  KernelPointerType   modelBuildingKernel;
  RawModelPointerType rawStatisticalModel;
  DatasetPointerType  mean;

  auto representer = RepresenterType::New();
  if (!opt.strOptionalModelPath.empty())
  {
    try
    {
      rawStatisticalModel =
        statismo::IO<DataType>::LoadStatisticalModel(representer.GetPointer(), opt.strOptionalModelPath);
      statModelKernel.reset(new statismo::StatisticalModelKernel<DataType>(rawStatisticalModel.get()));
      modelBuildingKernel.reset(new statismo::SumKernel<PointType>(statModelKernel.get(), scaledKernel.get()));
    }
    catch (const statismo::StatisticalModelException & s)
    {
      itkGenericExceptionMacro(<< "Failed to read the optional model: " << s.what());
    }

    mean = rawStatisticalModel->DrawMean();
  }
  else
  {
    modelBuildingKernel = std::move(scaledKernel);

    auto refReader = DataReaderType::New();
    refReader->SetFileName(opt.strReferenceFile);
    refReader->Update();
    representer->SetReference(refReader->GetOutput());
    mean = representer->IdentitySample();
  }

  using ModelBuilderType = itk::LowRankGPModelBuilder<DataType>;
  auto gpModelBuilder = ModelBuilderType::New();
  gpModelBuilder->SetRepresenter(representer);

  auto model = gpModelBuilder->BuildNewModel(mean, *modelBuildingKernel.get(), opt.iNrOfBasisFunctions);

  itk::StatismoIO<DataType>::SaveStatisticalModel(model, opt.strOutputFileName);
}

} // namespace

int
main(int argc, char ** argv)
{
  statismo::cli::CreateKernelMap();

  ProgramOptions poParameters;
  string         kernelHelp =
    "Specifies the kernel (covariance function). The following kernels are available: " + GetAvailableKernelsStr();
  po::program_options<std::string, float, int, unsigned, std::vector<std::string>> parser{ argv[0], "Program help:" };

  parser
    .add_opt<std::string>({ "type",
                            "t",
                            "Specifies the type of the model: shape and deformation are the two available types",
                            &poParameters.strType,
                            "shape" },
                          true)
    .add_opt<unsigned>({ "dimensionality",
                         "d",
                         "Dimensionality of the input image (only available if you're building a deformation model)",
                         &poParameters.uNumberOfDimensions,
                         3,
                         2,
                         3 },
                       true)
    .add_opt<std::string>({ "kernel", "k", kernelHelp, &poParameters.strKernel, "" }, true)
    .add_opt<std::vector<std::string>>({ "parameters",
                                         "p",
                                         "Specifies the kernel parameters. The Parameters depend on the kernel",
                                         &poParameters.vKernelParameters,
                                         {} },
                                       true)
    .add_opt<float>(
      { "scale", "s", "A Scaling factor with which the Kernel will be scaled", &poParameters.fKernelScale, 1.0f }, true)
    .add_opt<int>({ "numberofbasisfunctions",
                    "n",
                    "Number of basis functions/parameters the model will have",
                    &poParameters.iNrOfBasisFunctions,
                    0,
                    1 },
                  true)
    .add_opt<std::string>(
      { "reference", "r", "The reference that will be used to build the model", &poParameters.strReferenceFile, "" })
    .add_opt<std::string>({ "input-model",
                            "m",
                            "Extends an existing model with data from the specified kernel. This is useful to extend "
                            "existing models in case of insufficient data.",
                            &poParameters.strOptionalModelPath,
                            "" })
    .add_pos_opt<std::string>({ "Name of the output file", &poParameters.strOutputFileName });


  if (!parser.parse(argc, argv))
  {
    return EXIT_FAILURE;
  }

  if (IsOptionsConflictPresent(poParameters))
  {
    cerr << "A conflict in the options exists or insufficient options were set." << endl;
    cout << parser << endl;
    return EXIT_FAILURE;
  }

  try
  {
    if (poParameters.strType == "shape")
    {
      using RepresenterType = itk::StandardMeshRepresenter<float, statismo::cli::gk_dimensionality3D>;
      using DataReaderType = itk::MeshFileReader<statismo::cli::DataTypeShape>;
      BuildAndSaveModel<statismo::cli::DataTypeShape,
                        RepresenterType,
                        DataReaderType,
                        true,
                        statismo::cli::gk_dimensionality3D>(poParameters);
    }
    else
    {
      if (poParameters.uNumberOfDimensions == 2)
      {
        using RepresenterType =
          itk::StandardImageRepresenter<statismo::cli::VectorPixel2DType, statismo::cli::gk_dimensionality2D>;
        using DataReaderType = itk::ImageFileReader<statismo::cli::DataType2DDeformation>;
        BuildAndSaveModel<statismo::cli::DataType2DDeformation,
                          RepresenterType,
                          DataReaderType,
                          false,
                          statismo::cli::gk_dimensionality2D>(poParameters);
      }
      else
      {
        using RepresenterType =
          itk::StandardImageRepresenter<statismo::cli::VectorPixel3DType, statismo::cli::gk_dimensionality3D>;
        using DataReaderType = itk::ImageFileReader<statismo::cli::DataType3DDeformation>;
        BuildAndSaveModel<statismo::cli::DataType3DDeformation,
                          RepresenterType,
                          DataReaderType,
                          false,
                          statismo::cli::gk_dimensionality3D>(poParameters);
      }
    }
  }
  catch (itk::ExceptionObject & e)
  {
    cerr << "Could not build the model:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
