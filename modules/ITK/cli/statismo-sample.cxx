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

#include "utils/statismoLoggingUtils.h"

#include "statismo/ITK/itkStandardImageRepresenter.h"
#include "statismo/ITK/itkStandardMeshRepresenter.h"
#include "statismo/ITK/itkIO.h"
#include "statismo/ITK/itkStatisticalModel.h"
#include "statismo/ITK/itkDataManager.h"

#include "lpo.h"

#include <itkDirectory.h>
#include <itkImage.h>
#include <itkMesh.h>
#include <itkMeshFileReader.h>
#include <itkMeshFileWriter.h>

#include <iostream>
#include <set>
#include <string>

namespace po = lpo;
using namespace std;

namespace
{

constexpr unsigned gk_dimensionality3D = 3;
constexpr unsigned gk_dimensionality2D = 2;

using StringListType = vector<string>;

struct ProgramOptions
{
  string         strInputFileName;
  string         strOutputFileName;
  string         strType;
  StringListType vParameters;
  bool           bSampleMean{ false };
  bool           bSampleReference{ false };
  unsigned       uNumberOfDimensions{ 0 };
  bool           bIsQuiet{ false };
  std::string    strLogFile{ "" };
};

bool
IsOptionsConflictPresent(ProgramOptions & opt)
{
  statismo::utils::ToLower(opt.strType);

  return (opt.bSampleMean + !opt.vParameters.empty() + opt.bSampleReference > 1) || opt.strInputFileName.empty() ||
         opt.strOutputFileName.empty() || (opt.strType != "shape" && opt.strType != "deformation") ||
         opt.strInputFileName == opt.strOutputFileName;
}


template <class VectorType>
void
PopulateVectorWithParameters(const StringListType & paramsIn, VectorType & paramsOut)
{
  set<unsigned> indices;
  for (const auto & str : paramsIn)
  {
    bool hasSucceeded{ true };
    auto tokens = statismo::utils::Split<':'>(str);
    if (tokens.size() != 2)
    {
      hasSucceeded = false;
    }
    else
    {
      try
      {
        auto idx = statismo::utils::LexicalCast<unsigned>(tokens[0]) - 1;
        auto val = statismo::utils::LexicalCast<double>(tokens[1]);

        if (idx >= paramsOut.size())
        {
          itkGenericExceptionMacro(
            << "The parameter '" << str
            << "' is has an index value that is not in the range of this model's available parameters (1 to "
            << paramsOut.size() << ").");
        }

        if (indices.find(idx) == std::cend(indices))
        {
          indices.insert(idx);
          paramsOut[idx] = val;
        }
        else
        {
          itkGenericExceptionMacro(<< "The index '" << (idx + 1) << "' occurs more than once in the parameter list.");
        }
      }
      catch (const std::bad_cast &)
      {
        hasSucceeded = false;
      }
    }

    if (!hasSucceeded)
    {
      itkGenericExceptionMacro(
        << "The parameter '" << str
        << "' is in an incorrect format. The correct format is index:value. Like for example 0:1.1 or 19:-2");
    }
  }
}

template <class DataType, class RepresenterType, class DataWriterType>
void
DrawSampleFromModel(const ProgramOptions & opt, statismo::Logger * logger)
{
  using StatisticalModelType = itk::StatisticalModel<DataType>;

  auto representer = RepresenterType::New();
  representer->SetLogger(logger);

  auto model = itk::StatismoIO<DataType>::LoadStatisticalModel(representer, opt.strInputFileName);

  typename DataType::Pointer output;
  if (opt.bSampleMean)
  {
    output = model->DrawMean();
  }
  else if (!opt.vParameters.empty())
  {
    auto                                      paramsCount = model->GetNumberOfPrincipalComponents();
    typename StatisticalModelType::VectorType params(paramsCount);
    params.fill(0);

    PopulateVectorWithParameters<typename StatisticalModelType::VectorType>(opt.vParameters, params);
    output = model->DrawSample(params);
  }
  else if (opt.bSampleReference)
  {
    output = model->GetRepresenter()->GetReference();
  }
  else
  {
    output = model->DrawSample();
  }

  auto writer = DataWriterType::New();
  writer->SetFileName(opt.strOutputFileName);
  writer->SetInput(output);
  writer->Update();
}

} // namespace

int
main(int argc, char ** argv)
{

  ProgramOptions                                                   poParameters;
  po::program_options<std::string, StringListType, bool, unsigned> parser{ argv[0], "Program help:" };

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
    .add_opt<std::string>({ "input-file", "i", "The path to the model file.", &poParameters.strInputFileName, "" },
                          true)
    .add_flag({ "mean", "m", "Draws the mean from the model and saves it.", &poParameters.bSampleMean, true })
    .add_flag(
      { "reference", "r", "Draws the reference from the model and saves it.", &poParameters.bSampleReference, false })
    .add_opt<StringListType>(
      { "parameters",
        "p",
        "Makes it possible to specify a list of parameters and their positions that will then be used to draw a "
        "sample. Parameters are specified in the following format: POSITION1:VALUE1,...,POSITIONn:VALUEn. Unspecified "
        "parameters will be set to 0. The first parameter is at position 1.",
        &poParameters.vParameters,
        {} })
    .add_pos_opt<std::string>({ "Name of the output file/the sample.", &poParameters.strOutputFileName })
    .add_flag({ "quiet", "q", "Quiet mode (no log).", &poParameters.bIsQuiet, false })
    .add_opt<std::string>({ "log-file", "", "Path to the log file.", &poParameters.strLogFile, "" }, false);

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
    std::unique_ptr<statismo::Logger> logger{ nullptr };
    if (!poParameters.bIsQuiet)
    {
      logger = statismo::cli::CreateLogger(poParameters.strLogFile);
    }

    if (poParameters.strType == "shape")
    {
      using DataType = itk::Mesh<float, gk_dimensionality3D>;
      using RepresenterType = itk::StandardMeshRepresenter<float, gk_dimensionality3D>;
      using DataWriterType = itk::MeshFileWriter<DataType>;
      DrawSampleFromModel<DataType, RepresenterType, DataWriterType>(poParameters, logger.get());
    }
    else
    {
      if (poParameters.uNumberOfDimensions == 2)
      {
        using VectorPixelType = itk::Vector<float, gk_dimensionality2D>;
        using DataType = itk::Image<VectorPixelType, gk_dimensionality2D>;
        using RepresenterType = itk::StandardImageRepresenter<VectorPixelType, gk_dimensionality2D>;
        using DataWriterType = itk::ImageFileWriter<DataType>;
        DrawSampleFromModel<DataType, RepresenterType, DataWriterType>(poParameters, logger.get());
      }
      else
      {
        using VectorPixelType = itk::Vector<float, gk_dimensionality3D>;
        using DataType = itk::Image<VectorPixelType, gk_dimensionality3D>;
        using RepresenterType = itk::StandardImageRepresenter<VectorPixelType, gk_dimensionality3D>;
        using DataWriterType = itk::ImageFileWriter<DataType>;
        DrawSampleFromModel<DataType, RepresenterType, DataWriterType>(poParameters, logger.get());
      }
    }
  }
  catch (itk::ExceptionObject & e)
  {
    cerr << "Could not get a sample:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
