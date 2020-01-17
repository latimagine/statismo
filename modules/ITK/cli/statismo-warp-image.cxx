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

#include "lpo.h"

#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkWarpImageFilter.h>

#include <string>

namespace po = lpo;
using namespace std;

namespace {

constexpr unsigned _Dimensionality3D = 3;
constexpr unsigned _Dimensionality2D = 2;

struct _ProgramOptions
{
  string   strInputImageFileName;
  string   strInputDeformFieldFileName;
  string   strOutputFileName;
  unsigned uNumberOfDimensions{0};
};

bool
_IsOptionsConflictPresent(const _ProgramOptions & opt)
{
  return opt.strInputDeformFieldFileName.empty() ||
  opt.strInputImageFileName.empty() ||
  opt.strOutputFileName.empty();
}

template <unsigned Dimensions>
void
_ApplyDeformationFieldToImage(const _ProgramOptions & opt)
{
  using ImageType = itk::Image<float, Dimensions>   ;
  using ImageReaderType = itk::ImageFileReader<ImageType> ;
  using VectorPixelType = itk::Vector<float, Dimensions>          ;
  using VectorImageType = itk::Image<VectorPixelType, Dimensions> ;
  using VectorImageReaderType = itk::ImageFileReader<VectorImageType>   ;
  using InterpolatorType = itk::LinearInterpolateImageFunction<ImageType, double> ;
  using WarpFilterType = itk::WarpImageFilter<ImageType, ImageType, VectorImageType> ;
  using ImageWriterType = itk::ImageFileWriter<ImageType> ;

  auto       origImgReader = ImageReaderType::New();
  origImgReader->SetFileName(opt.strInputImageFileName);
  origImgReader->Update();

  typename ImageType::Pointer origImg = origImgReader->GetOutput();


  auto         defFieldReader = VectorImageReaderType::New();
  defFieldReader->SetFileName(opt.strInputDeformFieldFileName);
  defFieldReader->Update();
  typename VectorImageType::Pointer defField = defFieldReader->GetOutput();

  auto                            interpolator = InterpolatorType::New();

  auto warper = WarpFilterType::New();
  warper->SetInput(origImg);
  warper->SetInterpolator(interpolator);
  warper->SetOutputSpacing(origImg->GetSpacing());
  warper->SetOutputOrigin(origImg->GetOrigin());
  warper->SetOutputDirection(origImg->GetDirection());
  warper->SetDisplacementField(defField);
  warper->Update();

  typename ImageType::Pointer warpedImg = warper->GetOutput();

  auto writer = ImageWriterType::New();
  writer->SetInput(warpedImg);
  writer->SetFileName(opt.strOutputFileName);
  writer->Update();
}

}

int
main(int argc, char ** argv)
{
  _ProgramOptions                              poParameters;
  po::program_options<std::string, unsigned> parser{ argv[0], "Program help:" };

  parser
    .add_opt<unsigned>({ "dimensionality",
                         "d",
                         "Dimensionality of the input image (only available if you're building a deformation model)",
                         &poParameters.uNumberOfDimensions,
                         3,
                         2,
                         3 },
                       true)
    .add_opt<std::string>(
      { "input-image", "i", "The path to the original image.", &poParameters.strInputImageFileName, "" }, true)
    .add_opt<std::string>(
      { "input-deformation-field", "f", "The path to the original image.", &poParameters.strInputDeformFieldFileName, "" },
      true)
    .add_pos_opt<std::string>({ "Name of the warped output image.", &poParameters.strOutputFileName });

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
      _ApplyDeformationFieldToImage<_Dimensionality2D>(poParameters);
    }
    else
    {
      _ApplyDeformationFieldToImage<_Dimensionality3D>(poParameters);
    }
  }
  catch (itk::ExceptionObject & e)
  {
    cerr << "Could not warp the image:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}