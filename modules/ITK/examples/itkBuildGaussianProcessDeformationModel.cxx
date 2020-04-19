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

#include "statismo/core/Kernels.h"
#include "statismo/core/KernelCombinators.h"
#include "statismo/ITK/itkDataManager.h"
#include "statismo/ITK/itkLowRankGPModelBuilder.h"
#include "statismo/ITK/itkStandardImageRepresenter.h"
#include "statismo/ITK/itkIO.h"
#include "statismo/ITK/itkStatisticalModel.h"

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

/**
 * A scalar valued gaussian kernel.
 */
template <typename Point>
class GaussianKernel : public statismo::ScalarValuedKernel<Point>
{
public:
  using CoordRepType = typename Point::CoordRepType;
  using VectorType = vnl_vector<CoordRepType>;

  explicit GaussianKernel(double sigma)
    : m_sigma(sigma)
    , m_sigma2(sigma * sigma)
  {}

  inline double
  operator()(const Point & x, const Point & y) const override
  {
    VectorType xv = x.GetVnlVector();
    VectorType yv = y.GetVnlVector();

    VectorType r = yv - xv;
    return exp(-dot_product(r, r) / m_sigma2);
  }

  std::string
  GetKernelInfo() const override
  {
    std::ostringstream os;
    os << "GaussianKernel(" << m_sigma << ")";
    return os.str();
  }

private:
  double m_sigma;
  double m_sigma2;
};

template <typename RepresenterType, typename ImageType>
void
DoRunExample(const char * referenceFilename, double gaussianKernelSigma, const char * modelname)
{
  using ModelBuilderType = itk::LowRankGPModelBuilder<ImageType>;
  using ImageFileReaderType = itk::ImageFileReader<ImageType>;
  using PointType = typename ImageType::PointType;

  // we take an arbitrary dataset as the reference, as they have all the same resolution anyway
  auto refReader = ImageFileReaderType::New();
  refReader->SetFileName(referenceFilename);
  refReader->Update();

  auto representer = RepresenterType::New();
  representer->SetReference(refReader->GetOutput());

  auto gk = GaussianKernel<PointType>(gaussianKernelSigma); // a gk with sigma 100
  // make the kernel matrix valued and scale it by a factor of 100
  const auto & mvGk = statismo::UncorrelatedMatrixValuedKernel<PointType>(&gk, representer->GetDimensions());
  const auto & scaledGk = statismo::ScaledKernel<PointType>(&mvGk, 100.0);

  auto gpModelBuilder = ModelBuilderType::New();
  gpModelBuilder->SetRepresenter(representer);
  auto model = gpModelBuilder->BuildNewZeroMeanModel(scaledGk, 100);
  itk::StatismoIO<ImageType>::SaveStatisticalModel(model, modelname);
}
} // namespace

int
main(int argc, char * argv[])
{
  if (argc < 5)
  {
    std::cerr << "usage " << argv[0] << " dimension referenceFilename gaussianKernelSigma modelname" << std::endl;
    return 1;
  }

  unsigned int dimension = std::stoul(argv[1]);
  const char * referenceFilename = argv[2];
  double       gaussianKernelSigma = std::stod(argv[3]);
  const char * modelname = argv[4];

  if (dimension == 2)
  {
    DoRunExample<RepresenterType2D, VectorImageType2D>(referenceFilename, gaussianKernelSigma, modelname);
  }
  else if (dimension == 3)
  {
    DoRunExample<RepresenterType3D, VectorImageType3D>(referenceFilename, gaussianKernelSigma, modelname);
  }
  else
  {
    std::cerr << "invalid dimension" << std::endl;
    return 1;
  }

  std::cout << "Model building completed successfully." << std::endl;
  return 0;
}
