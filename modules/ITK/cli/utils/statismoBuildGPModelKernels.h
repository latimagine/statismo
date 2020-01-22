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

#ifndef __STATISMO_BUILD_GP_MODEL_KERNELS_H_
#define __STATISMO_BUILD_GP_MODEL_KERNELS_H_

#include <statismo/core/KernelCombinators.h>
#include <statismo/core/Kernels.h>

#include <itkImage.h>
#include <itkMesh.h>

#include <map>
#include <string>
#include <memory>

/*
You can add your kernels here:
  1. Either #include your kernel or put the class in here
  2. Write a function in this file that creates your kernel (create it on the heap and NOT on the stack; it will be
deleted, don't worry) Your function will receive a vector containing the kernel arguments (vector<string>). You have to
parse them or thrown an itk::ExceptionObject if something goes wrong.
  3. Add your kernel to the kernel map (do this in the function \a CreateKernelMap(); your kernel name has to be
lowercase)
  4. Document your kernel in the statismo-build-gp-model.md file. (Examples for kernels with multiple parameters are in
there, but currently commented out)
*/

namespace statismo::cli
{

//
// Internal type
//

constexpr unsigned Dimensionality3D = 3;
using DataTypeShape = itk::Mesh<float, Dimensionality3D>;
using VectorPixel3DType = itk::Vector<float, Dimensionality3D>;
using DataType3DDeformation = itk::Image<VectorPixel3DType, Dimensionality3D>;
constexpr unsigned Dimensionality2D = 2;
using VectorPixel2DType = itk::Vector<float, Dimensionality2D>;
using DataType2DDeformation = itk::Image<VectorPixel2DType, Dimensionality2D>;

//
// Internal structure
//

struct KernelContainer
{
  std::function<std::unique_ptr<statismo::ScalarValuedKernel<DataTypeShape::PointType>>(
    const std::vector<std::string> &)>
    createKernelShape;
  std::function<std::unique_ptr<statismo::ScalarValuedKernel<DataType3DDeformation::PointType>>(
    const std::vector<std::string> &)>
    createKernel3DDeformation;
  std::function<std::unique_ptr<statismo::ScalarValuedKernel<DataType2DDeformation::PointType>>(
    const std::vector<std::string> &)>
    createKernel2DDeformation;
};

//
// Internal kernel map used to retrieve the different kernels
//

using KernelMapType = std::map<std::string, KernelContainer>;
static KernelMapType s_kernelMap;

//
// Kernel definitions
//

template <class TPoint>
class GaussianKernel : public statismo::ScalarValuedKernel<TPoint>
{
public:
  using CoordRepType = typename TPoint::CoordRepType;
  using VectorType = vnl_vector<CoordRepType>;

  explicit GaussianKernel(double sigma)
    : m_sigma(sigma)
    , m_sigma2(-1.0 / (sigma * sigma))
  {}

  inline double
  operator()(const TPoint & x, const TPoint & y) const override
  {
    VectorType xv = x.GetVnlVector();
    VectorType yv = y.GetVnlVector();

    VectorType r = yv - xv;

    return exp((double)dot_product(r, r) * m_sigma2);
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


template <class TPoint>
class BSplineKernel : public statismo::ScalarValuedKernel<TPoint>
{
public:
  using CoordRepType = typename TPoint::CoordRepType;
  using VectorType = vnl_vector<CoordRepType>;

  explicit BSplineKernel(double support)
    : m_support(support)
  {}

  double
  bspline3(const double & x) const
  {
    const double absX = std::fabs(x);
    const double absXSquared = absX * absX;
    const double absXCube = absXSquared * absX;
    const double twoMinAbsX = 2.0 - absX;
    const double twoMinAbsXCube = twoMinAbsX * twoMinAbsX * twoMinAbsX;
    const double twoByThree = 2.0 / 3.0;

    double splineValue = 0;
    if (absX >= 0 && absX < 1)
    {
      splineValue = twoByThree - absXSquared + 0.5 * absXCube;
    }
    else if (absX >= 1 && absX < 2)
    {
      splineValue = twoMinAbsXCube / 6.0;
    }
    else
    {
      splineValue = 0;
    }

    return splineValue;
  }

  double
  TensorProductSpline(const VectorType & x) const
  {
    double prod{ 1.0 };
    for (unsigned d = 0; d < x.size(); ++d)
    {
      prod *= bspline3(x[d]);
    }
    return prod;
  }

  inline double
  operator()(const TPoint & x, const TPoint & y) const override
  {
    assert(x.Size() == y.Size());
    const unsigned dim = x.Size();

    if (dim < 1 || dim > 3)
    {
      std::ostringstream os;
      os << "Currently only dimensions 1 - 3 are supported.  ( Received" << dim << ")";
      throw statismo::StatisticalModelException(os.str().c_str(), statismo::Status::BAD_INPUT_ERROR);
    }

    const double supportBasisFunction = 4.0;
    const double scale = -1.0 * std::log(m_support / supportBasisFunction) / std::log(2.0);

    VectorType xScaled = x.GetVnlVector() * std::pow(2.0, scale);
    VectorType yScaled = y.GetVnlVector() * std::pow(2.0, scale);

    std::vector<int> kLower(dim);
    std::vector<int> kUpper(dim);
    for (unsigned d = 0; d < dim; ++d)
    {
      kLower[d] = static_cast<int>(std::ceil(std::max(xScaled[d], yScaled[d]) - 0.5 * supportBasisFunction));
      kUpper[d] = static_cast<int>(std::floor(std::min(xScaled[d], yScaled[d]) + 0.5 * supportBasisFunction));
    }

    // We need to generate the cartesian product k_1 x ... x k_d, where k_i goes through all the integers
    // within the given bounds. A non-recursive solution requires d loops. Here we just write down the cases
    // for 1 2 and 3D
    double sum = 0.0;
    double kx = kLower[0];
    while (kx <= kUpper[0])
    {
      if (dim == 1)
      {
        VectorType k(1);
        k[0] = kx;
        sum += (TensorProductSpline(xScaled - k) * TensorProductSpline(yScaled - k));
      }
      else
      {
        double ky = kLower[1];
        while (ky <= kUpper[1])
        {
          if (dim == 2)
          {
            VectorType k(2);
            k[0] = kx;
            k[1] = ky;
            sum += (TensorProductSpline(xScaled - k) * TensorProductSpline(yScaled - k));
          }
          else
          {
            double kz = kLower[2];
            while (kz <= kUpper[2])
            {
              VectorType k(3);
              k[0] = kx;
              k[1] = ky;
              k[2] = kz;
              sum += (TensorProductSpline(xScaled - k) * TensorProductSpline(yScaled - k));
              kz += 1;
            }
          }
          ky += 1;
        }
      }

      kx += 1;
    }

    return sum;
  }

  std::string
  GetKernelInfo() const override
  {
    std::ostringstream os;
    os << "BSplineKernel(" << m_support << ")";
    return os.str();
  }

private:
  double m_support;
};

//
// A kernel that represents deformations on different scale levels.
// Each scale level is modeled as a b-spline kernel, which leads to a "wavelet style"
// decomposition of the deformation. See
// Opfer, Roland. "Multiscale kernels." Advances in computational mathematics 25.4 (2006): 357-380.
//
template <class TPoint>
class MultiscaleKernel : public statismo::ScalarValuedKernel<TPoint>
{
public:
  using CoordRepType = typename TPoint::CoordRepType;
  using VectorType = vnl_vector<CoordRepType>;

  MultiscaleKernel(double supportBaseLevel, unsigned numberOfLevels)
    : m_supportBaseLevel(supportBaseLevel)
    , m_numberOfLevels(numberOfLevels)
  {
    for (unsigned i = 0; i < numberOfLevels; ++i)
    {
      m_kernels.push_back(std::make_unique<BSplineKernel<TPoint>>(m_supportBaseLevel * std::pow(2, -1.0 * i)));
      m_kernelWeights.push_back(std::pow(2, -1.0 * i));
    }
  }

  inline double
  operator()(const TPoint & x, const TPoint & y) const override
  {
    assert(x.Size() == y.Size());
    double sum = 0;
    for (unsigned i = 0; i < m_kernels.size(); ++i)
    {
      sum += (*m_kernels[i])(x, y) * m_kernelWeights[i];
    }

    return sum;
  }

  std::string
  GetKernelInfo() const override
  {
    std::ostringstream os;
    os << "MultiscaleKernel(" << m_supportBaseLevel << ", " << m_numberOfLevels << ")";
    return os.str();
  }

private:
  double                                              m_supportBaseLevel;
  unsigned                                            m_numberOfLevels;
  std::vector<std::unique_ptr<BSplineKernel<TPoint>>> m_kernels;
  std::vector<double>                                 m_kernelWeights;
};

//
// Kernel builders
//

template <class TPoint>
struct GaussianKernelBuilder
{
  static std::unique_ptr<statismo::ScalarValuedKernel<TPoint>>
  CreateKernel(const std::vector<std::string> & kernelArgs)
  {
    if (kernelArgs.size() == 1)
    {
      try
      {
        auto sigma = statismo::utils::LexicalCast<double>(kernelArgs[0]);
        if (sigma <= 0)
        {
          itkGenericExceptionMacro(<< "Error: sigma has to be > 0");
        }
        return std::make_unique<GaussianKernel<TPoint>>(sigma);
      }
      catch (const std::bad_cast &)
      {
        itkGenericExceptionMacro(<< "Error: could not parse the kernel argument (expected a floating point number)");
      }
    }
    else
    {
      itkGenericExceptionMacro(<< "The Gaussian Kernel takes one and only one argument: sigma. You provided "
                               << kernelArgs.size() << " Arguments.");
    }
  }
};

template <class TPoint>
struct MultiscaleKernelBuilder
{
  static std::unique_ptr<statismo::ScalarValuedKernel<TPoint>>
  CreateKernel(const std::vector<std::string> & kernelArgs)
  {
    if (kernelArgs.size() == 2)
    {
      try
      {
        auto baseLevel = statismo::utils::LexicalCast<double>(kernelArgs[0]);
        auto numberOfLevels = statismo::utils::LexicalCast<unsigned>(kernelArgs[1]);

        if (baseLevel <= 0)
        {
          itkGenericExceptionMacro(<< "Error: baselevel has to be > 0");
        }
        return std::make_unique<MultiscaleKernel<TPoint>>(baseLevel, numberOfLevels);
      }
      catch (const std::bad_cast &)
      {
        itkGenericExceptionMacro(<< "Error: could not parse the kernel argument");
      }
    }
    else
    {
      itkGenericExceptionMacro(<< "The Multiscale Kernel takes two arguments: the support of the base level and the "
                                  "number of levels. You provided"
                               << kernelArgs.size() << " Arguments.");
    }
  }
};

//
// Kernel map assessor and initializers
//

// Create static kernel map
template <typename T>
using KernelBuilderType = std::function<const statismo::ScalarValuedKernel<T> *(const std::vector<std::string> &)>;

template <template <typename> typename Builder>
static void
AddKernelToKernelMap(const std::string & kernelName)
{
  KernelContainer kernel;
  kernel.createKernel3DDeformation = &Builder<DataType3DDeformation::PointType>::CreateKernel;
  kernel.createKernel2DDeformation = &Builder<DataType2DDeformation::PointType>::CreateKernel;
  kernel.createKernelShape = &Builder<DataTypeShape::PointType>::CreateKernel;
  s_kernelMap[kernelName] = kernel;
}

static void
CreateKernelMap()
{
  AddKernelToKernelMap<GaussianKernelBuilder>("gaussian");
  AddKernelToKernelMap<MultiscaleKernelBuilder>("multiscale");
}
} // namespace statismo::cli

#endif