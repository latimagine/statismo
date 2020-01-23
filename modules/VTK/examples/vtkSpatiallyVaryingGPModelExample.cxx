/*
 * This file is part of the statismo library.
 *
 * Author: Marcel Luethi (marcel.luethi@unibas.ch)
 *
 * Copyright (c) 2011 University of Basel
 * All rights reserved.
 *
 * Statismo is licensed under the BSD licence (3 clause) license
 */

#include "statismo/core/Kernels.h"
#include "statismo/core/KernelCombinators.h"
#include "statismo/core/LowRankGPModelBuilder.h"
#include "statismo/core/StatisticalModel.h"
#include "statismo/core/IO.h"
#include "statismo/VTK/vtkStandardMeshRepresenter.h"

#include <vtkPolyDataReader.h>

#include <iostream>
#include <memory>

using namespace statismo;

namespace {

/*
 * We use a sum of gaussian kernels as our main model.
 */
class _MultiscaleGaussianKernel : public MatrixValuedKernel<vtkPoint>
{
public:
  _MultiscaleGaussianKernel(float baseWidth, float baseScale, unsigned nLevels)
    : MatrixValuedKernel<vtkPoint>(3)
    , m_baseWidth(baseWidth)
    , m_baseScale(baseScale)
    , m_nLevels(nLevels)
  {}

  inline MatrixType
  operator()(const vtkPoint & x, const vtkPoint & y) const override
  {
    VectorType r(3);
    r << x[0] - y[0], x[1] - y[1], x[2] - y[2];

    float minusRDotR = -r.dot(r);
    float kernelValue{0.0f};
    for (unsigned l = 1; l <= m_nLevels; ++l)
    {
      float scaleOnLevel = m_baseScale / static_cast<float>(l);
      float widthOnLevel = m_baseWidth / static_cast<float>(l);
      kernelValue += scaleOnLevel * std::exp(minusRDotR / (widthOnLevel * widthOnLevel));
    }
    return statismo::MatrixType::Identity(3, 3) * kernelValue;
  }

  std::string
  GetKernelInfo() const override
  {
    std::ostringstream os;
    os << "Multiscale GaussianKernel";
    return os.str();
  }

private:
  float    m_baseWidth;
  float    m_baseScale;
  unsigned m_nLevels;
};

vtkSmartPointer<vtkPolyData>
_LoadVTKPolyData(const std::string & filename)
{
  vtkNew<vtkPolyDataReader> reader;
  reader->SetFileName(filename.c_str());
  reader->Update();
  return reader->GetOutput();
}

// compute the center of mass for the given mesh
vtkPoint
_CenterOfMass(const vtkPolyData * _pd)
{
  // vtk is not const-correct, but we will not mutate pd here;
  auto pd = const_cast<vtkPolyData *>(_pd);

  vtkIdType numPoints = pd->GetNumberOfPoints();
  vtkPoint  massCenter(0.0, 0.0, 0.0);
  for (vtkIdType i = 0; i < numPoints; ++i)
  {
    double * ithPoint = pd->GetPoint(i);
    for (unsigned d = 0; d < 3; ++d)
    {
      massCenter[d] += ithPoint[d];
    }
  }
  double V = 1.0 / static_cast<double>(numPoints);
  for (unsigned d = 0; d < 3; ++d)
  {
    massCenter[d] *= V;
  }
  return massCenter;
}

// As an example of a tempering function, we use a function which is more smooth for points whose
// x-component is smaller than the x-component of the center of mass. To achieve a smooth transition between the areas,
// we use a sigmoid function. The variable a controls how fast the value of the tempering function changes from 0 to 1.
struct _MyTemperingFunction : public TemperingFunction<vtkPoint>
{
  explicit _MyTemperingFunction(const vtkPoint & massCenter)
    : m_centerOfMass{massCenter}
  {}

  static constexpr double a{0.5};
  
  double
  operator()(const vtkPoint & pt) const override
  {
    double xDiffToCenter = m_centerOfMass[0] - pt[0];
    return (1.0 / (1.0 + std::exp(-xDiffToCenter * a)) + 1.0);
  }

private:
  vtkPoint m_centerOfMass;
};
}

//
// Computes a multi-scale gaussian model and uses spatial tempering to make the smoothness spatially varying.
//
// For mathematical details of this procedure, refer to the paper:
// T. Gerig, K. Shahim, M. Reyes, T. Vetter, M. Luethi, Spatially-varying registration using Gaussian Processes.
//
int
main(int argc, char ** argv)
{
  if (argc < 7)
  {
    std::cerr << "Usage " << argv[0]
              << " referenceMesh baseKernelWidth baseScale numLevels numberOfComponents outputmodelName" << std::endl;
    return 1;
  }

  std::string refFilename(argv[1]);
  auto      baseKernelWidth = std::stof(argv[2]);
  auto      baseScale = std::stof(argv[3]);
  unsigned    numLevels = std::stoi(argv[4]);
  int         numberOfComponents = std::stoi(argv[5]);
  std::string outputModelFilename(argv[6]);


  // All the statismo classes have to be parameterized with the RepresenterType.

  using RepresenterType = vtkStandardMeshRepresenter         ;
  using ModelBuilderType = LowRankGPModelBuilder<vtkPolyData> ;

  try
  {

    auto                referenceMesh = _LoadVTKPolyData(refFilename);
    auto representer = vtkStandardMeshRepresenter::SafeCreate(referenceMesh);

    _MultiscaleGaussianKernel gk{baseKernelWidth, baseScale, numLevels};
    _MyTemperingFunction temperingFun{_CenterOfMass(referenceMesh)};

    SpatiallyVaryingKernel<RepresenterType::DatasetType> temperedKernel(
      representer.get(), gk, temperingFun, numberOfComponents, numberOfComponents * 2, true);

    // We create a new model using the combined kernel. The new model will be more flexible than the original
    // statistical model.
    auto modelBuilder = ModelBuilderType::SafeCreate(representer.get());
    auto newModel = modelBuilder->BuildNewModel(referenceMesh, temperedKernel, numberOfComponents);

    // Once we have built the model, we can save it to disk.
    statismo::IO<vtkPolyData>::SaveStatisticalModel(newModel.get(), outputModelFilename);
    std::cout << "Successfully saved shape model as " << outputModelFilename << std::endl;
  }
  catch (const StatisticalModelException & e)
  {
    std::cerr << "Exception occured while building the shape model" << std::endl;
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
