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

#include "statismo/core/StatisticalModel.h"
#include "statismo/core/IO.h"
#include "statismo/VTK/vtkStandardMeshRepresenter.h"

#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataWriter.h>
#include <vtkVersion.h>

#include <iostream>
#include <memory>

using namespace statismo;

namespace {

void
_SaveSample (const vtkPolyData * pd, const std::string & resdir, const std::string & basename)
{
  auto filename = resdir + std::string("/") + basename;

  vtkNew<vtkPolyDataWriter> w;
  w->SetInputData(const_cast<vtkPolyData *>(pd));
  w->SetFileName(filename.c_str());
  w->Update();
}

}

//
// Illustrates how to load a shape model and the basic sampling functinality
//
int
main(int argc, char ** argv)
{
  if (argc < 3)
  {
    std::cerr << "Usage " << argv[0] << " modelname resultdir" << std::endl;
    return 1;
  }

  std::string modelname(argv[1]);
  std::string resultdir(argv[2]);


  // All the statismo classes have to be parameterized with the RepresenterType.
  // For building a shape model with vtk, we use the vtkPolyDataRepresenter.
  using RepresenterType=  vtkStandardMeshRepresenter;
  using StatisticalModelType =  StatisticalModel<vtkPolyData>;

  try
  {
    // To load a model, we call the static Load method, which returns (a pointer to) a
    // new StatisticalModel object
    auto                   representer = RepresenterType::SafeCreate();
    auto model = statismo::IO<vtkPolyData>::LoadStatisticalModel(representer.get(), modelname);
    
    std::cout << "Loaded model with " << model->GetNumberOfPrincipalComponents() << " Principal Components"
              << std::endl;

    // get the model mean
    auto mean = model->DrawMean();
    _SaveSample (mean, resultdir, "mean.vtk");

    // draw a random sample
    auto randomSample = model->DrawSample();
    _SaveSample (randomSample, resultdir, "randomsample.vtk");

    // draw a sample with known pca coefficients (3 stddev in direction of the 1st PC)
    VectorType coefficients = VectorType::Zero(model->GetNumberOfPrincipalComponents());
    coefficients(0) = 3;
    auto samplePC1 = model->DrawSample(coefficients);
    _SaveSample(samplePC1, resultdir, "samplePC1.vtk");

    std::cout << "saved samples to " << resultdir << std::endl;
  }
  catch (const StatisticalModelException & e)
  {
    std::cerr << "Exception occured while building the shape model" << std::endl;
    std::cerr << e.what() << std::endl;
    return 1;
  }

  return 0;
}
