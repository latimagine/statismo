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

#ifndef __STATISMO_VTK_STANDARD_IMAGE_REPRESENTER_HXX_
#define __STATISMO_VTK_STANDARD_IMAGE_REPRESENTER_HXX_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/HDF5Utils.h"
#include "statismo/core/Utils.h"
#include "statismo/VTK/vtkStandardImageRepresenter.h"
#include "statismo/VTK/vtkHelper.h"

#include <vtkStructuredPointsReader.h>
#include <vtkStructuredPointsWriter.h>
#include <vtkPointData.h>
#include <vtkDataArray.h>
#include <vtkVersion.h>

namespace statismo
{

template <class TScalar, unsigned PIXEL_DIMENSIONS>
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::vtkStandardImageRepresenter(DatasetConstPointerType reference)
  : vtkStandardImageRepresenter()
{
  this->SetReference(reference);
}

template <class TScalar, unsigned PIXEL_DIMENSIONS>
void
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::AssertCompatibility(DatasetConstPointerType data) const
{
  if (sizeof(TScalar) != const_cast<vtkStructuredPoints *>(data)->GetScalarSize())
  {
    std::cout << "sizeof(TScalar) = " << sizeof(TScalar) << std::endl;
    std::cout << "sizeof(TScalar) = " << const_cast<vtkStructuredPoints *>(data)->GetScalarSize() << std::endl;
    throw StatisticalModelException("incompatible data (bad scalar type) given to representer",
                                    statismo::Status::INVALID_DATA_ERROR);
  }

  if (PIXEL_DIMENSIONS != const_cast<vtkStructuredPoints *>(data)->GetNumberOfScalarComponents())
  {
    throw StatisticalModelException("incompatible data (bad scalar components count) given to representer",
                                    statismo::Status::INVALID_DATA_ERROR);
  }
}

template <class TScalar, unsigned PIXEL_DIMENSIONS>
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS> *
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::CloneImpl() const
{
  // this works since Create deep copies the reference
  return vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::Create(m_reference);
}

template <class TScalar, unsigned PIXEL_DIMENSIONS>
void
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::Load(const H5::Group & fg)
{
  vtkSmartPointer<vtkStructuredPoints> ref;

  std::string repName = hdf5utils::ReadStringAttribute(fg, "name");
  if (repName == "vtkStructuredPointsRepresenter" || repName == "itkImageRepresenter" ||
      repName == "itkVectorImageRepresenter")
  {
    ref = LoadRefLegacy(fg);
  }
  else
  {
    ref = LoadRef(fg);
  }
  this->SetReference(ref);
}

template <class TScalar, unsigned PIXEL_DIMENSIONS>
vtkSmartPointer<vtkStructuredPoints>
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::LoadRef(const H5::Group & fg) const
{

  auto type = hdf5utils::ReadStringAttribute(fg, "datasetType");
  if (type != "IMAGE")
  {
    throw StatisticalModelException((std::string("Cannot load representer data: The ") +
                                     "representer specified in the given hdf5 file is of the wrong type: (" + type +
                                     ", expected IMAGE)")
                                      .c_str(),
                                    Status::INVALID_H5DATA_ERROR);
  }

  statismo::VectorType originVec;
  hdf5utils::ReadVector(fg, "origin", originVec);

  auto imageDimension = static_cast<unsigned>(hdf5utils::ReadInt(fg, "imageDimension"));
  if (imageDimension > 3)
  {
    throw StatisticalModelException("this representer does not support images of dimensionality > 3");
  }

  double origin[3] = { 0, 0, 0 };
  for (unsigned i = 0; i < imageDimension; i++)
  {
    origin[i] = originVec[i];
  }

  statismo::VectorType spacingVec;
  hdf5utils::ReadVector(fg, "spacing", spacingVec);
  double spacing[3] = { 0, 0, 0 };
  for (unsigned i = 0; i < imageDimension; i++)
  {
    spacing[i] = spacingVec[i];
  }

  typename statismo::GenericEigenTraits<int>::VectorType sizeVec;
  hdf5utils::ReadVectorOfType<int>(fg, "size", sizeVec);
  int size[3] = { 1, 1, 1 };
  for (unsigned i = 0; i < imageDimension; i++)
  {
    size[i] = sizeVec[i];
  }

  H5::Group pdGroup = fg.openGroup("./pointData");

  typename statismo::GenericEigenTraits<double>::MatrixType pixelMat;
  hdf5utils::ReadMatrixOfType<double>(pdGroup, "pixelValues", pixelMat);

  auto ds = pdGroup.openDataSet("pixelValues");
  auto datatype = static_cast<unsigned>(hdf5utils::ReadIntAttribute(ds, "datatype"));

  if (statismo::GetDataTypeId<TScalar>() != datatype)
  {
    std::cout << "Warning: The datatype specified for the scalars does not match the TPixel template argument used in "
                 "this representer."
              << std::endl;
  }

  auto newImage = vtkSmartPointer<vtkStructuredPoints>::New();
  newImage->SetDimensions(size[0], size[1], size[2]);
  newImage->SetExtent(0, size[0] - 1, 0, size[1] - 1, 0, size[2] - 1);
  newImage->SetOrigin(origin[0], origin[1], origin[2]);
  newImage->SetSpacing(spacing[0], spacing[1], spacing[2]);
  newImage->AllocateScalars(helper::vtkStatismoDataTypeIdToVtkDataTypeId(datatype), PIXEL_DIMENSIONS);

  for (int i = 0; i < size[2]; i++)
  {
    for (int j = 0; j < size[1]; j++)
    {
      for (int k = 0; k < size[0]; k++)
      {
        unsigned  index = size[1] * size[0] * i + size[0] * j + k;
        auto scalarPtr = static_cast<TScalar *>(newImage->GetScalarPointer(k, j, i));
        for (unsigned d = 0; d < PIXEL_DIMENSIONS; d++)
        {
          scalarPtr[d] = pixelMat(d, index);
        }
      }
    }
  }

  return newImage;
}


template <class TScalar, unsigned PIXEL_DIMENSIONS>
vtkSmartPointer<vtkStructuredPoints>
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::LoadRefLegacy(const H5::Group & fg) const
{
  auto tmpfilename = statismo::utils::CreateTmpName(".vtk");

  auto uw = statismo::MakeStackUnwinder([=]() { statismo::utils::RemoveFile(tmpfilename); });

  statismo::hdf5utils::GetFileFromHDF5(fg, "./reference", tmpfilename.c_str());

  vtkNew<vtkStructuredPointsReader> reader;
  reader->SetFileName(tmpfilename.c_str());
  reader->Update();

  if (reader->GetErrorCode() != 0)
  {
    throw StatisticalModelException((std::string("Could not read file ") + tmpfilename).c_str(), Status::IO_ERROR);
  }

  return reader->GetOutput();
}


template <class TScalar, unsigned PIXEL_DIMENSIONS>
statismo::VectorType
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::PointToVector(const PointType & pt) const
{
  // a vtk point is always 3 dimensional
  VectorType v(3);
  for (unsigned i = 0; i < 3; i++)
  {
    v(i) = pt[i];
  }
  return v;
}

template <class TScalar, unsigned PIXEL_DIMENSIONS>
VectorType
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::SampleToSampleVector(DatasetConstPointerType sample) const
{
  AssertCompatibility(sample);

  auto reference = const_cast<vtkStructuredPoints *>(this->m_reference.GetPointer());
  auto sp = const_cast<vtkStructuredPoints *>(sample);

  if (reference->GetNumberOfPoints() != sp->GetNumberOfPoints())
  {
    throw StatisticalModelException("The sample does not have the correct number of points");
  }

  VectorType sampleVector = VectorType::Zero(m_reference->GetNumberOfPoints() * PIXEL_DIMENSIONS);

  vtkDataArray * dataArray = sp->GetPointData()->GetArray(0);

  // \todo Make this more efficient using SetVoidArray of vtk
  // \warning This is only possible, if we enforce VectorType and
  // vtkStructuredPoints to have the same data type, e.g. float or int.
  for (unsigned i = 0; i < m_reference->GetNumberOfPoints(); i++)
  {
    double val[PIXEL_DIMENSIONS];
    dataArray->GetTuple(i, val);
    for (unsigned d = 0; d < PIXEL_DIMENSIONS; d++)
    {
      sampleVector(vtkStandardImageRepresenter::MapPointIdToInternalIdx(i, d)) = val[d];
    }
  }

  return sampleVector;
}

template <class TScalar, unsigned PIXEL_DIMENSIONS>
typename vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::DatasetPointerType
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::SampleVectorToSample(const VectorType & sampleVector) const
{
  auto                  sample = vtkSmartPointer<vtkStructuredPoints>::New();
  auto reference = const_cast<vtkStructuredPoints *>(m_reference.GetPointer());
  sample->DeepCopy(reference);

  vtkDataArray * dataArray = sample->GetPointData()->GetArray(0);
  for (unsigned i = 0; i < GetNumberOfPoints(); i++)
  {
    double val[PIXEL_DIMENSIONS];
    for (unsigned d = 0; d < PIXEL_DIMENSIONS; d++)
    {
      val[d] = sampleVector(vtkStandardImageRepresenter::MapPointIdToInternalIdx(i, d));
    }
    dataArray->SetTuple(i, val);
  }

  return sample;
}

template <class TScalar, unsigned PIXEL_DIMENSIONS>
typename vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::ValueType
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::PointSampleFromSample(DatasetConstPointerType sample,
                                                                             unsigned                ptid) const
{
  AssertCompatibility(sample);

  auto mutableSample = const_cast<vtkStructuredPoints *>(sample);
  if (ptid >= mutableSample->GetNumberOfPoints())
  {
    throw StatisticalModelException("invalid ptid provided to PointSampleFromSample");
  }

  double doubleVal[PIXEL_DIMENSIONS];
  mutableSample->GetPointData()->GetScalars()->GetTuple(ptid, doubleVal);
  ValueType val(PIXEL_DIMENSIONS);

  // vtk returns double. We need to convert it to whatever precision we are using in NDPixel
  for (unsigned i = 0; i < PIXEL_DIMENSIONS; i++)
  {
    val[i] = static_cast<TScalar>(doubleVal[i]);
  }
  return val;
}

template <class TScalar, unsigned PIXEL_DIMENSIONS>
statismo::VectorType
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::PointSampleToPointSampleVector(const ValueType & v) const
{
  VectorType vec(PIXEL_DIMENSIONS);
  for (unsigned i = 0; i < PIXEL_DIMENSIONS; i++)
  {
    vec(i) = v[i];
  }
  return vec;
}

template <class TScalar, unsigned PIXEL_DIMENSIONS>
typename vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::ValueType
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::PointSampleVectorToPointSample(
  const VectorType & pointSample) const
{
  ValueType value(PIXEL_DIMENSIONS);
  for (unsigned i = 0; i < PIXEL_DIMENSIONS; i++)
  {
    value[i] = pointSample[i];
  }

  return value;
}

template <class TScalar, unsigned PIXEL_DIMENSIONS>
void
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::Save(const H5::Group & fg) const
{
  using namespace H5;

  // get the effective image dimension, by check the size
  int *    size = m_reference->GetDimensions();
  unsigned imageDimension = 0;
  if (size[2] == 1 && size[1] == 1)
  {
    imageDimension = 1;
  }
  else if (size[2] == 1)
  {
    imageDimension = 2;
  }
  else
  {
    imageDimension = 3;
  }

  hdf5utils::WriteInt(fg, "imageDimension", imageDimension);

  double *             origin = m_reference->GetOrigin();
  statismo::VectorType originVec = statismo::VectorType::Zero(imageDimension);
  for (unsigned i = 0; i < imageDimension; i++)
  {
    originVec(i) = origin[i];
  }
  hdf5utils::WriteVector(fg, "origin", originVec);

  double *             spacing = m_reference->GetSpacing();
  statismo::VectorType spacingVec = statismo::VectorType::Zero(imageDimension);
  for (unsigned i = 0; i < imageDimension; i++)
  {
    spacingVec(i) = spacing[i];
  }
  hdf5utils::WriteVector(fg, "spacing", spacingVec);

  statismo::GenericEigenTraits<int>::VectorType sizeVec =
    statismo::GenericEigenTraits<int>::VectorType::Zero(imageDimension);
  for (unsigned i = 0; i < imageDimension; i++)
  {
    sizeVec(i) = size[i];
  }
  hdf5utils::WriteVectorOfType<int>(fg, "size", sizeVec);

  // VTK does not support image direction. We write the identity matrix
  statismo::MatrixType directionMat = statismo::MatrixType::Identity(imageDimension, imageDimension);
  hdf5utils::WriteMatrix(fg, "direction", directionMat);

  using DoubleMatrixType = statismo::GenericEigenTraits<double>::MatrixType;
  DoubleMatrixType pixelMat(vtkStandardImageRepresenter::GetDimensions(), GetNumberOfPoints());

  for (unsigned i = 0; i < static_cast<unsigned>(size[2]); i++)
  {
    for (unsigned j = 0; j < static_cast<unsigned>(size[1]); j++)
    {
      for (unsigned k = 0; k < static_cast<unsigned>(size[0]); k++)
      {
        unsigned  ind = i * size[1] * size[0] + j * size[0] + k;
        auto pixel = static_cast<TScalar *>(m_reference->GetScalarPointer(k, j, i));
        for (unsigned d = 0; d < vtkStandardImageRepresenter::GetDimensions(); d++)
        {
          pixelMat(d, ind) = pixel[d];
        }
      }
    }
  }

  auto pdGroup = fg.createGroup("pointData");
  statismo::hdf5utils::WriteInt(pdGroup, "pixelDimension", vtkStandardImageRepresenter::GetDimensions());

  auto ds = hdf5utils::WriteMatrixOfType<double>(pdGroup, "pixelValues", pixelMat);
  hdf5utils::WriteIntAttribute(ds, "datatype", statismo::GetDataTypeId<TScalar>());
}

template <class TScalar, unsigned PIXEL_DIMENSIONS>
unsigned
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::GetNumberOfPoints() const
{
  return this->GetNumberOfPoints(this->m_reference);
}

template <class TScalar, unsigned PIXEL_DIMENSIONS>
void
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::SetReference(DatasetConstPointerType reference)
{
  AssertCompatibility(reference);

  m_reference->DeepCopy(const_cast<vtkStructuredPoints *>(reference));
  // set the domain
  typename DomainType::DomainPointsListType ptList;
  for (unsigned i = 0; i < m_reference->GetNumberOfPoints(); i++)
  {
    ptList.emplace_back(m_reference->GetPoint(i));
  }
  m_domain = DomainType(ptList);
}

template <class TScalar, unsigned PIXEL_DIMENSIONS>
unsigned
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::GetPointIdForPoint(const PointType & pt) const
{
  assert(m_reference);
  return this->m_reference->FindPoint(const_cast<double *>(pt.Data()));
}

template <class TScalar, unsigned PIXEL_DIMENSIONS>
unsigned
vtkStandardImageRepresenter<TScalar, PIXEL_DIMENSIONS>::GetNumberOfPoints(DatasetPointerType reference)
{
  assert(reference);
  return reference->GetNumberOfPoints();
}

} // namespace statismo

#endif
