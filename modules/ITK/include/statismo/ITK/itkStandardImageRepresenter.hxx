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

#ifndef __STATIMO_ITK_STANDARD_IMAGE_REPRESENTER_HXX_
#define __STATIMO_ITK_STANDARD_IMAGE_REPRESENTER_HXX_

#include "statismo/core/HDF5Utils.h"
#include "statismo/core/Utils.h"
#include "statismo/ITK/itkStandardImageRepresenter.h"

#include <itkImageDuplicator.h>
#include <itkImageIterator.h>
#include <itkImageRegionConstIterator.h>
#include <itkImageRegionIterator.h>
#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>
#include <itkIndex.h>
#include <itkPoint.h>
#include <itkVector.h>

namespace itk
{

template <typename Pixel, unsigned IMAGE_DIMENSION>
StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::StandardImageRepresenter()
  : m_reference{ nullptr }
{}
template <typename Pixel, unsigned IMAGE_DIMENSION>
StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::~StandardImageRepresenter() = default;

template <typename Pixel, unsigned IMAGE_DIMENSION>
StandardImageRepresenter<Pixel, IMAGE_DIMENSION> *
StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::CloneImpl() const
{
  auto               clone = new StandardImageRepresenter();
  DatasetPointerType clonedReference = this->CloneDataset(m_reference);
  clone->SetReference(clonedReference);
  clone->SetLogger(this->GetLogger());
  return clone;
}

template <typename Pixel, unsigned IMAGE_DIMENSION>
void
StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::Load(const H5::Group & fg)
{
  auto repName = statismo::HDF5Utils::ReadStringAttribute(fg, "name");
  if (repName == "vtkStructuredPointsRepresenter" || repName == "itkImageRepresenter" ||
      repName == "itkVectorImageRepresenter")
  {
    this->SetReference(LoadRefLegacy(fg));
  }
  else
  {
    this->SetReference(LoadRef(fg));
  }
}


template <typename Pixel, unsigned IMAGE_DIMENSION>
typename StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::ImageType::Pointer
StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::LoadRef(const H5::Group & fg) const
{
  auto readImageDimension = statismo::HDF5Utils::ReadInt(fg, "imageDimension");
  if (readImageDimension != IMAGE_DIMENSION)
  {
    throw statismo::StatisticalModelException(
      "the image dimension specified in the statismo file does not match the one specified as template parameter",
      statismo::Status::INVALID_H5DATA_ERROR);
  }

  // Read
  statismo::VectorType originVec;
  statismo::HDF5Utils::ReadVector(fg, "origin", originVec);

  statismo::VectorType spacingVec;
  statismo::HDF5Utils::ReadVector(fg, "spacing", spacingVec);

  typename statismo::GenericEigenTraits<int>::VectorType sizeVec;
  statismo::HDF5Utils::ReadVectorOfType<int>(fg, "size", sizeVec);

  typename ImageType::PointType   origin;
  typename ImageType::SpacingType spacing;
  typename ImageType::SizeType    size;
  for (unsigned i = 0; i < IMAGE_DIMENSION; ++i)
  {
    origin[i] = originVec[i];
    spacing[i] = spacingVec[i];
    size[i] = sizeVec[i];
  }

  statismo::MatrixType directionMat;
  statismo::HDF5Utils::ReadMatrix(fg, "direction", directionMat);
  typename ImageType::DirectionType direction;
  for (unsigned i = 0; i < directionMat.rows(); ++i)
  {
    for (unsigned j = 0; j < directionMat.rows(); ++j)
    {
      direction[i][j] = directionMat(i, j);
    }
  }

  H5::Group pdGroup = fg.openGroup("./pointData");
  auto      readPixelDimension = static_cast<unsigned>(statismo::HDF5Utils::ReadInt(pdGroup, "pixelDimension"));
  if (readPixelDimension != StandardImageRepresenter::GetDimensions())
  {
    throw statismo::StatisticalModelException(
      "the pixel dimension specified in the statismo file does not match the one specified as template parameter",
      statismo::Status::INVALID_H5DATA_ERROR);
  }

  typename statismo::GenericEigenTraits<double>::MatrixType pixelMatDouble;
  statismo::HDF5Utils::ReadMatrixOfType<double>(pdGroup, "pixelValues", pixelMatDouble);
  statismo::MatrixType          pixelMat = pixelMatDouble.cast<statismo::ScalarType>();
  typename ImageType::Pointer   newImage = ImageType::New();
  typename ImageType::IndexType start;
  start.Fill(0);

  auto ds = pdGroup.openDataSet("pixelValues");
  auto type = static_cast<unsigned>(statismo::HDF5Utils::ReadIntAttribute(ds, "datatype"));
  if (type != PixelConversionTrait<Pixel>::GetDataType())
  {
    STATISMO_LOG_WARNING(
      "The datatype specified for the scalars does not match the Pixel template argument used in this representer");
  }

  typename ImageType::RegionType region(start, size);
  newImage->SetRegions(region);
  newImage->Allocate();
  newImage->SetOrigin(origin);
  newImage->SetSpacing(spacing);
  newImage->SetDirection(direction);

  itk::ImageRegionIterator<DatasetType> it(newImage, newImage->GetLargestPossibleRegion());
  it.GoToBegin();
  for (unsigned i = 0; !(it.IsAtEnd()); ++it, ++i)
  {
    Pixel v = PixelConversionTrait<Pixel>::FromVector(pixelMat.col(i));
    it.Set(v);
  }

  return newImage;
}

template <typename Pixel, unsigned IMAGE_DIMENSION>
typename StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::ImageType::Pointer
StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::LoadRefLegacy(const H5::Group & fg) const
{
  auto tmpfilename = statismo::utils::CreateTmpName(".vtk");
  statismo::HDF5Utils::GetFileFromHDF5(fg, "./reference", tmpfilename.c_str());

  auto uw = statismo::MakeStackUnwinder([=]() { statismo::utils::RemoveFile(tmpfilename); });

  auto reader = itk::ImageFileReader<ImageType>::New();
  reader->SetFileName(tmpfilename);
  try
  {
    reader->Update();
  }
  catch (const itk::ImageFileReaderException & e)
  {
    throw statismo::StatisticalModelException((std::string("Could not read file ") + tmpfilename).c_str(),
                                              statismo::Status::IO_ERROR);
  }
  typename DatasetType::Pointer img = reader->GetOutput();
  img->Register();
  return img;
}

template <typename Pixel, unsigned IMAGE_DIMENSION>
void
StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::SetReference(ImageType * reference)
{
  m_reference = reference;

  typename DomainType::DomainPointsListType  domainPoints;
  itk::ImageRegionConstIterator<DatasetType> it(reference, reference->GetLargestPossibleRegion());
  for (it.GoToBegin(); !(it.IsAtEnd()); ++it)
  {
    PointType pt;
    reference->TransformIndexToPhysicalPoint(it.GetIndex(), pt);
    domainPoints.push_back(pt);
  }
  m_domain = DomainType(domainPoints);
}

template <typename Pixel, unsigned IMAGE_DIMENSION>
statismo::VectorType
StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::PointToVector(const PointType & pt) const
{
  statismo::VectorType v(PointType::GetPointDimension());
  for (unsigned i = 0; i < PointType::GetPointDimension(); i++)
  {
    v(i) = pt[i];
  }
  return v;
}

template <typename Pixel, unsigned IMAGE_DIMENSION>
statismo::VectorType
StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::SampleToSampleVector(DatasetConstPointerType sample) const
{
  statismo::VectorType sampleVec(this->GetNumberOfPoints() * StandardImageRepresenter::GetDimensions());
  itk::ImageRegionConstIterator<DatasetType> it(sample, sample->GetLargestPossibleRegion());

  it.GoToBegin();
  for (unsigned i = 0; !(it.IsAtEnd()); ++i, ++it)
  {

    statismo::VectorType sampleAtPt = PixelConversionTrait<Pixel>::ToVector(it.Value());
    for (unsigned j = 0; j < StandardImageRepresenter::GetDimensions(); j++)
    {
      unsigned idx = this->MapPointIdToInternalIdx(i, j);
      sampleVec[idx] = sampleAtPt[j];
    }
  }
  return sampleVec;
}

template <typename Pixel, unsigned IMAGE_DIMENSION>
typename StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::DatasetPointerType
StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::SampleVectorToSample(const statismo::VectorType & sample) const
{
  using DuplicatorType = itk::ImageDuplicator<DatasetType>;
  auto duplicator = DuplicatorType::New();
  duplicator->SetInputImage(this->m_reference);
  duplicator->Update();
  DatasetPointerType clonedImage = duplicator->GetOutput();

  itk::ImageRegionIterator<DatasetType> it(clonedImage, clonedImage->GetLargestPossibleRegion());
  it.GoToBegin();
  for (unsigned i = 0; !(it.IsAtEnd()); ++it, i++)
  {

    statismo::VectorType valAtPoint(StandardImageRepresenter::GetDimensions());
    for (unsigned d = 0; d < StandardImageRepresenter::GetDimensions(); d++)
    {
      unsigned idx = this->MapPointIdToInternalIdx(i, d);
      valAtPoint[d] = sample[idx];
    }
    ValueType v = PixelConversionTrait<Pixel>::FromVector(valAtPoint);
    it.Set(v);
  }
  return clonedImage;
}

template <typename Pixel, unsigned IMAGE_DIMENSION>
typename StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::ValueType
StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::PointSampleFromSample(DatasetConstPointerType sample,
                                                                        unsigned                ptid) const
{
  if (ptid >= GetDomain().GetNumberOfPoints())
  {
    throw statismo::StatisticalModelException("invalid ptid provided to PointSampleFromSample");
  }

  // we get the point with the id from the domain, as itk does not allow us get a point via its index.
  PointType                     pt = GetDomain().GetDomainPoints()[ptid];
  typename ImageType::IndexType idx;
  sample->TransformPhysicalPointToIndex(pt, idx);
  ValueType value = sample->GetPixel(idx);
  return value;
}

template <typename Pixel, unsigned IMAGE_DIMENSION>
typename StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::ValueType
StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::PointSampleVectorToPointSample(
  const statismo::VectorType & pointSample) const
{
  return PixelConversionTrait<Pixel>::FromVector(pointSample);
}

template <typename Pixel, unsigned IMAGE_DIMENSION>
statismo::VectorType
StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::PointSampleToPointSampleVector(const ValueType & v) const
{
  return PixelConversionTrait<Pixel>::ToVector(v);
}

template <typename Pixel, unsigned IMAGE_DIMENSION>
void
StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::Save(const H5::Group & fg) const
{
  auto                 origin = m_reference->GetOrigin();
  statismo::VectorType originVec(IMAGE_DIMENSION);
  for (unsigned i = 0; i < IMAGE_DIMENSION; i++)
  {
    originVec(i) = origin[i];
  }
  statismo::HDF5Utils::WriteVector(fg, "origin", originVec);

  auto                 spacing = m_reference->GetSpacing();
  statismo::VectorType spacingVec(IMAGE_DIMENSION);
  for (unsigned i = 0; i < IMAGE_DIMENSION; i++)
  {
    spacingVec(i) = spacing[i];
  }
  statismo::HDF5Utils::WriteVector(fg, "spacing", spacingVec);
  statismo::GenericEigenTraits<int>::VectorType sizeVec(IMAGE_DIMENSION);
  for (unsigned i = 0; i < IMAGE_DIMENSION; i++)
  {
    sizeVec(i) = m_reference->GetLargestPossibleRegion().GetSize()[i];
  }
  statismo::HDF5Utils::WriteVectorOfType<int>(fg, "size", sizeVec);

  auto                 direction = m_reference->GetDirection();
  statismo::MatrixType directionMat(IMAGE_DIMENSION, IMAGE_DIMENSION);
  for (unsigned i = 0; i < IMAGE_DIMENSION; i++)
  {
    for (unsigned j = 0; j < IMAGE_DIMENSION; j++)
    {
      directionMat(i, j) = direction[i][j];
    }
  }
  statismo::HDF5Utils::WriteMatrix(fg, "direction", directionMat);
  statismo::HDF5Utils::WriteInt(fg, "imageDimension", IMAGE_DIMENSION);

  H5::Group pdGroup = fg.createGroup("pointData");
  statismo::HDF5Utils::WriteInt(pdGroup, "pixelDimension", StandardImageRepresenter::GetDimensions());

  using DoubleMatrixType = statismo::GenericEigenTraits<double>::MatrixType;
  statismo::MatrixType pixelMat(StandardImageRepresenter::GetDimensions(), GetNumberOfPoints());

  itk::ImageRegionIterator<DatasetType> it(m_reference, m_reference->GetLargestPossibleRegion());
  it.GoToBegin();
  for (unsigned i = 0; !(it.IsAtEnd()); ++i, ++it)
  {
    pixelMat.col(i) = PixelConversionTrait<Pixel>::ToVector(it.Get());
  }
  DoubleMatrixType pixelMatDouble = pixelMat.cast<double>();
  H5::DataSet      ds = statismo::HDF5Utils::WriteMatrixOfType<double>(pdGroup, "pixelValues", pixelMatDouble);
  statismo::HDF5Utils::WriteIntAttribute(ds, "datatype", PixelConversionTrait<Pixel>::GetDataType());
}

template <typename Pixel, unsigned IMAGE_DIMENSION>
unsigned
StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::GetNumberOfPoints() const
{
  return m_reference->GetLargestPossibleRegion().GetNumberOfPixels();
}

template <typename Pixel, unsigned IMAGE_DIMENSION>
unsigned
StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::GetPointIdForPoint(const PointType & pt) const
{
  // itks organization is slice row col
  typename DatasetType::IndexType idx;
  bool                            ptInImage = this->m_reference->TransformPhysicalPointToIndex(pt, idx);

  auto size = this->m_reference->GetLargestPossibleRegion().GetSize();

  // It does not make sense to allow points outside the image, because only the inside is modeled.
  // However, some discretization artifacts of image and surface operations may produce points that
  // are just on the boundary of the image, but mathematically outside. We accept these points and
  // return the iD of the closest image point.
  //
  // Any points further out will trigger an exception.
  //
  if (!ptInImage)
  {
    for (unsigned int i = 0; i < ImageType::ImageDimension; ++i)
    {
      // As soon as one coordinate is further away than one pixel, we throw an exception.
      if (idx[i] < -1 || idx[i] > static_cast<int>(size[i]))
      {
        throw statismo::StatisticalModelException(
          "GetPointIdForPoint computed invalid ptId. Make sure that the point is within the reference you chose ");
      }
      // If it is on the boundary, we set it to the nearest boundary coordinate.
      if (idx[i] == -1)
      {
        idx[i] = 0;
      }
      if (idx[i] == static_cast<int>(size[i]))
      {
        idx[i] = size[i] - 1;
      }
    }
  }


  // in itk, idx 0 is by convention the fastest moving index
  unsigned int index = 0;
  for (unsigned int i = 0; i < ImageType::ImageDimension; ++i)
  {
    unsigned int multiplier = 1;
    for (int d = i - 1; d >= 0; --d)
    {
      multiplier *= size[d];
    }
    index += multiplier * idx[i];
  }

  return index;
}

template <typename Pixel, unsigned IMAGE_DIMENSION>
typename StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::DatasetPointerType
StandardImageRepresenter<Pixel, IMAGE_DIMENSION>::CloneDataset(DatasetConstPointerType d) const
{
  using DuplicatorType = itk::ImageDuplicator<DatasetType>;
  auto duplicator = DuplicatorType::New();
  duplicator->SetInputImage(d);
  duplicator->Update();
  DatasetPointerType clone = duplicator->GetOutput();
  clone->DisconnectPipeline();
  return clone;
}

} // namespace itk

#endif
