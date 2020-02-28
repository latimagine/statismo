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

#ifndef __STATIMO_CORE_HDF5_UTILS_HXX_
#define __STATIMO_CORE_HDF5_UTILS_HXX_

#include "statismo/core/HDF5Utils.h"
#include "statismo/core/CommonTypes.h"
#include "statismo/core/Exceptions.h"

#include <H5Cpp.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

namespace statismo
{
namespace details
{
template <typename Scalar>
struct HDF5PredTypeTraits;

template <>
struct HDF5PredTypeTraits<double>
{
  static const H5::PredType &
  GetPredRef()
  {
    return H5::PredType::NATIVE_DOUBLE;
  }
};

template <>
struct HDF5PredTypeTraits<float>
{
  static const H5::PredType &
  GetPredRef()
  {
    return H5::PredType::NATIVE_FLOAT;
  }
};

template <>
struct HDF5PredTypeTraits<unsigned int>
{
  static const H5::PredType &
  GetPredRef()
  {
    return H5::PredType::NATIVE_UINT;
  }
};

template <>
struct HDF5PredTypeTraits<int>
{
  static const H5::PredType &
  GetPredRef()
  {
    return H5::PredType::NATIVE_INT;
  }
};
} // namespace details

template <class T>
inline void
HDF5Utils::ReadMatrixOfType(const H5::H5Location &                       fg,
                          const char *                                 name,
                          typename GenericEigenTraits<T>::MatrixType & matrix)
{
  H5::DataSet ds = fg.openDataSet(name);
  hsize_t     dims[2];
  ds.getSpace().getSimpleExtentDims(dims, nullptr);

  // simply read the whole dataspace
  matrix.resize(dims[0], dims[1]);
  ds.read(matrix.data(), details::HDF5PredTypeTraits<T>::GetPredRef());
}

template <class T>
inline H5::DataSet
HDF5Utils::WriteMatrixOfType(const H5::H5Location &                             fg,
                  const char *                                       name,
                  const typename GenericEigenTraits<T>::MatrixType & matrix)
{
  // HDF5 does not like empty matrices.
  //
  if (matrix.rows() == 0 || matrix.cols() == 0)
  {
    throw StatisticalModelException("Empty matrix provided to writeMatrix", Status::INVALID_DATA_ERROR);
  }

  hsize_t     dims[2] = { static_cast<hsize_t>(matrix.rows()), static_cast<hsize_t>(matrix.cols()) };
  H5::DataSet ds = fg.createDataSet(name, details::HDF5PredTypeTraits<T>::GetPredRef(), H5::DataSpace(2, dims));
  ds.write(matrix.data(), details::HDF5PredTypeTraits<T>::GetPredRef());
  return ds;
}

template <class T>
inline void
HDF5Utils::ReadVectorOfType(const H5::H5Location &                       fg,
                            const char *                                 name,
                            typename GenericEigenTraits<T>::VectorType & vector)
{
  H5::DataSet ds = fg.openDataSet(name);
  hsize_t     dims[1];
  ds.getSpace().getSimpleExtentDims(dims, nullptr);
  vector.resize(dims[0], 1);
  ds.read(vector.data(), details::HDF5PredTypeTraits<T>::GetPredRef());
}

template <class T>
inline H5::DataSet
HDF5Utils::WriteVectorOfType(const H5::H5Location &                             fg,
                  const char *                                       name,
                  const typename GenericEigenTraits<T>::VectorType & vector)
{
  hsize_t     dims[1] = { static_cast<hsize_t>(vector.size()) };
  H5::DataSet ds = fg.createDataSet(name, details::HDF5PredTypeTraits<T>::GetPredRef(), H5::DataSpace(1, dims));
  ds.write(vector.data(), details::HDF5PredTypeTraits<T>::GetPredRef());
  return ds;
}

template <typename T>
inline void
HDF5Utils::ReadArray(const H5::H5Location & fg, const char * name, std::vector<T> & array)
{
  H5::DataSet ds = fg.openDataSet(name);
  hsize_t     dims[1];
  ds.getSpace().getSimpleExtentDims(dims, nullptr);
  array.resize(dims[0]);
  ds.read(&array[0], details::HDF5PredTypeTraits<T>::GetPredRef());
}

template <typename T>
inline H5::DataSet
HDF5Utils::WriteArray(const H5::H5Location & fg, const char * name, std::vector<T> const & array)
{
  hsize_t     dims[1] = { array.size() };
  H5::DataSet ds = fg.createDataSet(name, details::HDF5PredTypeTraits<T>::GetPredRef(), H5::DataSpace(1, dims));
  ds.write(&array[0], details::HDF5PredTypeTraits<T>::GetPredRef());
  return ds;
}

inline bool
HDF5Utils::ExistsObjectWithName(const H5::H5Location & fg, const std::string & name)
{
  for (hsize_t i = 0; i < fg.getNumObjs(); ++i)
  {
    if (fg.getObjnameByIdx(i) == name)
    {
      return true;
    }
  }
  return false;
}

inline H5::H5File
HDF5Utils::OpenOrCreateFile(const std::string & filename)
{
  // check if file exists
  std::ifstream ifile(filename.c_str());
  H5::H5File    file;

  if (!ifile)
  {
    // create it
    file = H5::H5File(filename.c_str(), H5F_ACC_EXCL);
  }
  else
  {
    // open it
    file = H5::H5File(filename.c_str(), H5F_ACC_RDWR);
  }

  return file;
}

inline H5::Group
HDF5Utils::OpenPath(H5::H5File & file, const std::string & path, bool createPath)
{
  H5::Group group;

  // take the first part of the path
  std::size_t curpos = 1;
  std::size_t nextpos = path.find_first_of('/', curpos);
  auto        g = file.openGroup("/");
  auto        name = path.substr(curpos, nextpos - 1);

  while (curpos != std::string::npos && !name.empty())
  {
    if (ExistsObjectWithName(g, name))
    {
      g = g.openGroup(name);
    }
    else
    {
      if (createPath)
      {
        g = g.createGroup(name);
      }
      else
      {
        std::string msg = std::string("the path ") + path + " does not exist";
        throw StatisticalModelException(msg.c_str(), Status::IO_ERROR);
      }
    }

    curpos = nextpos + 1;
    nextpos = path.find_first_of('/', curpos);
    if (nextpos != std::string::npos)
    {
      name = path.substr(curpos, nextpos - curpos);
    }
    else
    {
      name = path.substr(curpos);
    }
  }

  return g;
}

inline void
HDF5Utils::ReadMatrix(const H5::H5Location & fg, const char * name, MatrixType & matrix)
{
  ReadMatrixOfType<ScalarType>(fg, name, matrix);
}

inline void
HDF5Utils::ReadMatrix(const H5::H5Location & fg, const char * name, unsigned maxNumColumns, MatrixType & matrix)
{
  auto    ds = fg.openDataSet(name);
  hsize_t dims[2];
  ds.getSpace().getSimpleExtentDims(dims, nullptr);

  auto nRows = dims[0]; // take the number of rows defined in the hdf5 file
  auto nCols = std::min(dims[1], static_cast<hsize_t>(maxNumColumns)); // take the number of cols provided by the user

  hsize_t offset[2] = { 0, 0 }; // hyperslab offset in the file
  hsize_t count[2];
  count[0] = nRows;
  count[1] = nCols;

  H5::DataSpace dataspace = ds.getSpace();
  dataspace.selectHyperslab(H5S_SELECT_SET, count, offset);

  /* Define the memory dataspace. */
  hsize_t dimsm[2];
  dimsm[0] = nRows;
  dimsm[1] = nCols;
  H5::DataSpace memspace(2, dimsm);

  /* Define memory hyperslab. */
  hsize_t offsetOut[2] = { 0, 0 }; // hyperslab offset in memory
  hsize_t countOut[2];             // size of the hyperslab in memory

  countOut[0] = nRows;
  countOut[1] = nCols;
  memspace.selectHyperslab(H5S_SELECT_SET, countOut, offsetOut);

  matrix.resize(nRows, nCols);
  ds.read(matrix.data(), H5::PredType::NATIVE_FLOAT, memspace, dataspace);
}

inline H5::DataSet
HDF5Utils::WriteMatrix(const H5::H5Location & fg, const char * name, const MatrixType & matrix)
{
  return WriteMatrixOfType<ScalarType>(fg, name, matrix);
}

inline void
HDF5Utils::ReadVector(const H5::H5Location & fg, const char * name, VectorType & vector)
{
  ReadVectorOfType<ScalarType>(fg, name, vector);
}

inline void
HDF5Utils::ReadVector(const H5::H5Location & fg, const char * name, unsigned maxNumElements, VectorType & vector)
{
  H5::DataSet ds = fg.openDataSet(name);
  hsize_t     dims[1];
  ds.getSpace().getSimpleExtentDims(dims, nullptr);

  hsize_t nElements =
    std::min(dims[0], static_cast<hsize_t>(maxNumElements)); // take the number of rows defined in the hdf5 file

  hsize_t offset[1] = { 0 }; // hyperslab offset in the file
  hsize_t count[1];
  count[0] = nElements;

  H5::DataSpace dataspace = ds.getSpace();
  dataspace.selectHyperslab(H5S_SELECT_SET, count, offset);

  /* Define the memory dataspace. */
  hsize_t dimsm[1];
  dimsm[0] = nElements;
  H5::DataSpace memspace(1, dimsm);

  /* Define memory hyperslab. */
  hsize_t offsetOut[1] = { 0 }; // hyperslab offset in memory
  hsize_t countOut[1];          // size of the hyperslab in memory

  countOut[0] = nElements;
  memspace.selectHyperslab(H5S_SELECT_SET, countOut, offsetOut);

  vector.resize(nElements);
  ds.read(vector.data(), H5::PredType::NATIVE_FLOAT, memspace, dataspace);
}

inline H5::DataSet
HDF5Utils::WriteVector(const H5::H5Location & fg, const char * name, const VectorType & vector)
{
  return WriteVectorOfType<ScalarType>(fg, name, vector);
}

inline H5::DataSet
HDF5Utils::WriteString(const H5::H5Location & fg, const char * name, const std::string & s)
{
  H5::StrType flsType(H5::PredType::C_S1, s.length() + 1); // + 1 for trailing zero
  H5::DataSet ds = fg.createDataSet(name, flsType, H5::DataSpace(H5S_SCALAR));
  ds.write(s, flsType);
  return ds;
}

inline std::string
HDF5Utils::ReadString(const H5::H5Location & fg, const char * name)
{
  H5std_string outputString;
  H5::DataSet  ds = fg.openDataSet(name);
  ds.read(outputString, ds.getStrType());
  return outputString;
}

inline void
HDF5Utils::WriteStringAttribute(const H5::H5Object & fg, const char * name, const std::string & s)
{
  H5::StrType   strdatatype(H5::PredType::C_S1, s.length() + 1); // + 1 for trailing 0
  H5::Attribute att = fg.createAttribute(name, strdatatype, H5::DataSpace(H5S_SCALAR));
  att.write(strdatatype, s);
}

inline std::string
HDF5Utils::ReadStringAttribute(const H5::H5Object & fg, const char * name)
{
  H5std_string outputString;

  H5::Attribute attOut = fg.openAttribute(name);
  attOut.read(attOut.getStrType(), outputString);
  return outputString;
}

inline void
HDF5Utils::WriteIntAttribute(const H5::H5Object & fg, const char * name, int value)
{
  H5::IntType   intType(H5::PredType::NATIVE_INT32);
  H5::DataSpace attSpace(H5S_SCALAR);
  H5::Attribute att = fg.createAttribute(name, intType, attSpace);
  att.write(intType, &value);
}

inline int
HDF5Utils::ReadIntAttribute(const H5::H5Object & fg, const char * name)
{
  H5::IntType   flsType(H5::PredType::NATIVE_INT32);
  int           value = 0;
  H5::Attribute attOut = fg.openAttribute(name);
  attOut.read(flsType, &value);
  return value;
}

inline H5::DataSet
HDF5Utils::WriteInt(const H5::H5Location & fg, const char * name, int value)
{
  H5::IntType flsType(H5::PredType::NATIVE_INT32); // 0 is a dummy argument
  H5::DataSet ds = fg.createDataSet(name, flsType, H5::DataSpace(H5S_SCALAR));
  ds.write(&value, flsType);
  return ds;
}

inline int
HDF5Utils::ReadInt(const H5::H5Location & fg, const char * name)
{
  H5::IntType flsType(H5::PredType::NATIVE_INT32);
  H5::DataSet ds = fg.openDataSet(name);

  int value = 0;
  ds.read(&value, flsType);
  return value;
}

inline H5::DataSet
HDF5Utils::WriteFloat(const H5::H5Location & fg, const char * name, float value)
{
  H5::FloatType flsType(H5::PredType::NATIVE_FLOAT); // 0 is a dummy argument
  H5::DataSet   ds = fg.createDataSet(name, flsType, H5::DataSpace(H5S_SCALAR));
  ds.write(&value, flsType);
  return ds;
}

inline float
HDF5Utils::ReadFloat(const H5::H5Location & fg, const char * name)
{
  H5::FloatType flsType(H5::PredType::NATIVE_FLOAT);
  H5::DataSet   ds = fg.openDataSet(name);

  float value = 0;
  ds.read(&value, flsType);
  return value;
}

inline void
HDF5Utils::DumpFileToHDF5(const char * filename, const H5::H5Location & fg, const char * name)
{
  std::ifstream ifile(filename, std::ios::binary);
  if (!ifile)
  {
    std::string s = std::string("could not open file ") + filename;
    throw StatisticalModelException(s.c_str(), Status::IO_ERROR);
  }

  std::vector<char> buffer;
  ifile >> std::noskipws;
  std::copy(std::istream_iterator<char>(ifile), std::istream_iterator<char>(), std::back_inserter(buffer));

  hsize_t     dims[] = { buffer.size() };
  H5::DataSet ds = fg.createDataSet(name, H5::PredType::NATIVE_CHAR, H5::DataSpace(1, dims));
  ds.write(&buffer[0], H5::PredType::NATIVE_CHAR);
}

inline void
HDF5Utils::GetFileFromHDF5(const H5::H5Location & fg, const char * name, const char * filename)
{
  H5::DataSet ds = fg.openDataSet(name);
  hsize_t     dims[1];
  ds.getSpace().getSimpleExtentDims(dims, nullptr);
  std::vector<char> buffer(dims[0]);
  if (!buffer.empty())
  {
    ds.read(&buffer[0], H5::PredType::NATIVE_CHAR);
  }

  std::ofstream ofile(filename, std::ios::binary);
  if (!ofile)
  {
    std::string s = std::string("could not open file ") + filename;
    throw StatisticalModelException(s.c_str(), Status::IO_ERROR);
  }

  std::copy(std::begin(buffer), std::end(buffer), std::ostream_iterator<char>(ofile));
}

}

#endif
