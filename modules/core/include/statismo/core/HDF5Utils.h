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

#ifndef __STATIMO_CORE_HDF5_UTILS_H_
#define __STATIMO_CORE_HDF5_UTILS_H_

#include "statismo/core/CommonTypes.h"

namespace H5
{
class H5Location;
class Group;
class H5File;
class H5Object;
class DataSet;
} // namespace H5

namespace statismo
{

/**
 * \brief Wrapper class that gathers HDF5 utilities
 * \ingroup Core
 */
class HDF5Utils
{
public:
  /**
   * \brief Open or create hdf5
   * \param filename Path to the file
   */
  static H5::H5File
  OpenOrCreateFile(const std::string & filename);

  /**
   * \brief Opens the hdf5 group or creates it if it doesn't exist.
   * \param file file object
   * \param path absolute path that defines a group
   * \param createPath if true, creates the path if it does not exist
   * \return the group object representing the path in the hdf5 file
   */
  static H5::Group
  OpenPath(H5::H5File & file, const std::string & path, bool createPath = false);

  /**
   * \brief Read a Matrix from a HDF5 group
   * \param fg hdf5 group
   * \param name name of the entry
   * \param matrix output matrix
   */
  static void
  ReadMatrix(const H5::H5Location & fg, const char * name, MatrixType & matrix);

  /**
   * \brief Read a submatrix from the file, with the given number of columns
   * \param fg hdf5 group
   * \param name name of the entry
   * \param maxNumColumns number of columns to be read
   * \param matrix output matrix
   */
  static void
  ReadMatrix(const H5::H5Location & fg, const char * name, unsigned maxNumColumns, MatrixType & matrix);

  /**
   * \brief Read a Matrix of a given type from a HDF5 File
   * \param fg hdf5 group
   * \param name name of the entry
   * \param matrix output matrix
   */
  template <class T>
  static void
  ReadMatrixOfType(const H5::H5Location & fg, const char * name, typename GenericEigenTraits<T>::MatrixType & matrix);

  /**
   * \brief Write a Matrix to the HDF5 File
   * \param fg hdf5 group
   * \param name name of the entry
   * \param matrix to be written
   */
  static H5::DataSet
  WriteMatrix(const H5::H5Location & fg, const char * name, const MatrixType & matrix);

  /**
   * \brief Write a Matrix of the given type to the HDF5 File
   * \param fg hdf5 group
   * \param name name of the entry
   * \param matrix to be written
   */
  template <class T>
  static H5::DataSet
  WriteMatrixOfType(const H5::H5Location &                             fg,
                    const char *                                       name,
                    const typename GenericEigenTraits<T>::MatrixType & matrix);


  /**
   * \brief Read a Vector from a HDF5 File with the given number of elements
   * \param fg hdf5 group
   * \param name name of the entry
   * \param maxNumElements number of elements to be read from the file
   * \param vector output vector
   */
  static void
  ReadVector(const H5::H5Location & fg, const char * name, unsigned maxNumElements, VectorType & vector);

  /**
   * \brief Read a Vector from a HDF5 File
   * \param fg hdf5 group
   * \param name name of the entry
   * \param vector output vector
   */
  static void
  ReadVector(const H5::H5Location & fg, const char * name, VectorType & vector);

  template <class T>
  static void
  ReadVectorOfType(const H5::H5Location & fg, const char * name, typename GenericEigenTraits<T>::VectorType & vector);

  /**
   * \brief Write a vector to the HDF5 File
   * \param fg hdf5 group
   * \param name name of the entry
   * \param vector to be written
   */
  static H5::DataSet
  WriteVector(const H5::H5Location & fg, const char * name, const VectorType & vector);

  template <class T>
  static H5::DataSet
  WriteVectorOfType(const H5::H5Location &                             fg,
                    const char *                                       name,
                    const typename GenericEigenTraits<T>::VectorType & vector);


  /**
   * \brief Read a file (in binary mode) and saves it as a byte array in the hdf5 file.
   * \param filename filename of the file to be stored
   * \param fg hdf5 group
   * \param name name of the entry
   */
  static void
  DumpFileToHDF5(const char * filename, const H5::H5Location & fg, const char * name);

  /**
   * \brief Read an entry from an HDF5 byte array and writes it to a file
   * \param fg hdf5 group
   * \param name name of the entry
   * \param filename filename where the data from the HDF5 file is stored
   */
  static void
  GetFileFromHDF5(const H5::H5Location & fg, const char * name, const char * filename);

  /**
   * \brief Write a string to the hdf5 file
   * \param fg hdf5 group
   * \param name name of the entry in the group
   * \param s string to be written
   */
  static H5::DataSet
  WriteString(const H5::H5Location & fg, const char * name, const std::string & s);

  /**
   * \brief Read a string from the given group
   * \param fg hdf5 group
   * \param name name of the entry
   */
  static std::string
  ReadString(const H5::H5Location & fg, const char * name);

  /**
   * \brief Write a string attribute for the given group
   * \param fg hdf5 group
   * \param name name of the entry
   * \param s string to be written
   */
  static void
  WriteStringAttribute(const H5::H5Object & fg, const char * name, const std::string & s);

  /**
   * \brief Write an int attribute for the given group
   * \param fg hdf5 group
   * \param name name of the entry
   * \param value int value to be written
   */
  static void
  WriteIntAttribute(const H5::H5Object & fg, const char * name, int value);


  /**
   * \brief Read a string attribute from the given group
   * \param fg hdf5 group
   * \param name name of the entry
   */
  static std::string
  ReadStringAttribute(const H5::H5Object & fg, const char * name);

  /**
   * \brief Reads a int attribute from the given group
   * \param fg hdf5 group
   * \param name name of the entry in the group
   */
  static int
  ReadIntAttribute(const H5::H5Object & fg, const char * name);


  /**
   * \brief Read an integer from the hdf5 file
   * \param fg hdf5 group
   * \param name name of the entry
   */
  static int
  ReadInt(const H5::H5Location & fg, const char * name);

  /**
   * \brief Write an integer to the hdf5 file
   * \param fg hdf5 group
   * \param name name of the entry
   * \param value The value to be written
   */
  static H5::DataSet
  WriteInt(const H5::H5Location & fg, const char * name, int value);

  /**
   * \brief Read a double from the hdf5 file
   * \param fg hdf5 group
   * \param name name of the entry
   */
  static float
  ReadFloat(const H5::H5Location & fg, const char * name);

  /**
   * \brief Write a double to the hdf5 file
   * \param fg hdf5 group
   * \param name name of the entry
   * \param value value to be written
   */
  static H5::DataSet
  WriteFloat(const H5::H5Location & fg, const char * name, float value);

  /**
   * \brief Read an array from the hdf5 group
   * \param fg hdf5 group
   * \param name name of the entry
   * \param array output array
   */
  template <typename T>
  static void
  ReadArray(const H5::H5Location & fg, const char * name, std::vector<T> & array);

  /**
   * \brief Write an array to the hdf5 group
   * \param fg hdf5 group
   * \param name name of the entry
   * \param array array to be written
   */
  template <typename T>
  static H5::DataSet
  WriteArray(const H5::H5Location & fg, const char * name, std::vector<T> const & array);


  /**
   * \brief Check whether an object (direct child) of fg with the given name exists
   */
  static bool
  ExistsObjectWithName(const H5::H5Location & fg, const std::string & name);
};
} // namespace statismo

#include "HDF5Utils.hxx"

#endif
