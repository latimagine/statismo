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

#ifndef __STATIMO_CORE_COMMON_TYPES_H_
#define __STATIMO_CORE_COMMON_TYPES_H_

#include "statismo/core/CoreTraits.h"
#include "statismo/core/Exceptions.h"
#include "statismo/core/Config.h"

#include <memory>
#include <cstdint>

namespace statismo
{

// numerical value
static constexpr double gk_pi = 3.14159265358979323846;

// the type that is used for all vector and matrices throughout the library.
using ScalarType = float;

// aliases to generic eigen types
using MatrixType = GenericEigenTraits<ScalarType>::MatrixType;
using MatrixTypeDoublePrecision = GenericEigenTraits<double>::MatrixType;
using DiagMatrixType = GenericEigenTraits<ScalarType>::DiagMatrixType;
using VectorType = GenericEigenTraits<ScalarType>::VectorType;
using VectorTypeDoublePrecision = GenericEigenTraits<double>::VectorType;
using RowVectorType = GenericEigenTraits<ScalarType>::RowVectorType;

// type definitions used in the standard file format.
// Note that these are the same as used by VTK
static constexpr unsigned gk_void = 0; // not capitalized, as windows defines: #define VOID void, which causes trouble
static constexpr unsigned gk_signedChar = 2;
static constexpr unsigned gk_unsignedChar = 3;
static constexpr unsigned gk_signedShort = 4;
static constexpr unsigned gk_unsignedShort = 5;
static constexpr unsigned gk_signedInt = 6;
static constexpr unsigned gk_unsignedInt = 7;
static constexpr unsigned gk_signedLong = 8;
static constexpr unsigned gk_unsignedLong = 9;
static constexpr unsigned gk_float = 10;
static constexpr unsigned gk_double = 11;

template <class T>
constexpr unsigned
GetDataTypeId();

template <>
inline constexpr unsigned
GetDataTypeId<signed char>()
{
  return gk_signedChar;
}
template <>
inline constexpr unsigned
GetDataTypeId<unsigned char>()
{
  return gk_unsignedChar;
}
template <>
inline constexpr unsigned
GetDataTypeId<short>() // NOLINT
{
  return gk_signedShort;
}
template <>
inline constexpr unsigned
GetDataTypeId<unsigned short>() // NOLINT
{
  return gk_unsignedShort;
}
template <>
inline constexpr unsigned
GetDataTypeId<signed int>()
{
  return gk_signedInt;
}
template <>
inline constexpr unsigned
GetDataTypeId<unsigned int>()
{
  return gk_unsignedInt;
}
template <>
inline constexpr unsigned
GetDataTypeId<long>() // NOLINT
{
  return gk_signedLong;
}
template <>
inline constexpr unsigned
GetDataTypeId<unsigned long>() // NOLINT
{
  return gk_unsignedLong;
}
template <>
inline constexpr unsigned
GetDataTypeId<float>()
{
  return gk_float;
}
template <>
inline constexpr unsigned
GetDataTypeId<double>()
{
  return gk_double;
}

/**
 * Standard deletor
 */
template <typename T>
using StdDeletor = std::default_delete<T>;

/**
 * Custom deletor functor used as default deletor in smart memory management
 */
template <typename T>
struct DefaultDeletor
{
  void
  operator()(T * t)
  {
    t->Delete();
  }
};

/**
 * Custom unique pointer
 */
template <typename T>
using UniquePtrType = std::unique_ptr<T, DefaultDeletor<T>>;

/**
 * Custom shared pointer
 */
template <typename T>
using SharedPtrType = std::shared_ptr<T>;

/**
 * Factory function for shared pointer
 */
template <typename T>
auto
MakeSharedPointer(T && t)
{
  return std::shared_ptr<T>{ std::forward<T>(t), DefaultDeletor<T>() };
}

template <typename T>
auto
MakeSharedPointer(T * t)
{
  return std::shared_ptr<T>{ t, DefaultDeletor<T>() };
}

/**
 * Standard unique pointer
 */
template <typename T>
using StdUniquePtrType = std::unique_ptr<T, StdDeletor<T>>;

/**
 * RAII enforcer
 */
template <typename Callable>
class StackUnwinder
{
private:
  Callable m_c;
  bool     m_do{ true };

public:
  explicit StackUnwinder(Callable && c)
    : m_c{ std::move(c) }
  {}

  ~StackUnwinder()
  {
    if (m_do)
    {
      m_c();
    }
  }

  StackUnwinder(const StackUnwinder &) = delete;
  StackUnwinder(const StackUnwinder &&) = delete;
  StackUnwinder &
  operator=(const StackUnwinder &) = delete;
  StackUnwinder &
  operator=(StackUnwinder &&) = delete;

  void
  Set()
  {
    m_do = true;
  }

  void
  Unset()
  {
    m_do = false;
  }
};

template <typename Callable>
inline auto
MakeStackUnwinder(Callable && c)
{
  return StackUnwinder(std::forward<Callable>(c));
}

template <char D>
struct WordDelimiter : public std::string
{};

template <char D>
std::istream &
operator>>(std::istream & is, statismo::WordDelimiter<D> & output)
{
  std::getline(is, output, D);
  return is;
}

} // namespace statismo

#endif
