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

#ifndef __STATIMO_ITK_TYPE_CONVERSION_TRAIT_H_
#define __STATIMO_ITK_TYPE_CONVERSION_TRAIT_H_

#include "statismo/core/Exceptions.h"
#include "statismo/core/CommonTypes.h"

#include <itkVector.h>

#include <type_traits>

namespace itk
{

// \note These traits are used to allow a conversion from the generic pixel type to a statismo vector.
// \warning Currently only scalar types are supported.

namespace details {
  template <typename T, typename = void>
  struct PixelConversionTraitImpl;

  template <typename T>
  struct PixelConversionTraitImpl<T, std::enable_if_t<std::is_scalar_v<T>>> {
    static statismo::VectorType
    ToVector(T pixel)
    {
      statismo::VectorType v(1);
      v << pixel;
      return v;
    }

    static T
    FromVector(const statismo::VectorType & v)
    {
      assert(v.size() == 1);
      return v(0);
    }

    static constexpr unsigned
    GetDataType()
    {
      return statismo::GetDataTypeId<T>();
    }

    static constexpr unsigned
    GetPixelDimension()
    {
      return 1;
    }
  };

  template <typename U, auto N>
  struct PixelConversionTraitImpl<itk::Vector<U, N>, std::enable_if_t<std::is_scalar_v<U>>>
  {
    static statismo::VectorType
    ToVector(const itk::Vector<U, N>& pixel)
    {
      statismo::VectorType v(N);
      for (std::size_t i = 0; i < N; ++i) {
        v(i) = pixel[i];
      }
      return v;
    }
    static auto
    FromVector(const statismo::VectorType & v)
    {
      assert(v.size() == N);
      itk::Vector<U, N> itkVec;
      for (std::size_t i = 0; i < N; ++i) {
        itkVec[i] = v(i);
      }
      return itkVec;
    }
    static constexpr unsigned
    GetDataType()
    {
      return statismo::GetDataTypeId<U>();
    }
    static constexpr unsigned
    GetPixelDimension()
    {
      return N;
    }
};
}

// We use an internal impl to avoid exposing 
// the second dummy parameters to api user
template <typename T>
struct PixelConversionTrait : details::PixelConversionTraitImpl<T>
{
};
} // namespace itk

#endif
