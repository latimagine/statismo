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

%{
#include "statismo/VTK/vtkStandardMeshRepresenter.h"
#include "statismo/VTK/vtkStandardImageRepresenter.h"
#include "statismo/VTK/vtkUnstructuredGridRepresenter.h"
%}

// \warning Include core AFTER vtkUnstructuredGridRepresenter
%include "statismoCore.i"

//
// Standard mesh reperesenter
//

%template(RepresenterBase_vtkPD) statismo::RepresenterBase<vtkPolyData, statismo::vtkStandardMeshRepresenter>;

namespace statismo {

class vtkStandardMeshRepresenter : public RepresenterBase<vtkPolyData, vtkStandardMeshRepresenter> {
    public:

        typedef RepresenterTraits<vtkPolyData>::DatasetPointerType DatasetPointerType;
        typedef typename RepresenterTraits<vtkPolyData>::DatasetConstPointerType DatasetConstPointerType;
        typedef typename RepresenterTraits<vtkPolyData>::PointType PointType;
        typedef typename RepresenterTraits<vtkPolyData>::ValueType ValueType;
        typedef statismo::Domain<PointType> DomainType;

        %newobject Create;
        static vtkStandardMeshRepresenter* Create(); 
        static vtkStandardMeshRepresenter* Create(const vtkPolyData* reference);

        DatasetConstPointerType GetReference() const;
    
        const DomainType& GetDomain() const;
        unsigned GetNumberOfPoints() const;
        unsigned GetPointIdForPoint(const vtkPoint & pt) const;

    private:
        vtkStandardMeshRepresenter();
        vtkStandardMeshRepresenter(const vtkPolyData* reference);
};

}

//
// Unstructured grid representer
//

%template(RepresenterBase_vtkUG) statismo::RepresenterBase<vtkUnstructuredGrid, statismo::vtkUnstructuredGridRepresenter>;

namespace statismo {

class vtkUnstructuredGridRepresenter : public RepresenterBase<vtkUnstructuredGrid, vtkUnstructuredGridRepresenter> {
  public:
    enum class AlignmentType {
      NONE=999,
      RIGID=VTK_LANDMARK_RIGIDBODY,
      SIMILARITY=VTK_LANDMARK_SIMILARITY,
      AFFINE=VTK_LANDMARK_AFFINE
    };

    typedef RepresenterTraits<vtkUnstructuredGrid>::DatasetPointerType DatasetPointerType;
        typedef typename RepresenterTraits<vtkUnstructuredGrid>::DatasetConstPointerType DatasetConstPointerType;
        typedef typename RepresenterTraits<vtkUnstructuredGrid>::PointType PointType;
        typedef typename RepresenterTraits<vtkUnstructuredGrid>::ValueType ValueType;
        typedef statismo::Domain<PointType> DomainType;

    %newobject Create;
    static vtkUnstructuredGridRepresenter* Create(); 
    static vtkUnstructuredGridRepresenter* Create(const vtkUnstructuredGrid* reference, statismo::vtkUnstructuredGridRepresenter::AlignmentType alignment);

    statismo::vtkUnstructuredGridRepresenter::AlignmentType GetAlignment() const;
      
    DatasetConstPointerType GetReference() const;
        const DomainType& GetDomain() const;
        unsigned GetNumberOfPoints() const;
        unsigned GetPointIdForPoint(const vtkPoint & pt) const;

  private:
    vtkUnstructuredGridRepresenter();
    vtkUnstructuredGridRepresenter(const vtkUnstructuredGrid* reference, statismo::vtkUnstructuredGridRepresenter::AlignmentType alignment);
};

}

//
// Standard image representer
//

%template(RepresenterBase_vtkSPF2) statismo::RepresenterBase<vtkStructuredPoints, statismo::vtkStandardImageRepresenter<float, 2>>;
%template(RepresenterBase_vtkSPF3) statismo::RepresenterBase<vtkStructuredPoints, statismo::vtkStandardImageRepresenter<float, 3>>;

namespace statismo {

template <class TPixel, unsigned TDimensions>
class vtkStandardImageRepresenter : public RepresenterBase<vtkStructuredPoints, vtkStandardImageRepresenter<TPixel, TDimensions>> {
    public:

        typedef RepresenterTraits<vtkStructuredPoints>::DatasetPointerType DatasetPointerType;
        typedef typename RepresenterTraits<vtkStructuredPoints>::DatasetConstPointerType DatasetConstPointerType;
        typedef typename RepresenterTraits<vtkStructuredPoints>::PointType PointType;
        typedef typename RepresenterTraits<vtkStructuredPoints>::ValueType ValueType;
        typedef statismo::Domain<PointType> DomainType;

        %newobject Create;
        static vtkStandardImageRepresenter* Create(); 
        static vtkStandardImageRepresenter* Create(const vtkStructuredPoints* reference);

        DatasetConstPointerType GetReference() const;
    
        const DomainType& GetDomain() const;
        unsigned GetNumberOfPoints() const;
        unsigned GetPointIdForPoint(const vtkPoint & pt) const;

    private:
        vtkStandardImageRepresenter();
        vtkStandardImageRepresenter(const vtkStructuredPoints* reference);
};

}

%template(vtkStandardImageRepresenter_F2) statismo::vtkStandardImageRepresenter<float, 2>;
%template(vtkStandardImageRepresenter_F3) statismo::vtkStandardImageRepresenter<float, 3>;
