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
 
#ifdef SWIGPYTHON

%{
#include "vtkPythonUtil.h"
#include "vtkVersion.h"

#include "statismo/core/CommonTypes.h"

#define SWIG_FILE_WITH_INIT
%}
%include "numpy.i"
%init %{
import_array();
%}

//
// Statismo typemap
//

%typemap (out) statismo::VectorType
{
    npy_intp dims[1];
    dims[0] = $1.rows();
    PyObject* c = PyArray_SimpleNew(1, dims, NPY_FLOAT);        
    memcpy(PyArray_DATA((PyArrayObject*)c), $1.data(), dims[0] * sizeof(float));
    $result= c;  
}

%typemap (out) const statismo::VectorType&
{
    npy_intp dims[1];
    dims[0] = $1->rows();
    PyObject* c = PyArray_SimpleNew(1, dims, NPY_FLOAT);        
    memcpy(PyArray_DATA((PyArrayObject*)c), $1->data(), dims[0] * sizeof(float));
    $result= c;  
}

%typemap (in) const statismo::VectorType& (statismo::VectorType temp)
{
    PyArrayObject* array = (PyArrayObject*)PyArray_ContiguousFromObject($input, NPY_DOUBLE, 1, 1);
    unsigned dim = PyArray_DIM(array,0);

    temp.resize(dim, Eigen::NoChange);
    for (unsigned i = 0; i < dim; i++) { 
        temp(i) = (float) ((double*) PyArray_DATA(array))[i];
    }         
    $1= &temp;
}

%typemap (in) (statismo::VectorType)
{
    PyArrayObject* array = (PyArrayObject*) PyArray_ContiguousFromObject($input, NPY_DOUBLE, 1, 1);
    unsigned dim = PyArray_DIM(array,0);
    
    $1.resize(dim, Eigen::NoChange);
    for (unsigned i = 0; i < dim; i++) { 
        $1(i) = (float)((double*) PyArray_DATA(array))[i];
    }         
}

%typemap(typecheck, precedence=SWIG_TYPECHECK_POINTER) statismo::VectorType, const statismo::VectorType&
{
    $1 = PyArray_Check($input) && PyArray_NDIM((PyArrayObject*)$input) == 1 ? 1 : 0;
}

%typemap (out) statismo::MatrixType
{
    npy_intp dims[2];
    dims[0] = $1.rows();
    dims[1] = $1.cols();
    PyObject* c = PyArray_SimpleNew(2, dims, NPY_FLOAT);        
    memcpy(PyArray_DATA((PyArrayObject*)c), $1.data(), dims[0] * dims[1] * sizeof(float));
    $result = c;
}

%typemap (out) const statismo::MatrixType&
{
    npy_intp dims[2];
    dims[0] = $1->rows();
    dims[1] = $1->cols();
    PyObject* c = PyArray_SimpleNew(2, dims, NPY_FLOAT);
    memcpy(PyArray_DATA((PyArrayObject*)c), $1->data(), dims[0] * dims[1] * sizeof(float));
    $result = c;
}

%typemap (in) const statismo::MatrixType& (statismo::MatrixType temp)
{
    PyArrayObject* array = (PyArrayObject*) PyArray_ContiguousFromObject($input, NPY_DOUBLE, 2, 2);
    unsigned dim1 = PyArray_DIM(array,0);
    unsigned dim2 = PyArray_DIM(array,1);
    
    temp.resize(dim1, dim2);
    for (unsigned i = 0; i < dim1; i++) { 
        for (unsigned j = 0; j < dim2; j++) {
            temp(i,j) = (float) ((double*) PyArray_DATA(array))[i* dim2 + j];
        }
    }         
    $1= &temp;
}

%typemap (in) statismo::MatrixType
{
    PyArrayObject* array = (PyArrayObject*) PyArray_ContiguousFromObject($input, NPY_DOUBLE, 2, 2);
    unsigned dim1 = PyArray_DIM(array,0);
    unsigned dim2 = PyArray_DIM(array,1);

    $1.resize(dim1, dim2);
    for (unsigned i = 0; i < dim1; i++) { 
        for (unsigned j = 0; j < dim2; j++) {
        $1(i,j) = (float) ((double*) PyArray_DATA(array))[i* dim2 + j];
        }
    }    
}

%typemap(typecheck, precedence=SWIG_TYPECHECK_POINTER) statismo::MatrixType, const statismo::MatrixType&
{
    // we could check it is a 2-d sequence here
    $1 = PyArray_Check($input) && PyArray_NDIM((PyArrayObject*)$input) == 2 ? 1 : 0;
}

// Grab a 3 element array as a Python 3-tuple
%typemap(in) double[3](double temp[3]) {   // temp[3] becomes a local variable
    if (PyTuple_Check($input)) {
    if (!PyArg_ParseTuple($input,"ddd",temp,temp+1,temp+2)) {
        PyErr_SetString(PyExc_TypeError,"tuple must have 3 elements");
        return NULL;
    }
    $1 = &temp[0];
    } else {
    PyErr_SetString(PyExc_TypeError,"expected a tuple.");
    return NULL;
    }
}

%typemap (out) (statismo::vtkPoint)
{
    $result = PyTuple_New(3);
    statismo::vtkPoint pt = $1;
    PyTuple_SetItem($result,0,PyFloat_FromDouble(pt[0]));
    PyTuple_SetItem($result,1,PyFloat_FromDouble(pt[1]));
    PyTuple_SetItem($result,2,PyFloat_FromDouble(pt[2]));                
}

%typemap (in) const double* (std::unique_ptr<double[]> temp)
{
    temp.reset(new double[PySequence_Length($input)]);
    for (int i = 0; i < PySequence_Length($input); i++) {
        PyObject *o = PySequence_GetItem($input,i);
        if (PyNumber_Check(o)) {
            temp[i] = PyFloat_AsDouble(o);
        } else {
            PyErr_SetString(PyExc_ValueError,"Sequence elements must be numbers");
            return NULL;
        }
    }  

    $1= temp.get();
}

%typemap(typecheck, precedence=SWIG_TYPECHECK_POINTER) const double*
{
    $1 = PySequence_Check($input) ? 1 : 0;
}

//
// VTK typemaps
//

%define VTK_TYPEMAPS(type)
%typemap (in) type*
{
    $1 =  static_cast<type*>(vtkPythonUtil::GetPointerFromObject($input, #type));
}

%typemap (out) type*
{
    $result =  vtkPythonUtil::GetObjectFromPointer(static_cast<type*>($1));
}

%typemap (out) const type*
{
    $result =  vtkPythonUtil::GetObjectFromPointer(const_cast<type*>(static_cast<const type*>($1)));
}

%typemap (out) vtkSmartPointer<type>
{
    auto rawPtr = type::New();
    rawPtr->DeepCopy(static_cast<type*>($1));

    $result =  vtkPythonUtil::GetObjectFromPointer(rawPtr);
}

%typecheck(SWIG_TYPECHECK_POINTER) type * {
    $1 = dynamic_cast<type*>(vtkPythonUtil::GetPointerFromObject($input, #type)) ? 1 : 0;
}
%enddef

VTK_TYPEMAPS(vtkPolyData)
VTK_TYPEMAPS(vtkUnstructuredGrid)
VTK_TYPEMAPS(vtkStructuredPoints)

#else
  #warning no "in" typemap defined
#endif

