#
# This file is part of the statismo library.
#
# Author: Marcel Luethi (marcel.luethi@unibas.ch)
#
# Copyright (c) 2011 University of Basel
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# Neither the name of the project's author nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#

import vtk
import statismovtk as statismo
import os
from os.path import dirname as up


def get_data_dir():
    try:
        data_dir = os.environ["STATISMO_DATA_DIR"]
    except KeyError:
        data_dir = os.path.join(
            up(up(up(up(up(up(os.path.abspath(__file__))))))), "data")

    return data_dir


def get_polydata_dir():
    return os.path.join(get_data_dir(), "hand_polydata")


def get_structured_points_dir():
    return os.path.join(get_data_dir(), "hand_dfs")


def get_surrogates_data_dir():
    return os.path.join(get_data_dir(), "hand_images")


def get_data_files(datadir):
    return [os.path.join(datadir, f) for f in os.listdir(datadir) if f.endswith('.vtk')]


def read_vtkpd(filename):
    reader = vtk.vtkPolyDataReader()
    reader.SetFileName(filename)
    reader.Update()

    return reader.GetOutput()


def read_vtksp(filename):
    reader = vtk.vtkStructuredPointsReader()
    reader.SetFileName(filename)
    reader.Update()

    return reader.GetOutput()


def build_pd_manager(datadir):
    files = get_data_files(datadir)
    ref = read_vtkpd(files[0])

    representer = statismo.vtkStandardMeshRepresenter.Create(ref)
    dm = statismo.BasicDataManager_vtkPD.Create(representer)

    datasets = map(read_vtkpd, files)
    for (dataset, filename) in zip(datasets, files):
        dm.AddDataset(dataset, filename)

    return (representer, dm)


def build_sp_manager(datadir):
    files = get_data_files(datadir)
    ref = read_vtksp(files[0])

    representer = statismo.vtkStandardImageRepresenter_F2.Create(ref)
    dm = statismo.BasicDataManager_vtkSP.Create(representer)

    datasets = map(read_vtksp, files)
    for (dataset, filename) in zip(datasets, files):
        dm.AddDataset(dataset, filename)

    return (representer, dm)


def build_sp_model(datadir, noise):

    _, dm = build_sp_manager(datadir)

    builder = statismo.PCAModelBuilder_vtkSP.Create()
    model = builder.BuildNewModel(dm.GetData(), noise)

    return model


def build_pd_model(datadir, noise):

    _, dm = build_pd_manager(datadir)

    builder = statismo.PCAModelBuilder_vtkPD.Create()
    model = builder.BuildNewModel(dm.GetData(), noise)

    return model


def get_point_from_id(data, id):
    x = data.GetPoint(id)[0]
    y = data.GetPoint(id)[1]
    z = data.GetPoint(id)[2]
    return (x, y, z)


def get_coords_from_id(data, id, dim):
    pts = []
    for i in range(0, dim):
        pts.append(data.GetPoint(id)[i])

    return pts
