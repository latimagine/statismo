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
from statutils import get_structured_points_dir, get_polydata_dir, get_surrogates_data_dir, \
    get_data_files, read_vtkpd, read_vtksp

import unittest
import tempfile
import os.path


class Test(unittest.TestCase):

    def setUp(self):
        self.pd_files = get_data_files(get_polydata_dir())
        self.pd_representer = statismo.vtkStandardMeshRepresenter.Create(
            read_vtkpd(self.pd_files[0]))
        self.sp_files = get_data_files(get_structured_points_dir())
        self.sp_representer = statismo.vtkStandardImageRepresenter_F2.Create(
            read_vtksp(self.sp_files[0])
        )

    def tearDown(self):
        pass

    def testName(self):
        pass

    def check_add_dataset(self, manager, datasets, files):
        for (dataset, filename) in zip(datasets, files):
            manager.AddDataset(dataset, filename)

        self.assertEqual(manager.GetNumberOfSamples(), len(files))

        for (i, sample) in enumerate(manager.GetData()):
            self.assertEqual(sample.GetDatasetURI(), files[i])

    def test_add_dataset_vtkpd(self):
        manager = statismo.BasicDataManager_vtkPD.Create(self.pd_representer)
        datasets = map(read_vtkpd, self.pd_files)
        self.check_add_dataset(manager, datasets, self.pd_files)

    def test_add_dataset_vtksp(self):
        manager = statismo.BasicDataManager_vtkSP.Create(self.sp_representer)
        datasets = map(read_vtksp, self.sp_files)
        self.check_add_dataset(manager, datasets, self.sp_files)

    def test_load_save_vtkpd(self):
        tmpfile = tempfile.mktemp(suffix="h5")
        manager = statismo.BasicDataManager_vtkPD.Create(self.pd_representer)

        datasets = map(read_vtkpd, self.pd_files)
        for (dataset, filename) in zip(datasets, self.pd_files):
            manager.AddDataset(dataset, filename)

        manager.Save(tmpfile)

        representer = statismo.vtkStandardMeshRepresenter.Create()
        new_manager = statismo.BasicDataManager_vtkPD.Load(
            representer, tmpfile)

        self.assertEqual(manager.GetNumberOfSamples(),
                         new_manager.GetNumberOfSamples())

        for (sample, new_sample) in zip(manager.GetData(), new_manager.GetData()):
            self.assertTrue((sample.GetSampleVector() ==
                             new_sample.GetSampleVector()).all())

    def test_load_save_vtksp(self):
        tmpfile = tempfile.mktemp(suffix="h5")
        manager = statismo.BasicDataManager_vtkSP.Create(self.sp_representer)

        datasets = map(read_vtksp, self.sp_files)
        for (dataset, filename) in zip(datasets, self.sp_files):
            manager.AddDataset(dataset, filename)

        manager.Save(tmpfile)

        representer = statismo.vtkStandardImageRepresenter_F2.Create()
        new_manager = statismo.BasicDataManager_vtkSP.Load(
            representer, tmpfile)

        self.assertEqual(manager.GetNumberOfSamples(),
                         new_manager.GetNumberOfSamples())

        for (sample, new_sample) in zip(manager.GetData(), new_manager.GetData()):
            self.assertTrue((sample.GetSampleVector() ==
                             new_sample.GetSampleVector()).all())

    def test_load_save_surrogates(self):
        tmpfile = tempfile.mktemp(suffix="h5")
        surrogates_dir = os.path.join(get_surrogates_data_dir(), "surrogates")
        typesfn = os.path.join(surrogates_dir, "hand_surrogates_types.txt")

        manager = statismo.DataManagerWithSurrogates_vtkPD.Create(
            self.pd_representer, typesfn)

        datasetfn = os.path.join(get_polydata_dir(), "hand-1.vtk")
        dataset = read_vtkpd(datasetfn)
        surrogatefn = os.path.join(surrogates_dir, "hand-1_surrogates.txt")

        manager.AddDatasetWithSurrogates(dataset, datasetfn, surrogatefn)

        manager.Save(tmpfile)

        representer = statismo.vtkStandardMeshRepresenter.Create()
        new_manager = statismo.DataManagerWithSurrogates_vtkPD.Load(
            representer, tmpfile, typesfn)

        self.assertEqual(manager.GetNumberOfSamples(),
                         new_manager.GetNumberOfSamples())

        for (sample, new_sample) in zip(manager.GetData(), new_manager.GetData()):
            self.assertTrue((sample.GetSampleVector() ==
                             new_sample.GetSampleVector()).all())

    def check_cross_validation(self, manager, datasets, files):
        for (dataset, filename) in zip(datasets, files):
            manager.AddDataset(dataset, filename)

        cvfolds = manager.GetCrossValidationFolds(3, True)

        self.assertEqual(len(cvfolds), 3)

        training_data = cvfolds[0].GetTrainingData()
        test_data = cvfolds[0].GetTestingData()

        self.assertTrue(len(training_data) + len(test_data)
                        == manager.GetNumberOfSamples())
        self.assertTrue(set(training_data).isdisjoint(test_data),
                        "a dataset is both in the test and training data")

    def test_cross_validation_pd(self):
        manager = statismo.BasicDataManager_vtkPD.Create(self.pd_representer)
        datasets = map(read_vtkpd, self.pd_files)
        self.check_cross_validation(manager, datasets, self.pd_files)

    def test_cross_validation_sp(self):
        manager = statismo.BasicDataManager_vtkSP.Create(self.sp_representer)
        datasets = map(read_vtksp, self.sp_files)
        self.check_cross_validation(manager, datasets, self.sp_files)


if __name__ == "__main__":
    unittest.main()
