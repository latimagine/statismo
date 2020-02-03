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
from statutils import get_polydata_dir, build_pd_model, get_point_from_id, \
    get_structured_points_dir, build_sp_model

import unittest
import tempfile
from os import listdir
from os.path import join
from scipy.stats import norm as normdist
from numpy import zeros
from numpy.random import randn
from numpy.lib.scimath import log


class Test(unittest.TestCase):

    def setUp(self):
        self.pd_model = build_pd_model(get_polydata_dir(), 0)
        self.sp_model = build_sp_model(get_structured_points_dir(), 0)

    def tearDown(self):
        pass

    def check_sample_vector_to_sample(self, model):
        # check whether the mechanism for vectors is ok in the first place
        coeffs = zeros(model.GetNumberOfPrincipalComponents())
        self.assertTrue((model.DrawSampleVector(coeffs) ==
                         model.GetMeanVector()).all() == True)

        # now we check that after drawing a sample the equality still holds
        sample = model.DrawSample(coeffs)
        sample_points = sample.GetPoints()
        sample_vec = model.DrawSampleVector(coeffs)
        for pid in range(0, sample.GetNumberOfPoints()):
            self.assertTrue(sample_points.GetPoint(pid)[0] == sample_vec[pid * 3] and
                            sample_points.GetPoint(pid)[1] == sample_vec[pid * 3 + 1] and
                            sample_points.GetPoint(pid)[2] == sample_vec[pid * 3 + 2])

    def test_sample_vector_to_sample_pd(self):
        self.check_sample_vector_to_sample(self.pd_model)

    def check_eval_sample_at_point(self, model):
        # draw a sample and check if the point evaluated is the same as what would be obtained by drawing the smaple directly at the point
        pt = model.GetDomain().GetDomainPoints(
        )[model.GetDomain().GetNumberOfPoints() // 2]

        sample = model.DrawMean()
        point_sample = model.DrawMeanAtPoint(pt)
        point_sample2 = model.EvaluateSampleAtPoint(sample, pt)

        self.assertTrue(point_sample[0] == point_sample2[0] and
                        point_sample[1] == point_sample2[1] and
                        point_sample[2] == point_sample2[2])

    def test_eval_sample_at_point_pd(self):
        self.check_eval_sample_at_point(self.pd_model)

    def check_draw_sample_at_points(self, model):
        # choose arbitrary coefficients
        coeffs = zeros(model.GetNumberOfPrincipalComponents())
        coeffs[0] = -1
        coeffs[1] = 1

        sample = model.DrawSample(coeffs)
        points_count = sample.GetNumberOfPoints()
        for pid in range(0, points_count, points_count // 10):
            sampleAtPt = model.DrawSampleAtPoint(coeffs, pid)
            self.assertTrue(sampleAtPt[0] == sample.GetPoints().GetPoint(pid)[0] and
                            sampleAtPt[1] == sample.GetPoints().GetPoint(pid)[1] and
                            sampleAtPt[2] == sample.GetPoints().GetPoint(pid)[2])

    def test_draw_sample_at_points_pd(self):
        self.check_draw_sample_at_points(self.pd_model)

    def check_load_save(self, model, representer, load_cb, save_cb):
        tmpfile = tempfile.mktemp(suffix="h5")
        save_cb(model, tmpfile)

        new_model = load_cb(representer, tmpfile)
        self.assertTrue((model.GetPCAVarianceVector() ==
                         new_model.GetPCAVarianceVector()).all())
        self.assertTrue((model.GetMeanVector() ==
                         new_model.GetMeanVector()).all())
        mat_p = model.GetPCABasisMatrix()
        new_mat_p = new_model.GetPCABasisMatrix()
        self.assertTrue((abs(mat_p - new_mat_p) < 1e-5).all())

        # check model info
        scores = model.GetModelInfo().GetScoresMatrix()
        new_scores = new_model.GetModelInfo().GetScoresMatrix()

        builder_info_list = model.GetModelInfo().GetBuilderInfoList()
        new_builder_info_list = model.GetModelInfo().GetBuilderInfoList()
        self.assertEqual(len(builder_info_list), len(new_builder_info_list), 1)

        di = builder_info_list[0].GetDataInfo()  # there is only one list
        # there is only one list
        new_di = new_builder_info_list[0].GetDataInfo()
        self.assertEqual(len(di), len(new_di))

        for (sorted_di, new_sorted_di) in zip(sorted(di), sorted(new_di)):
            self.assertEqual(sorted_di[0], new_sorted_di[0])
            self.assertEqual(sorted_di[1], new_sorted_di[1])

        pi = builder_info_list[0].GetParameterInfo()
        # there is only one list
        new_pi = new_builder_info_list[0].GetParameterInfo()
        self.assertEqual(len(pi), len(new_pi))

        for (sorted_pi, new_sorted_pi) in zip(sorted(pi), sorted(new_pi)):
            self.assertEqual(sorted_pi[0], new_sorted_pi[0])
            self.assertEqual(sorted_pi[1], new_sorted_pi[1])

        self.assertTrue(((scores - new_scores) < 1e-3).all())

        # now we test if loading with less parameters loads the right submatrix
        new_model_sub = load_cb(representer, tmpfile, 2)

        self.assertEqual(new_model_sub.GetNumberOfPrincipalComponents(), 2)
        self.assertTrue((new_model_sub.GetPCAVarianceVector()[
                        0:1] == model.GetPCAVarianceVector()[0:1]).all)
        self.assertTrue((abs(new_model_sub.GetPCABasisMatrix()[
                        :, 0:1] - model.GetPCABasisMatrix()[:, 0:1]) < 1e-3).all())

    def test_load_save_pd(self):
        self.check_load_save(self.pd_model, statismo.vtkStandardMeshRepresenter.Create(), statismo.IO_vtkPD.LoadStatisticalModel,
                             statismo.IO_vtkPD.SaveStatisticalModel)

    def test_load_save_sp(self):
        self.check_load_save(self.sp_model, statismo.vtkStandardImageRepresenter_F2.Create(), statismo.IO_vtkSP.LoadStatisticalModel,
                          statismo.IO_vtkSP.SaveStatisticalModel)

    def check_probability(self, model):
        samples_count = 100
        components_count = model.GetNumberOfPrincipalComponents()
        for _ in range(0, samples_count):
            coeffs = randn(components_count)
            s = model.DrawSample(coeffs)
            p = model.ComputeProbability(s)

            # as the distribution we are looking for is just a standard mv normal, all the
            # components are independent. We can thus use a 1d normal distribution to compute the
            # probability of the full pdf.
            p_with_spicy = 1
            for c in coeffs:
                p_with_spicy *= normdist(0, 1).pdf(c)
            self.assertTrue(p, p_with_spicy)

    def test_probability_pd(self):
        self.check_probability(self.pd_model)

    def test_probability_sp(self):
        self.check_probability(self.sp_model)

    def check_log_probability(self, model):
        samples_count = 100

        for _ in range(0, samples_count):
            coeffs = randn(model.GetNumberOfPrincipalComponents())
            s = model.DrawSample(coeffs)
            p = model.ComputeProbability(s)
            lp = model.ComputeLogProbability(s)
            self.assertTrue(log(
                p) - lp < 0.05, "Log probability should roughtly equal the log of the probability")

    def test_log_probability_pd(self):
        self.check_log_probability(self.pd_model)

    def test_log_probability_sp(self):
        self.check_log_probability(self.sp_model)

    def check_mahalanobis_distance(self, model):
        mean = model.DrawMean()
        md_mean = model.ComputeMahalanobisDistance(mean)
        self.assertEqual(md_mean, 0)

        coeffs = zeros(model.GetNumberOfPrincipalComponents())
        coeffs[0] = 3
        s = model.DrawSample(coeffs)
        md_sample = model.ComputeMahalanobisDistance(s)
        self.assertAlmostEqual(md_sample, 3, places=3)

    def test_mahalanobis_distance_pd(self):
        self.check_mahalanobis_distance(self.pd_model)

    def test_mahalanobis_distance_sp(self):
        self.check_mahalanobis_distance(self.sp_model)

    def check_compute_coefficients(self, model):
        coeffs = randn(model.GetNumberOfPrincipalComponents())
        s = model.DrawSample(coeffs)
        computed_coeffs = model.ComputeCoefficients(s)
        # we ignore the last coefficient
        diff = (coeffs - computed_coeffs)[:-1]
        # don't make the threshold too small, as we are dealing with floats
        self.assertTrue((diff < 1e-3).all())

    def test_compute_coefficients_pd(self):
        self.check_compute_coefficients(self.pd_model)

    def test_compute_coefficients_sp(self):
        self.check_compute_coefficients(self.sp_model)

    def check_compute_coefficients_for_point(self, model):
        coeffs = randn(model.GetNumberOfPrincipalComponents())
        s = model.DrawSample(coeffs)

        points_count = s.GetNumberOfPoints()

        pv_list = statismo.PointPointValueList()
        pidv_list = statismo.PointIdPointValueList()
        for pid in range(0, points_count, points_count // 500):
            # we create a list of fixed points once using the point id ....
            val = statismo.vtkPoint(*get_point_from_id(s, pid))
            pidv_list.append(statismo.PointIdPointValuePair(pid, val))

            # ... and once more with the points
            representer = model.GetRepresenter()
            ref_pt = statismo.vtkPoint(
                *get_point_from_id(representer.GetReference(), pid))
            pv_list.append(statismo.PointPointValuePair(ref_pt, val))

        computed_coeffs_pids = model.ComputeCoefficientsForPointIDValues(
            pidv_list)
        computed_coeffs_pts = model.ComputeCoefficientsForPointValues(
            pv_list)

        # does the list with the point and the one with the point ids yield the same result
        self.assertTrue((computed_coeffs_pids == computed_coeffs_pts).all())

        # now compare it to the real coefficients
        # we ignore the last coefficient
        diff = (coeffs - computed_coeffs_pts)[:-1]
        self.assertTrue((diff < 1e-3).all())

    def test_compute_coefficients_for_point_pd(self):
        self.check_compute_coefficients_for_point(self.pd_model)

    def check_matrix_state(self, model):
        representer = model.GetRepresenter()
        points_count = representer.GetReference().GetNumberOfPoints()
        dim = representer.GetDimensions()
        pcaBasisMatrix = model.GetPCABasisMatrix()
        self.assertEqual(pcaBasisMatrix.shape[0], points_count * dim)
        self.assertEqual(
            pcaBasisMatrix.shape[1], model.GetNumberOfPrincipalComponents())

        mean_vec = model.GetMeanVector()
        self.assertEqual(mean_vec.shape[0], points_count * dim)

    def test_matrix_state_pd(self):
        self.check_matrix_state(self.pd_model)

    def test_matrix_state_sp(self):
        self.check_matrix_state(self.sp_model)


if __name__ == "__main__":
    unittest.main()
