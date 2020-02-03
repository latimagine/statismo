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
from statutils import get_polydata_dir, get_surrogates_data_dir, get_data_files, \
    read_vtkpd, get_point_from_id, build_pd_manager, get_structured_points_dir, \
    build_sp_manager, get_coords_from_id

import unittest
import tempfile
from os import listdir
from os.path import join
from scipy import randn, log, any, identity
from numpy import isnan, zeros
from numpy.lib.scimath import sqrt


class Test(unittest.TestCase):

    def setUp(self):
        self.pd_files = get_data_files(get_polydata_dir())
        self.pd_representer, self.pd_manager = build_pd_manager(
            get_polydata_dir())

        self.sp_files = get_data_files(get_structured_points_dir())
        self.sp_representer, self.sp_manager = build_sp_manager(
            get_structured_points_dir())

    def tearDown(self):
        pass

    def assertPointsAlmostEquals(self, pts1, pts2, numPoints, noise):
        for i in range(0, pts1.GetNumberOfPoints(), pts1.GetNumberOfPoints() // numPoints):
            self.assertTrue(
                abs(pts1.GetPoint(i)[0] - pts2.GetPoint(i)[0]) <= max(sqrt(noise), 1e-2))
            self.assertTrue(
                abs(pts1.GetPoint(i)[1] - pts2.GetPoint(i)[1]) <= max(sqrt(noise), 1e-2))
            self.assertTrue(
                abs(pts1.GetPoint(i)[2] - pts2.GetPoint(i)[2]) <= max(sqrt(noise), 1e-2))

    def build_and_test_model(self, model, manager, files, noise):
        self.assertTrue(model.GetNumberOfPrincipalComponents() <= len(files))

        # we cannot have negative eigenvalues
        self.assertTrue((model.GetPCAVarianceVector() >= 0).all() == True)
        self.assertTrue(isnan(model.GetPCAVarianceVector()).any() == False)

        # we project a dataset into the model and try to restore it.
        samples = manager.GetData()

        sample = samples[0].GetSample()
        coeffs = model.ComputeCoefficients(sample)
        restored_sample = model.DrawSample(coeffs)

        self.assertEqual(sample.GetNumberOfPoints(),
                         restored_sample.GetNumberOfPoints())
        self.assertPointsAlmostEquals(sample, restored_sample, 100, noise)

        # check if the scores can be used to restore the data in the datamanager
        scores = model.GetModelInfo().GetScoresMatrix()
        for i in range(0, scores.shape[1]):
            sample_from_scores = model.DrawSample(scores[:, i])
            sample_from_dm = samples[i].GetSample()

            self.assertPointsAlmostEquals(
                sample_from_scores, sample_from_dm, 100, noise)

        return model

    def build_and_test_model_pd(self, noise):
        builder = statismo.PCAModelBuilder_vtkPD.Create()
        model = builder.BuildNewModel(self.pd_manager.GetData(), noise)
        return self.build_and_test_model(model, self.pd_manager, self.pd_files, noise)

    def build_and_test_model_sp(self, noise):
        builder = statismo.PCAModelBuilder_vtkSP.Create()
        model = builder.BuildNewModel(self.sp_manager.GetData(), noise)
        return self.build_and_test_model(model, self.sp_manager, self.sp_files, noise)

    def check_pca_model_no_score(self, model, files):
        self.assertTrue(model.GetNumberOfPrincipalComponents() <= len(files))

        # we cannot have negative eigenvalues
        self.assertTrue((model.GetPCAVarianceVector() >= 0).all() == True)

        scores = model.GetModelInfo().GetScoresMatrix()
        self.assertTrue(scores.shape[0] == 0 and scores.shape[1] == 0)

    def test_pcamodel_builder_noscore_pd(self):
        # check if a model can be build when there are no scores
        builder = statismo.PCAModelBuilder_vtkPD.Create()
        model = builder.BuildNewModel(self.pd_manager.GetData(), 0, False)
        self.check_pca_model_no_score(model, self.pd_files)

    def test_pcamodel_builder_noscore_sp(self):
        # check if a model can be build when there are no scores
        builder = statismo.PCAModelBuilder_vtkSP.Create()
        model = builder.BuildNewModel(self.sp_manager.GetData(), 0, False)
        self.check_pca_model_no_score(model, self.sp_files)

    def test_pcamodel_builder_nonoise_pd(self):
        model = self.build_and_test_model_pd(0)
        self.assertAlmostEqual(model.GetNoiseVariance(), 0)

    def test_pcamodel_builder_nonoise_sp(self):
        model = self.build_and_test_model_sp(0)
        self.assertAlmostEqual(model.GetNoiseVariance(), 0)

    def test_pcamodel_builder_noise_pd(self):
        model = self.build_and_test_model_pd(0.1)
        self.assertAlmostEqual(model.GetNoiseVariance(), 0.1)

    def test_pcamodel_builder_noise_sp(self):
        model = self.build_and_test_model_sp(0.1)
        self.assertAlmostEqual(model.GetNoiseVariance(), 0.1)

    def test_pcamodel_builder_largenoise_pd(self):
        model = self.build_and_test_model_pd(1000)
        self.assertAlmostEqual(model.GetNoiseVariance(), 1000)

    def test_pcamodel_builder_largenoise_sp(self):
        model = self.build_and_test_model_sp(1000)
        self.assertAlmostEqual(model.GetNoiseVariance(), 1000)

    def test_posterior_builder_check_mean_pd(self):
        # if we fix many points to correspond to one of the samples, and build a
        # partially fixed model, its mean should correspond to the sample
        fixed_pt_count = 100
        test_pt_count = 1000

        sample = self.pd_manager.GetData()[0].GetSample()
        pv_list = statismo.PointPointValueList()

        domain_points = self.pd_representer.GetDomain().GetDomainPoints()

        for pid in range(0, len(domain_points), len(domain_points) // fixed_pt_count):
            fixed_pt = domain_points[pid]
            value = statismo.vtkPoint(*get_point_from_id(sample, pid))
            pv_list.append(statismo.PointPointValuePair(fixed_pt, value))

        post_builder = statismo.PosteriorModelBuilder_vtkPD.Create()
        post_model = post_builder.BuildNewModel(
            self.pd_manager.GetData(), pv_list, 0.1, 0.1)

        partial_mean = post_model.DrawMean()

        # now the sample that we used to fix the point should be similar to the mean. We test it by
        for pid in range(0, sample.GetNumberOfPoints(), sample.GetNumberOfPoints() // test_pt_count):
            mean_pt = get_point_from_id(partial_mean, pid)
            sample_pt = get_point_from_id(sample, pid)
            self.assertAlmostEqual(mean_pt[0], sample_pt[0], 0)
            self.assertAlmostEqual(mean_pt[1], sample_pt[1], 0)
            self.assertAlmostEqual(mean_pt[2], sample_pt[2], 0)

    def test_posterior_builder_noconstraint_pd(self):
        # if we fix no point, it should be the same as building a normal pca model
        pv_list = statismo.PointPointValueList()

        post_builder = statismo.PosteriorModelBuilder_vtkPD.Create()
        post_model = post_builder.BuildNewModel(
            self.pd_manager.GetData(), pv_list, 0.1, 0.1)

        pcabuilder = statismo.PCAModelBuilder_vtkPD.Create()
        pca_model = pcabuilder.BuildNewModel(self.pd_manager.GetData(), 0.1)

        sample = self.pd_manager.GetData()[0].GetSample()
        coeffs_post_model = post_model.ComputeCoefficients(sample)
        coeffs_pca_model = pca_model.ComputeCoefficients(sample)
        for i in range(0, len(coeffs_post_model)):
            # the sign is allowed to change
            self.assertAlmostEqual(
                abs(coeffs_post_model[i]), abs(coeffs_pca_model[i]), 1)

    def test_posterior_builder_variance_pd(self):
        # checks whether with every added point, the variance is decreasing
        reference = self.pd_representer.GetReference()
        sample = self.pd_manager.GetData()[0].GetSample()
        points_count = sample.GetNumberOfPoints()
        pv_list = statismo.PointPointValueList()

        post_builder = statismo.PosteriorModelBuilder_vtkPD.Create()
        post_model = post_builder.BuildNewModel(
            self.pd_manager.GetData(), pv_list, 0.1, 0.1)
        total_var = post_model.GetPCAVarianceVector().sum()
        for pid in range(0, points_count, points_count // 10):
            ref_pt = statismo.vtkPoint(*get_point_from_id(reference, pid))
            pt = statismo.vtkPoint(*get_point_from_id(sample, pid))
            pv_list.append(statismo.PointPointValuePair(ref_pt, pt))

            post_builder = statismo.PosteriorModelBuilder_vtkPD.Create()
            post_model = post_builder.BuildNewModel(
                self.pd_manager.GetData(), pv_list, 0.1, 0.1)
            total_sdev_prev = total_var
            total_var = post_model.GetPCAVarianceVector().sum()
            self.assertTrue(total_var < total_sdev_prev)

    def test_posterior_builder_constraint_pd(self):
        # Checks if a point that is fixed really stays where it was constrained to stay
        reference = self.pd_representer.GetReference()
        sample = self.pd_manager.GetData()[0].GetSample()
        pv_list = statismo.PointPointValueList()

        ref_pt = get_point_from_id(reference, 0)
        fixed_pt = get_point_from_id(sample, 0)
        pv_list.append(statismo.PointPointValuePair(
            statismo.vtkPoint(*ref_pt),  statismo.vtkPoint(*fixed_pt)))
        post_builder = statismo.PosteriorModelBuilder_vtkPD.Create()
        post_model = post_builder.BuildNewModel(
            self.pd_manager.GetData(), pv_list, 0.01, 0.01)

        # check for some samples if the points stay put
        coeffs1 = zeros(post_model.GetNumberOfPrincipalComponents())
        coeffs1[1] = 3
        coeffs2 = zeros(post_model.GetNumberOfPrincipalComponents())
        coeffs2[0] = -3

        for coeffs in [coeffs1, coeffs2]:
            constrained_sample = post_model.DrawSample(coeffs)
            self.assertAlmostEqual(
                constrained_sample.GetPoints().GetPoint(0)[0], fixed_pt[0], 1)
            self.assertAlmostEqual(
                constrained_sample.GetPoints().GetPoint(0)[1], fixed_pt[1], 1)
            self.assertAlmostEqual(
                constrained_sample.GetPoints().GetPoint(0)[2], fixed_pt[2], 1)

    def check_reduced_variance_builder(self, model, reduced_var_builder):
        reduced_component_count = model.GetNumberOfPrincipalComponents() // 2
        new_model = reduced_var_builder.BuildNewModelWithLeadingComponents(
            model, reduced_component_count)
        self.assertTrue(new_model.GetNumberOfPrincipalComponents()
                        == reduced_component_count)

        for total_var_ratio in [1.0, 0.9, 0.8, 0.7, 0.6, 0.4, 0.2, 0.1]:
            reduced_model = reduced_var_builder.BuildNewModelWithVariance(
                model, total_var_ratio)

            # we keep at least the required percentage of total variance
            self.assertTrue(reduced_model.GetPCAVarianceVector().sum(
            ) / model.GetPCAVarianceVector().sum() >= total_var_ratio)

            # make sure that one component less would not reach the variance
            self.assertTrue(reduced_model.GetPCAVarianceVector()[
                            0:-1].sum() / model.GetPCAVarianceVector().sum() < total_var_ratio)

        # check that there is a reduction (though we cannot say how much, as the specified variance is a lower bound)
        reduced_model05 = reduced_var_builder.BuildNewModelWithVariance(
            model, 0.5)
        self.assertTrue(reduced_model05.GetPCAVarianceVector().sum()
                        <= model.GetPCAVarianceVector().sum())

    def test_reduced_variance_builder_pd(self):
        builder = statismo.PCAModelBuilder_vtkPD.Create()

        model = builder.BuildNewModel(self.pd_manager.GetData(), 0.)
        reduced_var_builder = statismo.ReducedVarianceModelBuilder_vtkPD.Create()

        self.check_reduced_variance_builder(model, reduced_var_builder)

    def test_reduced_variance_builder_sp(self):
        builder = statismo.PCAModelBuilder_vtkSP.Create()

        model = builder.BuildNewModel(self.sp_manager.GetData(), 0.)
        reduced_var_builder = statismo.ReducedVarianceModelBuilder_vtkSP.Create()

        self.check_reduced_variance_builder(model, reduced_var_builder)

    def check_reduced_variance_builder_noscore(self, model, reduced_var_builder):
        reduced_component_count = model.GetNumberOfPrincipalComponents() // 2
        new_model = reduced_var_builder.BuildNewModelWithLeadingComponents(
            model, reduced_component_count)
        self.assertTrue(new_model.GetNumberOfPrincipalComponents()
                        == reduced_component_count)
        self.assertTrue(
            new_model.GetModelInfo().GetScoresMatrix().shape[0] == 0)

    def test_reduced_variance_builder_noscore_pd(self):
        # check that a model can also be reduced when no scores are present
        builder = statismo.PCAModelBuilder_vtkPD.Create()

        model = builder.BuildNewModel(self.pd_manager.GetData(), 0., False)
        reduced_var_builder = statismo.ReducedVarianceModelBuilder_vtkPD.Create()

        self.check_reduced_variance_builder_noscore(model, reduced_var_builder)

    def test_reduced_variance_builder_noscore_sp(self):
        # check that a model can also be reduced when no scores are present
        builder = statismo.PCAModelBuilder_vtkSP.Create()

        model = builder.BuildNewModel(self.sp_manager.GetData(), 0., False)
        reduced_var_builder = statismo.ReducedVarianceModelBuilder_vtkSP.Create()

        self.check_reduced_variance_builder_noscore(model, reduced_var_builder)


if __name__ == "__main__":
    unittest.main()
