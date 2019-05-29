/****************************************************************************

 Copyright (C) 2017-2018 Jan Hackenberg, free software developer
 All rights reserved.

 Contact : https://github.com/SimpleForest

 Developers : Jan Hackenberg

 This file is part of SimpleForest plugin Version 1 for Computree.

 SimpleForest plugin is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 SimpleForest plugin is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with SimpleForest plugin.  If not, see <http://www.gnu.org/licenses/>.

 PluginSimpleForest is an extended version of the SimpleTree platform.

*****************************************************************************/

#include "sf_downhillsimplex.h"

#include "qsm/algorithm/cloudQSM/sf_clustercloudbyqsm.h"

#include <cstdint>
#include <stdio.h>

#include <functional>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_ieee_utils.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_min.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_multifit_nlin.h>
#include <gsl/gsl_multifit_nlinear.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_test.h>
#include <vector>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_math.h>

SF_ParamSpherefollowingAdvanced<SF_PointNormal>
SF_DownHillSimplex::params() const
{
  return m_params;
}

void
SF_DownHillSimplex::setParams(const SF_ParamSpherefollowingAdvanced<SF_PointNormal>& params)
{
  m_params = params;
}

SF_DownHillSimplex::SF_DownHillSimplex() {}

void
SF_DownHillSimplex::serializeVec(gsl_vector* x, double fac, size_t numberClusters)
{
  size_t index = 0;
  for (size_t i = 0; i < numberClusters; i++) {
    gsl_vector_set(x, index++, static_cast<double>(m_params._sphereFollowingParams.m_optimizationParams[i]._epsilonSphere * fac));
    gsl_vector_set(
      x, index++, static_cast<double>(m_params._sphereFollowingParams.m_optimizationParams[i]._euclideanClusteringDistance * fac));
    gsl_vector_set(
      x, index++, static_cast<double>(m_params._sphereFollowingParams.m_optimizationParams[i]._sphereRadiusMultiplier * fac));
  }
}

void
SF_DownHillSimplex::compute()
{
  size_t size = m_params._clusters.size();
  auto allClusters = m_params._clusters;
  for (size_t numClusters = 1; numClusters <= size; numClusters++) {
    m_params.m_numClstrs = numClusters;
    std::vector<SF_CloudNormal::Ptr> clusters;
    for (size_t i = 0; i < numClusters; i++) {
      clusters.push_back(allClusters[i]);
    }
    m_params._clusters = clusters;
    m_params.m_numClstrs = numClusters;
    if (numClusters > m_params._sphereFollowingParams.m_optimizationParams.size()) {
      m_params._sphereFollowingParams.m_optimizationParams.push_back(
        m_params._sphereFollowingParams.m_optimizationParams[m_params._sphereFollowingParams.m_optimizationParams.size() - 1]);
    }
    std::uintptr_t par[1] = { reinterpret_cast<std::uintptr_t>(new SF_ParamSpherefollowingAdvanced<SF_PointNormal>(m_params)) };
    const gsl_multimin_fminimizer_type* T = gsl_multimin_fminimizer_nmsimplex2;
    gsl_multimin_fminimizer* s = NULL;
    gsl_vector *ss, *x;
    x = gsl_vector_alloc(numClusters * 3);
    ss = gsl_vector_alloc(numClusters * 3);
    gsl_multimin_function minex_func;
    size_t iter = 0;
    int status;
    double size;
    serializeVec(x, 1.0, numClusters);
    serializeVec(ss, 0.2, numClusters);
    minex_func.n = numClusters * 3;
    minex_func.f = downhillSimplex;
    minex_func.params = par;
    s = gsl_multimin_fminimizer_alloc(T, numClusters * 3);
    gsl_multimin_fminimizer_set(s, &minex_func, x, ss);
    do {
      iter++;
      status = gsl_multimin_fminimizer_iterate(s);

      if (status)
        break;
      size = gsl_multimin_fminimizer_size(s);
      status = gsl_multimin_test_size(size, m_params._fitQuality * 1e-3);
      if (status == GSL_SUCCESS) {
        printf("converged to minimum at\n");
      }
      double error = s->fval;
      if (m_params._modelCloudError > error) {
        m_params._modelCloudError = error;
        size_t index = 0;
        std::vector<SF_SphereFollowingOptimizationParameters> paramVec;
        SF_SphereFollowingOptimizationParameters paramsOptim;
        gsl_vector* v = s->x;
        for (size_t i = 0; i < numClusters; i++) {
          paramsOptim._epsilonSphere = gsl_vector_get(v, index++);
          paramsOptim._euclideanClusteringDistance = gsl_vector_get(v, index++);
          paramsOptim._sphereRadiusMultiplier = gsl_vector_get(v, index++);
          paramVec.push_back(paramsOptim);
        }
        m_params._sphereFollowingParams.m_optimizationParams = paramVec;
      }
      printf("%5d %10.6e %10.6e f() = %7.6f size = %.6f\n",
             static_cast<int>(iter),
             gsl_vector_get(s->x, 0),
             gsl_vector_get(s->x, 1),
             error,
             size);
      std::cout << std::endl;

    } while (status == GSL_CONTINUE && iter < m_params._iterations);
    gsl_vector_free(x);
    gsl_vector_free(ss);
    gsl_multimin_fminimizer_free(s);

    SF_SphereFollowing sphereFollowing;
    sphereFollowing.setParams(m_params);
    sphereFollowing.setClusters(clusters);
    sphereFollowing.compute();
    m_params._qsm = sphereFollowing.getQSM();
    m_params._cloudIn = sphereFollowing.cloud();
  }
}

double
downhillSimplex(const gsl_vector* v, void* params)
{
  std::uintptr_t* p = (std::uintptr_t*)params;
  SF_ParamSpherefollowingAdvanced<pcl::PointXYZINormal>* paramsBasic =
    reinterpret_cast<SF_ParamSpherefollowingAdvanced<pcl::PointXYZINormal>*>(p[0]);
  std::vector<SF_SphereFollowingOptimizationParameters> paramVec;
  size_t index = 0;
  SF_SphereFollowingOptimizationParameters paramsOptim = paramsBasic->_sphereFollowingParams.m_optimizationParams[0];
  for (size_t i = 0; i < paramsBasic->m_numClstrs; i++) {
    paramsOptim._epsilonSphere = gsl_vector_get(v, index++);
    paramsOptim._euclideanClusteringDistance = gsl_vector_get(v, index++);
    paramsOptim._sphereRadiusMultiplier = gsl_vector_get(v, index++);
    paramVec.push_back(paramsOptim);
  }
  paramsBasic->_sphereFollowingParams.m_optimizationParams = paramVec;
  SF_SphereFollowing sphereFollowing;
  sphereFollowing.setParams(*paramsBasic);
  sphereFollowing.setClusters(paramsBasic->_clusters);
  try {
    sphereFollowing.compute();
    paramsBasic->_tree = sphereFollowing.getQSM();
    paramsBasic->_modelCloudError = sphereFollowing.error();
  } catch (...) {
    paramsBasic->_tree = sphereFollowing.getQSM();
    paramsBasic->_modelCloudError = std::numeric_limits<float>::max();
  }
  double res = (paramsBasic->_modelCloudError);
  return res;
}
