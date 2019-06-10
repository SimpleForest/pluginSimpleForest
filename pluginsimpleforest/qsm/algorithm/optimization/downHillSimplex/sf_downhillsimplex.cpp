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

#include "cloud/filter/unary/maxIntensity/sf_maxintensity.h"
#include "qsm/algorithm/cloudQSM/sf_clustercloudbyqsm.h"
#include "qsm/algorithm/distance/sf_extractFittedPoints.h"

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
SF_DownHillSimplex::serializeVec(gsl_vector* x,
                                 double fac,
                                 size_t numberClusters,
                                 SF_ParamSpherefollowingAdvanced<SF_PointNormal>& params)
{
  size_t index = 0;
  for (size_t i = 0; i < numberClusters; i++) {
    gsl_vector_set(x, index++, static_cast<double>(params._sphereFollowingParams.m_optimizationParams[i]._epsilonSphere * fac));
    gsl_vector_set(
      x, index++, static_cast<double>(params._sphereFollowingParams.m_optimizationParams[i]._euclideanClusteringDistance * fac));
    gsl_vector_set(
      x, index++, static_cast<double>(params._sphereFollowingParams.m_optimizationParams[i]._sphereRadiusMultiplier * fac));
  }
}

void
SF_DownHillSimplex::computeDHS(SF_ParamSpherefollowingAdvanced<SF_PointNormal>& params, size_t numClusters)
{
  adjustParameters(params, numClusters);
  std::uintptr_t par[1] = { reinterpret_cast<std::uintptr_t>(new SF_ParamSpherefollowingAdvanced<SF_PointNormal>(params)) };
  const gsl_multimin_fminimizer_type* T = gsl_multimin_fminimizer_nmsimplex2;
  gsl_multimin_fminimizer* s = NULL;
  gsl_vector *ss, *x;
  x = gsl_vector_alloc(numClusters * 3);
  ss = gsl_vector_alloc(numClusters * 3);
  gsl_multimin_function minex_func;
  size_t iter = 0;
  int status;
  double size;
  serializeVec(x, 1.0, numClusters, params);
  serializeVec(ss, 0.1, numClusters, params);
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
    status = gsl_multimin_test_size(size, params._fitQuality * 1e-3);
    double error = s->fval;
    if (params._modelCloudError > error) {
      params._modelCloudError = error;
      size_t index = 0;
      std::vector<SF_SphereFollowingOptimizationParameters> paramVec = params._sphereFollowingParams.m_optimizationParams;
      gsl_vector* v = s->x;
      for (size_t i = 0; i < numClusters; i++) {
        paramVec[i]._epsilonSphere = gsl_vector_get(v, index++);
        paramVec[i]._euclideanClusteringDistance = gsl_vector_get(v, index++);
        paramVec[i]._sphereRadiusMultiplier = gsl_vector_get(v, index++);
      }
      params._sphereFollowingParams.m_optimizationParams = paramVec;
      m_params = params;
    }
    m_params._stepProgress->fireComputation();
  } while (status == GSL_CONTINUE && iter < static_cast<size_t>(params._iterations));
  while (iter < static_cast<size_t>(params._iterations)) {
    iter++;
    m_params._stepProgress->fireComputation();
  }
  gsl_vector_free(x);
  gsl_vector_free(ss);
  gsl_multimin_fminimizer_free(s);

  SF_SphereFollowing sphereFollowing;
  sphereFollowing.setParams(m_params);
  sphereFollowing.setCloud(params.m_cloudSphereFollowing);
  sphereFollowing.compute();

  params._qsm = sphereFollowing.getQSM();
  m_params = params;
  m_params.m_cloudSphereFollowing = sphereFollowing.cloud();
}

void
SF_DownHillSimplex::adjustParameters(SF_ParamSpherefollowingAdvanced<SF_PointNormal>& params, size_t numClusters)
{
  size_t lastOpimizedIndex = static_cast<size_t>(std::max(0, static_cast<int>(numClusters) - 1));
  auto optimizationParams = params._sphereFollowingParams.m_optimizationParams[lastOpimizedIndex];
  std::vector<SF_ParamSpherefollowingAdvanced<SF_PointNormal>> testParameters = paramVector(params, optimizationParams);

  std::for_each(testParameters.begin(), testParameters.end(), [this](SF_ParamSpherefollowingAdvanced<SF_PointNormal>& paramsRef) {
    SF_SphereFollowing sphereFollowing;
    sphereFollowing.setParams(paramsRef);
    sphereFollowing.setCloud(paramsRef.m_cloudSphereFollowing);
    try {
      sphereFollowing.compute();
      paramsRef._qsm = sphereFollowing.getQSM();
      paramsRef._modelCloudError = sphereFollowing.error();
    } catch (...) {
      paramsRef._qsm = sphereFollowing.getQSM();
      paramsRef._modelCloudError = std::numeric_limits<float>::max();
    }
    m_params._stepProgress->fireComputation();
  });
  std::sort(testParameters.begin(),
            testParameters.end(),
            [](SF_ParamSpherefollowingAdvanced<SF_PointNormal> params1, SF_ParamSpherefollowingAdvanced<SF_PointNormal> params2) {
              return params1._modelCloudError < params2._modelCloudError;
            });
  for (SF_ParamSpherefollowingAdvanced<SF_PointNormal> param : testParameters) {
    if (param._modelCloudError < params._modelCloudError) {
      params = param;
      m_params = param;
    }
  }
}

std::vector<SF_ParamSpherefollowingAdvanced<SF_PointNormal>>
SF_DownHillSimplex::paramVector(const SF_ParamSpherefollowingAdvanced<SF_PointNormal>& params,
                                SF_SphereFollowingOptimizationParameters optimizationParams)
{
  std::vector<SF_ParamSpherefollowingAdvanced<SF_PointNormal>> paramVec;
  for (size_t k = 0; k < 3; ++k) {
    auto optimizationParamsK = optimizationParams;
    switch (k) {
      case 0:
        break;
      case 1:
        optimizationParamsK._epsilonSphere = (optimizationParamsK._epsilonSphere + 0.01) / 2.0;
        break;
      case 2:
        optimizationParamsK._epsilonSphere = 0.01;
        break;
      default:
        break;
    }
    for (size_t l = 0; l < 3; ++l) {
      auto optimizationParamsL = optimizationParamsK;
      switch (l) {
        case 0:
          break;
        case 1:
          optimizationParamsL._euclideanClusteringDistance = (optimizationParamsL._euclideanClusteringDistance + 0.03) / 2.0;
          break;
        case 2:
          optimizationParamsL._euclideanClusteringDistance = 0.03;
          break;
        default:
          break;
      }
      for (size_t m = 0; m < 3; ++m) {
        auto optimizationParamsM = optimizationParamsL;
        switch (m) {
          case 0:
            break;
          case 1:
            optimizationParamsM._sphereRadiusMultiplier = (optimizationParamsM._sphereRadiusMultiplier + 2.0) / 2.0;
            break;
          case 2:
            optimizationParamsM._sphereRadiusMultiplier = 2;
            break;
          default:
            break;
        }
        SF_ParamSpherefollowingAdvanced<SF_PointNormal> paramCpy = params;
        transferOptimizationParams(paramCpy, params.m_numClstrs, optimizationParamsM);
        paramVec.push_back(paramCpy);
      }
    }
  }
  return paramVec;
}

void
SF_DownHillSimplex::transferOptimizationParams(SF_ParamSpherefollowingAdvanced<SF_PointNormal>& params,
                                               size_t numClusters,
                                               SF_SphereFollowingOptimizationParameters optimParams)
{
  size_t firstCopyIndex = static_cast<size_t>(std::max(0, static_cast<int>(numClusters) - 1));
  for (size_t index = firstCopyIndex; index < params._sphereFollowingParams.m_optimizationParams.size(); ++index) {
    params._sphereFollowingParams.m_optimizationParams[index] = optimParams;
  }
}

void
SF_DownHillSimplex::compute()
{
  size_t size = m_params.m_numClstrs;
  m_params._modelCloudError = std::numeric_limits<float>::max();
  while (size > m_params._sphereFollowingParams.m_optimizationParams.size()) {
    m_params._sphereFollowingParams.m_optimizationParams.push_back(
      m_params._sphereFollowingParams.m_optimizationParams[m_params._sphereFollowingParams.m_optimizationParams.size() - 1]);
  }

  SF_ParamSpherefollowingAdvanced<SF_PointNormal> paramCpy = m_params;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudCpy = m_params.m_cloudSphereFollowing;
  for (size_t numClusters = 1; numClusters <= size; numClusters++) {
    paramCpy = m_params;
    paramCpy.m_numClstrs = numClusters;
    computeDHS(paramCpy, numClusters);
  }
  m_params.m_numClstrs = size;
  paramCpy = m_params;
  SF_ExtractFittedPoints<pcl::PointXYZINormal> extract(paramCpy._qsm, cloudCpy, paramCpy._distanceParams);
  extract.compute();
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud = extract.cloudFitted();
  paramCpy.m_cloudSphereFollowing = cloud;
  computeDHS(paramCpy, size);
}

double
downhillSimplex(const gsl_vector* v, void* params)
{
  std::uintptr_t* p = (std::uintptr_t*)params;
  SF_ParamSpherefollowingAdvanced<pcl::PointXYZINormal>* paramsBasic =
    reinterpret_cast<SF_ParamSpherefollowingAdvanced<pcl::PointXYZINormal>*>(p[0]);
  size_t index = 0;
  std::vector<SF_SphereFollowingOptimizationParameters> paramsOptim = paramsBasic->_sphereFollowingParams.m_optimizationParams;
  for (size_t i = 0; i < paramsBasic->m_numClstrs; i++) {
    paramsOptim[i]._epsilonSphere = gsl_vector_get(v, index++);
    paramsOptim[i]._euclideanClusteringDistance = gsl_vector_get(v, index++);
    paramsOptim[i]._sphereRadiusMultiplier = gsl_vector_get(v, index++);
  }
  paramsBasic->_sphereFollowingParams.m_optimizationParams = paramsOptim;
  SF_SphereFollowing sphereFollowing;
  sphereFollowing.setParams(*paramsBasic);
  sphereFollowing.setCloud(paramsBasic->m_cloudSphereFollowing);
  try {
    sphereFollowing.compute();
    paramsBasic->_qsm = sphereFollowing.getQSM();
    paramsBasic->_modelCloudError = sphereFollowing.error();
  } catch (...) {
    paramsBasic->_qsm = sphereFollowing.getQSM();
    paramsBasic->_modelCloudError = std::numeric_limits<float>::max();
  }
  double res = (paramsBasic->_modelCloudError);
  return res;
}
