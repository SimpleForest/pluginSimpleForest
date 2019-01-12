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

#include <stdio.h>
#include <cstdint>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_min.h>
#include <gsl/gsl_multifit_nlinear.h>
#include <gsl/gsl_multifit_nlin.h>
#include <gsl/gsl_test.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_ieee_utils.h>
#include <vector>
#include <functional>

#include <gsl/gsl_math.h>
#include <gsl/gsl_errno.h>

SF_ParamSpherefollowingBasic<SF_PointNormal> SF_DownHillSimplex::params() const
{
    return m_params;
}

void SF_DownHillSimplex::setParams(const SF_ParamSpherefollowingBasic<SF_PointNormal> &params)
{
    m_params = params;
}

SF_DownHillSimplex::SF_DownHillSimplex()
{

}


void SF_DownHillSimplex::serializeParams(std::uintptr_t *par)
{
    std::uintptr_t p1 = reinterpret_cast<std::uintptr_t>(&(*m_params._cloudIn));
    std::uintptr_t p2 = reinterpret_cast<std::uintptr_t>(new float {m_params._distanceParams._inlierDistance});
    std::uintptr_t p3 = reinterpret_cast<std::uintptr_t>(new int {m_params._distanceParams._k});
    std::uintptr_t p4 = reinterpret_cast<std::uintptr_t>(new int {m_params._distanceParams._method});
    std::uintptr_t p5 = reinterpret_cast<std::uintptr_t>(new int {m_params._distanceParams._robustPercentage});
    std::uintptr_t p6 = reinterpret_cast<std::uintptr_t>(new int {m_params._sphereFollowingParams._fittingMethod});
    std::uintptr_t p7 = reinterpret_cast<std::uintptr_t>(new float {m_params._sphereFollowingParams._heapDelta});
    std::uintptr_t p8 = reinterpret_cast<std::uintptr_t>(new float {m_params._sphereFollowingParams._heightInitializationSlice});
    std::uintptr_t p9 = reinterpret_cast<std::uintptr_t>(new float {m_params._sphereFollowingParams._inlierDistance});
    std::uintptr_t p10 = reinterpret_cast<std::uintptr_t>(new float {m_params._sphereFollowingParams._minGlobalRadius});
    std::uintptr_t p11 = reinterpret_cast<std::uintptr_t>(new int {m_params._sphereFollowingParams._minPtsGeometry});
    std::uintptr_t p12 = reinterpret_cast<std::uintptr_t>(new int {m_params._sphereFollowingParams._RANSACIterations});
    std::uintptr_t p13 = reinterpret_cast<std::uintptr_t>(&(*m_params._tree));
    std::uintptr_t p14 = reinterpret_cast<std::uintptr_t>(new size_t {m_params.m_numClstrs});
    std::uintptr_t p[14] = {p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14};
    par = p;
}

void SF_DownHillSimplex::serializeVec(gsl_vector *x, double fac)
{
    size_t index = 0;
    for(size_t i = 0; i < m_params.m_numClstrs; i++) {
        gsl_vector_set (x, index++, static_cast<double>(m_params._sphereFollowingParams.m_optimizationParams[0]._epsilonSphere*fac));
        gsl_vector_set (x, index++, static_cast<double>(m_params._sphereFollowingParams.m_optimizationParams[0]._euclideanClusteringDistance*fac));
        gsl_vector_set (x, index++, static_cast<double>(m_params._sphereFollowingParams.m_optimizationParams[0]._medianRadiusMultiplier*fac));
        gsl_vector_set (x, index++, static_cast<double>(m_params._sphereFollowingParams.m_optimizationParams[0]._minRadius*fac));
        gsl_vector_set (x, index++, static_cast<double>(m_params._sphereFollowingParams.m_optimizationParams[0]._sphereRadiusMultiplier*fac));
    }
}

void
SF_DownHillSimplex::compute() {

    std::uintptr_t p1 = reinterpret_cast<std::uintptr_t>(&(*m_params._cloudIn));
    std::uintptr_t p2 = reinterpret_cast<std::uintptr_t>(new float {m_params._distanceParams._inlierDistance});
    std::uintptr_t p3 = reinterpret_cast<std::uintptr_t>(new int {m_params._distanceParams._k});
    std::uintptr_t p4 = reinterpret_cast<std::uintptr_t>(new int {m_params._distanceParams._method});
    std::uintptr_t p5 = reinterpret_cast<std::uintptr_t>(new int {m_params._distanceParams._robustPercentage});
    std::uintptr_t p6 = reinterpret_cast<std::uintptr_t>(new int {m_params._sphereFollowingParams._fittingMethod});
    std::uintptr_t p7 = reinterpret_cast<std::uintptr_t>(new float {m_params._sphereFollowingParams._heapDelta});
    std::uintptr_t p8 = reinterpret_cast<std::uintptr_t>(new float {m_params._sphereFollowingParams._heightInitializationSlice});
    std::uintptr_t p9 = reinterpret_cast<std::uintptr_t>(new float {m_params._sphereFollowingParams._inlierDistance});
    std::uintptr_t p10 = reinterpret_cast<std::uintptr_t>(new float {m_params._sphereFollowingParams._minGlobalRadius});
    std::uintptr_t p11 = reinterpret_cast<std::uintptr_t>(new int {m_params._sphereFollowingParams._minPtsGeometry});
    std::uintptr_t p12 = reinterpret_cast<std::uintptr_t>(new int {m_params._sphereFollowingParams._RANSACIterations});
    std::uintptr_t p13 = reinterpret_cast<std::uintptr_t>(&(*m_params._tree));
    std::uintptr_t p14 = reinterpret_cast<std::uintptr_t>(new size_t {m_params.m_numClstrs});
    std::uintptr_t par[14] = {p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14};
    {
        std::ofstream myfile;
        std::string str = "/home/drsnuggles/Documents/param/param";
        str.append(std::to_string(m_params.m_numClstrs));
        str.append(".csv");
        myfile.open(str);
        myfile << m_params._distanceParams._inlierDistance << std::endl;
        myfile << m_params._distanceParams._k << std::endl;
        myfile << m_params._distanceParams._method << std::endl;
        myfile << m_params._distanceParams._robustPercentage << std::endl;
        myfile << m_params._sphereFollowingParams._fittingMethod << std::endl;
        myfile << m_params._sphereFollowingParams._heapDelta << std::endl;
        myfile << m_params._sphereFollowingParams._heightInitializationSlice << std::endl;
        myfile << m_params._sphereFollowingParams._inlierDistance << std::endl;
        myfile << m_params._sphereFollowingParams._minGlobalRadius << std::endl;
        myfile << m_params._sphereFollowingParams._minPtsGeometry << std::endl;
        myfile << m_params._sphereFollowingParams._RANSACIterations << std::endl;
        myfile << m_params.m_numClstrs << std::endl;
        myfile.close();
    }

    {
        std::ofstream myfile;
        std::string str = "/home/drsnuggles/Documents/param/all";
        str.append(std::to_string(m_params.m_numClstrs));
        str.append(".csv");
        myfile.open(str);
        myfile << m_params._sphereFollowingParams.m_optimizationParams[0]._epsilonSphere << std::endl;
        myfile << m_params._sphereFollowingParams.m_optimizationParams[0]._euclideanClusteringDistance << std::endl;
        myfile << m_params._sphereFollowingParams.m_optimizationParams[0]._medianRadiusMultiplier << std::endl;
        myfile << m_params._sphereFollowingParams.m_optimizationParams[0]._minRadius << std::endl;
        myfile << m_params._sphereFollowingParams.m_optimizationParams[0]._sphereRadiusMultiplier << std::endl;
        myfile << m_params.m_numClstrs << std::endl;
        myfile.close();
    }

    const gsl_multimin_fminimizer_type *T =
            gsl_multimin_fminimizer_nmsimplex2;
    gsl_multimin_fminimizer *s = NULL;
    gsl_vector *ss, *x;
    x = gsl_vector_alloc (m_params.m_numClstrs*5);
    ss = gsl_vector_alloc (m_params.m_numClstrs*5);
    gsl_multimin_function minex_func;
    size_t iter = 0;
    int status;
    double size;
    serializeVec(x, 1.0);
    serializeVec(ss, 0.2);
    minex_func.n = m_params.m_numClstrs*5;
    minex_func.f = downhillSimplex;
    minex_func.params = par;
    s = gsl_multimin_fminimizer_alloc (T, m_params.m_numClstrs*5);
    gsl_multimin_fminimizer_set (s, &minex_func, x, ss);
    do {
        iter++;
        status = gsl_multimin_fminimizer_iterate(s);

        if (status)
            break;
        size = gsl_multimin_fminimizer_size (s);
        status = gsl_multimin_test_size (size, 5*1e-4);
        if (status == GSL_SUCCESS)  {
            printf ("converged to minimum at\n");
        }
        double error = s->fval;
        if(m_params._modelCloudError > error) {
           m_params._modelCloudError = error;
           size_t index = 0;
           std::vector<SF_SphereFollowingOptimizationParameters> paramVec;
           SF_SphereFollowingOptimizationParameters paramsOptim;
           gsl_vector *v = s->x;
           for(size_t i = 0; i < m_params.m_numClstrs; i++) {
               paramsOptim._epsilonSphere = gsl_vector_get(v, index++);
               paramsOptim._euclideanClusteringDistance = gsl_vector_get(v, index++);
               paramsOptim._medianRadiusMultiplier = gsl_vector_get(v, index++);
               paramsOptim._minRadius = gsl_vector_get(v, index++);
               paramsOptim._sphereRadiusMultiplier = gsl_vector_get(v, index++);
               paramVec.push_back(paramsOptim);
           }
           m_params._sphereFollowingParams.m_optimizationParams = paramVec;

        }
        printf ("%5d %10.6e %10.6e f() = %7.6f size = %.6f\n",
                iter,
                gsl_vector_get (s->x, 0),
                gsl_vector_get (s->x, 1),
                error, size);

    }
    while (status == GSL_CONTINUE && iter < 1000);
    gsl_vector_free(x);
    gsl_vector_free(ss);
    gsl_multimin_fminimizer_free (s);

    SF_CloudToModelDistanceParameters paramsDist = m_params._distanceParams;
    paramsDist._inlierDistance = 0.05;
    paramsDist._method = SF_CLoudToModelDistanceMethod::GROWTHDISTANCE;
    SF_ClusterCloudByQSM cloudQSM(paramsDist, m_params._tree, m_params._cloudIn, m_params.m_numClstrs);
    cloudQSM.compute();

    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> m_clusters = cloudQSM.clusters();

    SF_SphereFollowing sphereFollowing;
    sphereFollowing.setParams(m_params);
    sphereFollowing.setClusters(m_clusters);
    sphereFollowing.compute();
    m_params._tree = sphereFollowing.getQSM();
    m_params._cloudIn = sphereFollowing.cloud();

    {
        std::ofstream myfile;
        std::string str = "/home/drsnuggles/Documents/param/allSub";
        str.append(std::to_string(m_params.m_numClstrs));
        str.append(".csv");
        myfile.open(str);
        for(size_t i  = 0; i < m_params.m_numClstrs; i++) {
            myfile << m_params._sphereFollowingParams.m_optimizationParams[i]._epsilonSphere << std::endl;
            myfile << m_params._sphereFollowingParams.m_optimizationParams[i]._euclideanClusteringDistance << std::endl;
            myfile << m_params._sphereFollowingParams.m_optimizationParams[i]._medianRadiusMultiplier << std::endl;
            myfile << m_params._sphereFollowingParams.m_optimizationParams[i]._minRadius << std::endl;
            myfile << m_params._sphereFollowingParams.m_optimizationParams[i]._sphereRadiusMultiplier << std::endl;
        }
        myfile.close();
    }
}

double downhillSimplex(const gsl_vector *v, void *params)
{
    SF_ParamSpherefollowingBasic<pcl::PointXYZINormal> paramsBasic;
    std::uintptr_t *p = (std::uintptr_t *)params;
    paramsBasic._cloudIn.reset(reinterpret_cast<pcl::PointCloud<pcl::PointXYZINormal> *>(p[0]), [](pcl::PointCloud<pcl::PointXYZINormal> *){});
    paramsBasic._distanceParams._inlierDistance = *(reinterpret_cast<float *>(p[1]));
    paramsBasic._distanceParams._k = *(reinterpret_cast<int *>(p[2]));
    paramsBasic._distanceParams._method = *(reinterpret_cast<int *>(p[3]));
    paramsBasic._distanceParams._robustPercentage = *(reinterpret_cast<int *>(p[4]));
    paramsBasic._sphereFollowingParams._fittingMethod = *(reinterpret_cast<int *>(p[5]));
    paramsBasic._sphereFollowingParams._heapDelta = *(reinterpret_cast<float *>(p[6]));
    paramsBasic._sphereFollowingParams._heightInitializationSlice = *(reinterpret_cast<float *>(p[7]));
    paramsBasic._sphereFollowingParams._inlierDistance = *(reinterpret_cast<float *>(p[8]));
    paramsBasic._sphereFollowingParams._minGlobalRadius = *(reinterpret_cast<float *>(p[9]));
    paramsBasic._sphereFollowingParams._minPtsGeometry = *(reinterpret_cast<int *>(p[10]));
    paramsBasic._sphereFollowingParams._RANSACIterations = *(reinterpret_cast<int *>(p[11]));
    paramsBasic._tree.reset(reinterpret_cast<SF_ModelQSM *>(p[12]), [](SF_ModelQSM *){});
    paramsBasic.m_numClstrs = *(reinterpret_cast<int *>(p[13]));

    SF_CloudToModelDistanceParameters paramsDist = paramsBasic._distanceParams;
    paramsDist._inlierDistance = 0.05;
    paramsDist._method = SF_CLoudToModelDistanceMethod::GROWTHDISTANCE;
    SF_ClusterCloudByQSM cloudQSM(paramsDist, paramsBasic._tree, paramsBasic._cloudIn, paramsBasic.m_numClstrs);
    cloudQSM.compute();

    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> m_clusters = cloudQSM.clusters();

    std::vector<SF_SphereFollowingOptimizationParameters> paramVec;
    size_t index = 0;
    SF_SphereFollowingOptimizationParameters paramsOptim;
    for(size_t i = 0; i < paramsBasic.m_numClstrs; i++) {
        paramsOptim._epsilonSphere = gsl_vector_get(v, index++);
        paramsOptim._euclideanClusteringDistance = gsl_vector_get(v, index++);
        paramsOptim._medianRadiusMultiplier = gsl_vector_get(v, index++);
        paramsOptim._minRadius = gsl_vector_get(v, index++);
        paramsOptim._sphereRadiusMultiplier = gsl_vector_get(v, index++);
        paramVec.push_back(paramsOptim);
    }

    paramsBasic._sphereFollowingParams.m_optimizationParams = paramVec;
    SF_SphereFollowing sphereFollowing;
    sphereFollowing.setParams(paramsBasic);
    sphereFollowing.setClusters(m_clusters);
    try {
        sphereFollowing.compute();
        paramsBasic._tree = sphereFollowing.getQSM();
        paramsBasic._modelCloudError = sphereFollowing.error();
    } catch (...) {
        paramsBasic._tree = sphereFollowing.getQSM();
        paramsBasic._modelCloudError = std::numeric_limits<float>::max();
    }
    double res =(paramsBasic._modelCloudError);
    return res;

}
