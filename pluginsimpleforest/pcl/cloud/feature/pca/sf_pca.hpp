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
#ifndef SF_PCA_HPP
#define SF_PCA_HPP

#include "sf_pca.h"
template<typename PointType>
SF_PCA<PointType>::SF_PCA(typename pcl::PointCloud<PointType>::Ptr cloud_in) : SF_AbstractCloud<PointType>(cloud_in)
{
  pcaValues.resize(cloud_in->points.size());
}

template<typename PointType>
void
SF_PCA<PointType>::setParameters(float range, bool centerZero, bool useRange)
{
  _range = range;
  _centerZero = centerZero;
  _k = 5;
  _useRange = useRange;
}

template<typename PointType>
void
SF_PCA<PointType>::setParameters(int k, bool centerZero)
{
  _k = k;
  _centerZero = centerZero;
  _k = 0.03;
  _useRange = false;
}

template<typename PointType>
void
SF_PCA<PointType>::extractNeighbors(typename pcl::KdTree<PointType>::Ptr kdTree,
                                    PointType p,
                                    typename pcl::PointCloud<PointType>::Ptr neighborhood)
{
  if (_useRange) {
    SF_AbstractCloud<PointType>::extractNeighborsByRange(kdTree, p, neighborhood, _range);
  } else {
    SF_AbstractCloud<PointType>::extractNeighborsByKnn(kdTree, p, neighborhood, _k);
  }
}

template<typename PointType>
SF_PCAValues
SF_PCA<PointType>::computeFeaturesFromNeighbors(typename pcl::PointCloud<PointType>::Ptr neighborhood,
                                                const Eigen::Vector4f& xyz_centroid)
{
  Eigen::Matrix3f covariance_matrix;
  pcl::computeCovarianceMatrix(*neighborhood, xyz_centroid, covariance_matrix);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(covariance_matrix);
  SF_PCAValues ppca;
  ppca.lambdas = eig.eigenvalues();
  ppca.vectors = eig.eigenvectors();
  return ppca;
}

template<typename PointType>
void
SF_PCA<PointType>::computeFeaturesForPoint(const PointType& p, typename pcl::KdTree<PointType>::Ptr kdTree, int index)
{
  Eigen::Vector4f xyz_centroid;
  typename pcl::PointCloud<PointType>::Ptr neighborhood(new typename pcl::PointCloud<PointType>);
  extractNeighbors(kdTree, p, neighborhood);
  pcl::compute3DCentroid(*neighborhood, xyz_centroid);
  Eigen::Vector4f origin(0, 0, 0, 1);
  SF_PCAValues ppca = computeFeaturesFromNeighbors(neighborhood, (_centerZero ? origin : xyz_centroid));
  pcaValues[index] = ppca;
}

template<typename PointType>
void
SF_PCA<PointType>::computeFeatures()
{
  if (pcaValues.size() != SF_PCA<PointType>::_cloudIn->points.size()) {
    pcaValues.resize(SF_PCA<PointType>::_cloudIn->points.size());
  }

  typename pcl::KdTreeFLANN<PointType>::Ptr kd_tree(new typename pcl::KdTreeFLANN<PointType>());
  kd_tree->setInputCloud(SF_PCA<PointType>::_cloudIn);
  for (size_t i = 0; i < SF_PCA<PointType>::_cloudIn->points.size(); i++) {
    PointType p = SF_PCA<PointType>::_cloudIn->points[i];
    computeFeaturesForPoint(p, kd_tree, i);
  }
}

template<typename PointType>
std::vector<SF_PCAValues>
SF_PCA<PointType>::getPcaValues() const
{
  return pcaValues;
}

#endif // SF_PCA_HPP
