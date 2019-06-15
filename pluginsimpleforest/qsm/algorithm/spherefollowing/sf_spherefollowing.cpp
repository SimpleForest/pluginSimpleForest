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

#include "sf_spherefollowing.h"

#include "qsm/algorithm/distance/sf_cloudToModelDistance.h"
#include "qsm/algorithm/geometry/circle/sf_circle.h"
#include "qsm/algorithm/postprocessing/sf_removefalseconnections.h"
#include "qsm/build/sf_buildQSM.h"

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <tests/factory/sf_qsmfactory.h>

SF_SphereFollowing::SF_SphereFollowing() {}

void
SF_SphereFollowing::artificialTree()
{
  m_qsm = SF_QSMFactory::qsm(SORTTYPE::LARGEST_GROWTHLENGTH_SECOND);
}

void
SF_SphereFollowing::buildTree()
{
  if (m_cylinders.size() > 2) {
    SF_BuildQSM qsmBuilder(m_cylinders, 0);
    m_qsm = qsmBuilder.getTree();
  } else {
    throw std::runtime_error("SphereFollowing did not detect cylinders.");
    failedComputation = true;
  }
}

void
SF_SphereFollowing::compute()
{
  if (m_cloud->empty()) {
    failedComputation = true;
    throw std::runtime_error("SphereFollowing received an empty cloud.");
  }
  initialize();
  while (!m_map.empty()) {
    Circle circleStruct = (*m_map.begin()).second;
    m_map.erase(m_map.begin());
    pcl::PointIndices::Ptr surfaceIndicesVector = surfaceIndices(circleStruct);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr surface = extractCloud(surfaceIndicesVector);
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> surfaceClusters = clusterEuclidean(surface, circleStruct.m_clusterIndex);
    processClusters(surfaceClusters, circleStruct);
  }
  buildTree();
  SF_RemoveFalseConnections rfc;
  rfc.compute(m_qsm, m_params.m_overLapAngle);
}

float
SF_SphereFollowing::error()
{
  if (failedComputation) {
    return std::numeric_limits<float>::max();
  }
  Sf_CloudToModelDistance<pcl::PointXYZINormal> cmd(m_qsm, m_cloud, m_params._distanceParams);
  return cmd.getAverageDistance();
}

pcl::PointIndices::Ptr
SF_SphereFollowing::surfaceIndices(Circle& lastCircle)
{
  const pcl::ModelCoefficients& lastCircleCoeff = lastCircle.m_circleCoeff;
  int index = lastCircle.m_clusterIndex;
  pcl::PointIndices::Ptr surface(new pcl::PointIndices);
  pcl::PointXYZINormal center;
  center.x = lastCircleCoeff.values[0];
  center.y = lastCircleCoeff.values[1];
  center.z = lastCircleCoeff.values[2];
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = std::max(static_cast<double>(m_params._sphereFollowingParams._minGlobalRadius),
                          m_params._sphereFollowingParams.m_optimizationParams[index]._sphereRadiusMultiplier *
                            lastCircleCoeff.values[3]);
  float maxRadius = radius + m_params._sphereFollowingParams.m_optimizationParams[index]._epsilonSphere;
  float minRadius = radius - m_params._sphereFollowingParams.m_optimizationParams[index]._epsilonSphere;

  if (m_octree->radiusSearch(center, maxRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      auto pointIndex = pointIdxRadiusSearch[i];
      const pcl::PointXYZINormal& point = m_cloud->points[pointIndex];
      if (std::sqrt(pointRadiusSquaredDistance[i]) > minRadius) {
        surface->indices.push_back(pointIndex);
      }
      m_octree->deleteVoxelAtPoint(point);
    }
  }
  return surface;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr
SF_SphereFollowing::extractCloud(pcl::PointIndices::Ptr indices)
{
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::ExtractIndices<pcl::PointXYZINormal> extract;
  extract.setInputCloud(m_cloud);
  extract.setIndices(indices);
  extract.setNegative(false);
  extract.filter(*cloud);
  return cloud;
}

std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr>
SF_SphereFollowing::clusterEuclidean(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, size_t minIndex)
{
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clusters;
  std::vector<pcl::PointIndices> clusterIndices;
  pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZINormal>);
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZINormal> ec;
  ec.setClusterTolerance(m_params._sphereFollowingParams.m_optimizationParams[0]._euclideanClusteringDistance);
  ec.setMinClusterSize(m_params._sphereFollowingParams._minPtsGeometry);
  ec.setMaxClusterSize(std::numeric_limits<int>::max());
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusterIndices);
  for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZINormal>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloudCluster->points.push_back(cloud->points[*pit]);
    if (cloudCluster->points.size() >= static_cast<size_t>(m_params._sphereFollowingParams._minPtsGeometry)) {
      cloudCluster->width = cloudCluster->points.size();
      cloudCluster->height = 1;
      cloudCluster->is_dense = true;
      size_t minClusterIndex = std::numeric_limits<size_t>::max();
      for (size_t i = 0; i < cloudCluster->points.size(); i++) {
        if (cloudCluster->points[i].intensity < minClusterIndex) {
          minClusterIndex = cloudCluster->points[i].intensity;
        }
      }
      cloudCluster->points[0].intensity = minClusterIndex;
      clusters.push_back(cloudCluster);
    }
  }
  return clusters;
}

Eigen::Vector3f
SF_SphereFollowing::getCentroid(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
{
  pcl::CentroidPoint<pcl::PointXYZINormal> centroid;
  for (const pcl::PointXYZINormal& point : cloud->points) {
    centroid.add(point);
  }
  pcl::PointXYZINormal centroidPoint;
  centroid.get(centroidPoint);
  return Eigen::Vector3f(centroidPoint.x, centroidPoint.y, centroidPoint.z);
}

float
SF_SphereFollowing::getDistance(const pcl::ModelCoefficients& circle, const Circle& lastCircle)
{
  Eigen::Vector3f diff =
    Eigen::Vector3f(lastCircle.m_circleCoeff.values[0], lastCircle.m_circleCoeff.values[1], lastCircle.m_circleCoeff.values[2]) -
    Eigen::Vector3f(circle.values[0], circle.values[1], circle.values[2]);
  return diff.norm();
}

void
SF_SphereFollowing::processClusters(std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr>& clusters, const Circle& lastCircle)
{
  SF_ParamSpherefollowingBasic<pcl::PointXYZINormal> params(m_params);
  Eigen::Vector3f oldSphereCenter(
    lastCircle.m_circleCoeff.values[0], lastCircle.m_circleCoeff.values[1], lastCircle.m_circleCoeff.values[2]);
  if (lastCircle.m_firstSplit != oldSphereCenter && clusters.size() > 1) {
    std::sort(
      clusters.begin(),
      clusters.end(),
      [&lastCircle, &oldSphereCenter, this](pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud1,
                                            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud2) {
        Eigen::Vector3f centroid1 = getCentroid(cloud1);
        Eigen::Vector3f centroid2 = getCentroid(cloud2);
        float angle1 = SF_Math<float>::getAngleBetweenDegf(centroid1 - oldSphereCenter, oldSphereCenter - lastCircle.m_firstSplit);
        float angle2 = SF_Math<float>::getAngleBetweenDegf(centroid2 - oldSphereCenter, oldSphereCenter - lastCircle.m_firstSplit);
        return angle1 < angle2;
      });
  }
  std::for_each(
    clusters.begin(), clusters.end(), [&lastCircle, &clusters, &params, this](pcl::PointCloud<pcl::PointXYZINormal>::Ptr cluster) {
      auto intensity = cluster->points[0].intensity;
      SF_Circle<pcl::PointXYZINormal> circleFit(cluster, params, static_cast<size_t>(intensity));
      pcl::ModelCoefficients coeff = circleFit.coeff();
      if (coeff.values.size() == 4) {
        float clusterDistance = std::round(intensity - lastCircle.m_clusterIndex);
        float heapDistance;
        if (clusterDistance > 1)
        {
            heapDistance = lastCircle.m_distance + getDistance(coeff, lastCircle) + std::pow(10.0, clusterDistance);
        }
        else
        {
            heapDistance = lastCircle.m_distance + getDistance(coeff, lastCircle);
        }
        if (cluster != *clusters.begin()) {
          heapDistance += params._sphereFollowingParams._heapDelta;
        }
        if (clusters.size() > 1) {
          if (cluster != *clusters.begin()) {
            pushbackQueue(
              circleFit.coeff(), heapDistance, intensity, Eigen::Vector3f(coeff.values[0], coeff.values[1], coeff.values[2]));
          } else {
            pushbackQueue(circleFit.coeff(), heapDistance, intensity, lastCircle.m_firstSplit);
          }
        } else {
          pushbackQueue(circleFit.coeff(), heapDistance, intensity, lastCircle.m_firstSplit);
        }
        SF_QSMDetectionCylinder cyl(heapDistance, lastCircle.m_circleCoeff);
        cyl.addSecondCircle(coeff);
        m_cylinders.push_back(cyl);
      }
    });
}

void
SF_SphereFollowing::initialize()
{
  initializeOctree();
  initializeHeap();
}

void
SF_SphereFollowing::initializeOctree()
{
  m_octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZINormal>(m_params._voxelSize));
  m_octree->setInputCloud(m_cloud);
  m_octree->addPointsFromInputCloud();
}

void
SF_SphereFollowing::initializeHeap()
{
  float minZ = 0;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr lowestSliceCloud = lowestSlice(minZ);
  std::for_each(
    lowestSliceCloud->points.begin(), lowestSliceCloud->points.end(), [minZ](pcl::PointXYZINormal& point) { point.z = minZ; });
  SF_Circle<pcl::PointXYZINormal> circleFit(lowestSliceCloud, m_params, 0);
  pushbackQueue(circleFit.coeff(),
                0.0f,
                0,
                Eigen::Vector3f(circleFit.coeff().values[0], circleFit.coeff().values[1], circleFit.coeff().values[2]));
}

void
SF_SphereFollowing::pushbackQueue(pcl::ModelCoefficients circleCoeff, float distance, int clusterID, Eigen::Vector3f firstSplit)
{
  m_map[distance] = Circle(circleCoeff, distance, clusterID, firstSplit);
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr
SF_SphereFollowing::lowestSlice(float& minZ)
{
  minZ = std::numeric_limits<float>::max();
  std::for_each(m_cloud->points.begin(), m_cloud->points.end(), [&minZ](const pcl::PointXYZINormal& point) {
    if (point.z < minZ)
      minZ = point.z;
  });
  float maxZ = minZ + m_params._sphereFollowingParams._heightInitializationSlice;

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr lowestSlice(new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::ConditionAnd<pcl::PointXYZINormal>::Ptr rangeCond(new pcl::ConditionAnd<pcl::PointXYZINormal>());
  rangeCond->addComparison(pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr(
    new pcl::FieldComparison<pcl::PointXYZINormal>("z", pcl::ComparisonOps::GT, minZ)));
  rangeCond->addComparison(pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr(
    new pcl::FieldComparison<pcl::PointXYZINormal>("z", pcl::ComparisonOps::LT, maxZ)));
  pcl::ConditionalRemoval<pcl::PointXYZINormal> condrem;
  condrem.setCondition(rangeCond);
  condrem.setInputCloud(m_cloud);
  condrem.setKeepOrganized(false);
  condrem.filter(*lowestSlice);
  return lowestSlice;
}

/********************************** Getters and Setters *******************************************/
/********************************** Getters and Setters *******************************************/
/********************************** Getters and Setters *******************************************/

std::shared_ptr<SF_ModelQSM>
SF_SphereFollowing::getQSM()
{
  return m_qsm;
}

void
SF_SphereFollowing::setParams(const SF_ParamSpherefollowingBasic<pcl::PointXYZINormal>& params)
{
  m_params = params;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr
SF_SphereFollowing::cloud() const
{
  return m_cloud;
}

void
SF_SphereFollowing::setCloud(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud)
{
  m_cloud = cloud;
}
