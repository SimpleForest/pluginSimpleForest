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
#include "qsm/algorithm/sf_buildQSM.h"

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

SF_SphereFollowing::SF_SphereFollowing() {}

void SF_SphereFollowing::artificialTree() {
  pcl::ModelCoefficients circleA;
  circleA.values.push_back(0);
  circleA.values.push_back(0);
  circleA.values.push_back(0);
  circleA.values.push_back(1);

  pcl::ModelCoefficients circleB;
  circleB.values.push_back(0);
  circleB.values.push_back(0);
  circleB.values.push_back(1);
  circleB.values.push_back(1);

  pcl::ModelCoefficients circleC;
  circleC.values.push_back(0);
  circleC.values.push_back(0);
  circleC.values.push_back(2);
  circleC.values.push_back(1);

  SF_QSMDetectionCylinder cylinder(0, circleA);
  cylinder.addSecondCircle(circleB);
  SF_QSMDetectionCylinder cylinder2(1, circleB);
  cylinder2.addSecondCircle(circleC);
  m_cylinders.push_back(cylinder);
  m_cylinders.push_back(cylinder2);
  SF_BuildQSM qsmBuilder(m_cylinders, 0);
  m_qsm = qsmBuilder.getTree();
}

void SF_SphereFollowing::buildTree() {
  if (m_cylinders.size() > 2) {
    SF_BuildQSM qsmBuilder(m_cylinders, 0);
    m_qsm = qsmBuilder.getTree();
  } else {
    artificialTree();
  }
}

void SF_SphereFollowing::compute() {

  initialize();
  while (!m_priorityHeap.empty()) {
    Circle circleStruct = m_priorityHeap.top()._circle;
    m_priorityHeap.pop();
    pcl::PointIndices::Ptr surfaceIndicesVector = surfaceIndices(circleStruct);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr surface =
        extractCloud(surfaceIndicesVector);
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> surfaceClustersID =
        clusterByID(surface, circleStruct.m_clusterIndex);
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> surfaceClusters =
        clusterEuclidean(surfaceClustersID);
    processClusters(surfaceClusters, circleStruct);
  }
  buildTree();
}

float SF_SphereFollowing::error() {
  Sf_CloudToModelDistance cmd(m_qsm, m_cloud, m_params._distanceParams);
  return cmd.getAverageDistance();
}

pcl::PointIndices::Ptr SF_SphereFollowing::surfaceIndices(Circle &lastCircle) {
  const pcl::ModelCoefficients &lastCircleCoeff = lastCircle.m_circleCoeff;
  int index = lastCircle.m_clusterIndex;
  pcl::PointIndices::Ptr surface(new pcl::PointIndices);
  pcl::PointXYZINormal center;
  center.x = lastCircleCoeff.values[0];
  center.y = lastCircleCoeff.values[1];
  center.z = lastCircleCoeff.values[2];
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  float radius = m_params._sphereFollowingParams.m_optimizationParams[index]
                         ._sphereRadiusMultiplier *
                     lastCircleCoeff.values[3] +
                 m_params._sphereFollowingParams.m_optimizationParams[index]
                     ._epsilonSphere;
  if (m_octree->radiusSearch(center, radius, pointIdxRadiusSearch,
                             pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      const pcl::PointXYZINormal &point =
          m_cloud->points[pointIdxRadiusSearch[i]];
      if ((std::sqrt(pointRadiusSquaredDistance[i]) +
           2 * m_params._sphereFollowingParams.m_optimizationParams[index]
                   ._epsilonSphere) > radius) {
        surface->indices.push_back(pointIdxRadiusSearch[i]);
      }
      if (radius - m_params._sphereFollowingParams.m_optimizationParams[index]
                       ._epsilonSphere >
          std::sqrt(pointRadiusSquaredDistance[i])) {
        m_octree->deleteVoxelAtPoint(point);
      }
    }
  }
  return surface;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr
SF_SphereFollowing::extractCloud(pcl::PointIndices::Ptr indices) {
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::ExtractIndices<pcl::PointXYZINormal> extract;
  extract.setInputCloud(m_cloud);
  extract.setIndices(indices);
  extract.setNegative(false);
  extract.filter(*cloud);
  return cloud;
}

std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr>
SF_SphereFollowing::clusterByID(
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, size_t minID) {
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clusters;
  clusters.push_back(pcl::PointCloud<pcl::PointXYZINormal>::Ptr(
      new pcl::PointCloud<pcl::PointXYZINormal>()));
  size_t maxID = minID;
  std::for_each(cloud->points.begin(), cloud->points.end(),
                [&maxID, &minID, &clusters](const pcl::PointXYZINormal &point) {
                  size_t currentID = point.intensity;
                  while (maxID < currentID) {
                    clusters.push_back(
                        pcl::PointCloud<pcl::PointXYZINormal>::Ptr(
                            new pcl::PointCloud<pcl::PointXYZINormal>()));
                    maxID++;
                  }
                  clusters[currentID - minID]->push_back(point);
                });
  return clusters;
}

std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr>
SF_SphereFollowing::clusterEuclidean(
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> &clusters) {
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> newClusters;
  SF_ParamSpherefollowingBasic<pcl::PointXYZINormal> params(m_params);
  std::for_each(
      clusters.begin(), clusters.end(),
      [&newClusters,
       &params](pcl::PointCloud<pcl::PointXYZINormal>::Ptr cluster) {
        std::vector<pcl::PointIndices> clusterIndices;
        pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZINormal>);
        tree->setInputCloud(cluster);

        pcl::EuclideanClusterExtraction<pcl::PointXYZINormal> ec;
        ec.setClusterTolerance(
            params._sphereFollowingParams.m_optimizationParams[0]
                ._euclideanClusteringDistance);
        ec.setMinClusterSize(params._sphereFollowingParams._minPtsGeometry);
        ec.setMaxClusterSize(std::numeric_limits<int>::max());
        ec.setSearchMethod(tree);
        ec.setInputCloud(cluster);
        ec.extract(clusterIndices);
        for (std::vector<pcl::PointIndices>::const_iterator it =
                 clusterIndices.begin();
             it != clusterIndices.end(); ++it) {
          pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudCluster(
              new pcl::PointCloud<pcl::PointXYZINormal>);
          for (std::vector<int>::const_iterator pit = it->indices.begin();
               pit != it->indices.end(); ++pit)
            cloudCluster->points.push_back(cluster->points[*pit]); //*
          cloudCluster->width = cloudCluster->points.size();
          cloudCluster->height = 1;
          cloudCluster->is_dense = true;
          newClusters.push_back(cloudCluster);
        }
      });
  return newClusters;
}

void SF_SphereFollowing::processClusters(
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> &clusters,
    const Circle &lastCircle) {
  SF_ParamSpherefollowingBasic<pcl::PointXYZINormal> params(m_params);
  Eigen::Vector3f oldSphereCenter(lastCircle.m_circleCoeff.values[0],
                                  lastCircle.m_circleCoeff.values[1],
                                  lastCircle.m_circleCoeff.values[2]);
  if (lastCircle.m_firstSplit != oldSphereCenter && clusters.size() > 1) {
    std::sort(clusters.begin(), clusters.end(),
              [&lastCircle, &oldSphereCenter](
                  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud1,
                  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud2) {
                pcl::CentroidPoint<pcl::PointXYZINormal> centroid1;
                for (const pcl::PointXYZINormal &point : cloud1->points) {
                  centroid1.add(point);
                }
                pcl::PointXYZINormal centroidPoint1;
                centroid1.get(centroidPoint1);

                pcl::CentroidPoint<pcl::PointXYZINormal> centroid2;
                for (const pcl::PointXYZINormal &point : cloud2->points) {
                  centroid2.add(point);
                }
                pcl::PointXYZINormal centroidPoint2;
                centroid2.get(centroidPoint2);
                Eigen::Vector3f centerVec1(centroidPoint1.x, centroidPoint1.y,
                                           centroidPoint1.z);
                Eigen::Vector3f centerVec2(centroidPoint2.x, centroidPoint2.y,
                                           centroidPoint2.z);

                float angle1 = SF_Math<float>::getAngleBetweenDeg(
                    centerVec1 - oldSphereCenter,
                    oldSphereCenter - lastCircle.m_firstSplit);
                angle1 = std::min(angle1, 180.0f - angle1);
                float angle2 = SF_Math<float>::getAngleBetweenDeg(
                    centerVec2 - oldSphereCenter,
                    oldSphereCenter - lastCircle.m_firstSplit);
                angle2 = std::min(angle2, 180.0f - angle2);
                return angle1 < angle2;
              });
  }
  std::for_each(
      clusters.begin(), clusters.end(),
      [&lastCircle, &clusters, &params,
       this](pcl::PointCloud<pcl::PointXYZINormal>::Ptr cluster) {
        SF_Circle<pcl::PointXYZINormal> circleFit(
            cluster, params, static_cast<size_t>(cluster->points[0].intensity));
        pcl::ModelCoefficients coeff = circleFit.coeff();
        if (coeff.values.size() == 4) {
          Eigen::Vector3f diff =
              Eigen::Vector3f(lastCircle.m_circleCoeff.values[0],
                              lastCircle.m_circleCoeff.values[1],
                              lastCircle.m_circleCoeff.values[2]) -
              Eigen::Vector3f(coeff.values[0], coeff.values[1],
                              coeff.values[2]);
          float distance = diff.norm();
          if (distance > 0) {
            float dist = lastCircle.m_distance + distance;
            if (cluster != *clusters.begin()) {
              dist += params._sphereFollowingParams._heapDelta;
            }
            if (clusters.size() > 1) {
              if (cluster != *clusters.begin()) {
                pushbackQueue(circleFit.coeff(), dist,
                              cluster->points[0].intensity,
                              Eigen::Vector3f(coeff.values[0], coeff.values[1],
                                              coeff.values[2]));

              } else {
                pushbackQueue(circleFit.coeff(), dist,
                              cluster->points[0].intensity,
                              lastCircle.m_firstSplit);
              }
            } else {
              pushbackQueue(circleFit.coeff(), dist,
                            cluster->points[0].intensity,
                            lastCircle.m_firstSplit);
            }
            SF_QSMDetectionCylinder cyl(dist, lastCircle.m_circleCoeff);
            cyl.addSecondCircle(coeff);
            m_cylinders.push_back(cyl);
          }
        }
      });
}

void SF_SphereFollowing::initialize() {
  initializeCloud();
  initializeOctree();
  initializeHeap();
}

void SF_SphereFollowing::initializeCloud() {
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZINormal>());
  size_t index = 0;
  std::for_each(
      m_clusters.begin(), m_clusters.end(),
      [&index, &cloud](pcl::PointCloud<pcl::PointXYZINormal>::Ptr cluster) {
        std::for_each(cluster->points.begin(), cluster->points.end(),
                      [&index, &cloud](pcl::PointXYZINormal &point) {
                        point.intensity = index;
                        cloud->push_back(point);
                      });
        index++;
      });
  m_cloud = cloud;
}

void SF_SphereFollowing::initializeOctree() {
  m_octree.reset(
      new pcl::octree::OctreePointCloudSearch<pcl::PointXYZINormal>(0.015));
  m_octree->setInputCloud(m_cloud);
  m_octree->addPointsFromInputCloud();
}

void SF_SphereFollowing::initializeHeap() {
  float minZ = 0;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr lowestSliceCloud =
      lowestSlice(minZ);
  std::for_each(lowestSliceCloud->points.begin(),
                lowestSliceCloud->points.end(),
                [minZ](pcl::PointXYZINormal &point) { point.z = minZ; });
  SF_Circle<pcl::PointXYZINormal> circleFit(lowestSliceCloud, m_params, 0);
  pushbackQueue(circleFit.coeff(), 0.0f, 0,
                Eigen::Vector3f(circleFit.coeff().values[0],
                                circleFit.coeff().values[1],
                                circleFit.coeff().values[2]));
}

void SF_SphereFollowing::pushbackQueue(pcl::ModelCoefficients circleCoeff,
                                       float distance, int clusterID,
                                       Eigen::Vector3f firstSplit) {
  Circle circle(circleCoeff, distance, clusterID, firstSplit);
  HeapCircle::handle_type h = m_priorityHeap.push(circle);
  m_handle.push_back(h);
  (*h).handle = h;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr
SF_SphereFollowing::lowestSlice(float &minZ) {
  minZ = std::numeric_limits<float>::max();
  std::for_each(m_cloud->points.begin(), m_cloud->points.end(),
                [&minZ](const pcl::PointXYZINormal &point) {
                  if (point.z < minZ)
                    minZ = point.z;
                });
  float maxZ =
      minZ + m_params._sphereFollowingParams._heightInitializationSlice;

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr lowestSlice(
      new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::ConditionAnd<pcl::PointXYZINormal>::Ptr rangeCond(
      new pcl::ConditionAnd<pcl::PointXYZINormal>());
  rangeCond->addComparison(pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZINormal>(
          "z", pcl::ComparisonOps::GT, minZ)));
  rangeCond->addComparison(pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZINormal>(
          "z", pcl::ComparisonOps::LT, maxZ)));
  pcl::ConditionalRemoval<pcl::PointXYZINormal> condrem;
  condrem.setCondition(rangeCond);
  condrem.setInputCloud(m_cloud);
  condrem.setKeepOrganized(false);
  condrem.filter(*lowestSlice);
  return lowestSlice;
}

/******************************Getters and
 * Setters*********************************/
/******************************Getters and
 * Setters*********************************/
/******************************Getters and
 * Setters*********************************/

std::shared_ptr<SF_ModelQSM> SF_SphereFollowing::getQSM() { return m_qsm; }

void SF_SphereFollowing::setParams(
    const SF_ParamSpherefollowingBasic<pcl::PointXYZINormal> &params) {
  m_params = params;
}

void SF_SphereFollowing::setClusters(
    const std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> &clusters) {
  m_clusters = clusters;
}
