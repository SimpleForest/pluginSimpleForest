/****************************************************************************

 Copyright (C) 2017-2019 Dr. Jan Hackenberg, free software developer
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


#include "sf_dijkstraLightRecursive.h"

#include "qsm/algorithm/distance/sf_extractFittedPoints.h"
#include "qsm/model/sf_modelCylinderBuildingbrick.h"

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pcl/cloud/segmentation/dijkstra/sf_dijkstra.h"
#include "qsm/algorithm/sf_QSMCylinder.h"
#include "qsm/build/sf_buildQSM.h"

void
SF_DijkstraLightRecursive::setParams(const SF_ParamSpherefollowingRecursive<SF_PointNormal>& params)
{
  m_params = params;
}

void
SF_DijkstraLightRecursive::setCloud(const SF_CloudNormal::Ptr& cloud)
{
  m_cloud = cloud;
}

void
SF_DijkstraLightRecursive::setQsm(const std::shared_ptr<SF_ModelQSM>& qsm)
{
  m_qsm = qsm;  m_bricks = m_qsm->getBuildingBricks();
  initializeKdTree();
}

std::shared_ptr<SF_ModelQSM>
SF_DijkstraLightRecursive::getQsm() const
{
  return m_qsm;
}

SF_CloudNormal::Ptr
SF_DijkstraLightRecursive::extractUnfittedPoints()
{
  SF_CloudToModelDistanceParameters distanceParams;
  distanceParams._method = SF_CLoudToModelDistanceMethod::FIRSTMOMENTUMORDER;
  distanceParams._inlierDistance = m_params.m_unfittedDistance;
  distanceParams._k = 9;
  SF_ExtractFittedPoints<pcl::PointXYZINormal> extract(m_qsm, m_cloud, distanceParams);
  extract.compute();
  return extract.cloudUnfitted();
}

std::vector<SF_CloudNormal::Ptr>
SF_DijkstraLightRecursive::clusters(SF_CloudNormal::Ptr cloud)
{
  size_t minSize = static_cast<size_t>(m_params.m_minPercentage * m_cloud->points.size());
  size_t maxSize = static_cast<size_t>(m_params.m_maxPercentage * m_cloud->points.size());
  pcl::search::KdTree<SF_PointNormal>::Ptr tree(new pcl::search::KdTree<SF_PointNormal>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<SF_PointNormal> ec;
  ec.setClusterTolerance(m_params.m_clusteringDistance);
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setMaxClusterSize(m_cloud->points.size());
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusterIndices);
  std::vector<SF_CloudNormal::Ptr> result;
  for (pcl::PointIndices clusterindices : clusterIndices) {
    SF_CloudNormal::Ptr cloudCluster(new SF_CloudNormal);
    for (std::vector<int>::const_iterator pit = clusterindices.indices.begin(); pit != clusterindices.indices.end(); ++pit)
      cloudCluster->points.push_back(cloud->points[*pit]);
    cloudCluster->width = cloudCluster->points.size();
    cloudCluster->height = 1;
    cloudCluster->is_dense = true;
    result.push_back(cloudCluster);
  }
  return result;
}

pcl::ModelCoefficients::Ptr SF_DijkstraLightRecursive::coefficients(SF_PointNormal &p)
{
    pcl::ModelCoefficients::Ptr coeff2 (new pcl::ModelCoefficients);
    coeff2->values.push_back(p.x);
    coeff2->values.push_back(p.y);
    coeff2->values.push_back(p.z);
    coeff2->values.push_back(0.1f);
    return coeff2;
}

std::vector<SF_CloudNormal::Ptr>
SF_DijkstraLightRecursive::sortClusters(const std::vector<SF_CloudNormal::Ptr>& clusters)
{
  return clusters;
}

void
SF_DijkstraLightRecursive::compute()
{
  SF_CloudNormal::Ptr unfitted = extractUnfittedPoints();
  std::vector<SF_CloudNormal::Ptr> clouds = clusters(unfitted);
  std::vector<SF_CloudNormal::Ptr> sortedClusters = sortClusters(clouds);
  processClusters(sortedClusters);
}

void
SF_DijkstraLightRecursive::processClusters(const std::vector<SF_CloudNormal::Ptr>& clusters)
{
  for (SF_CloudNormal::Ptr cluster : clusters) {
      SF_CloudNormal::Ptr downScaled (new SF_CloudNormal);
      pcl::VoxelGrid<SF_PointNormal> sor;
      sor.setInputCloud(cluster);
      {
        sor.setLeafSize(m_params.m_clusterDownScale, m_params.m_clusterDownScale, m_params.m_clusterDownScale);
      }
      sor.filter(*downScaled);
      cluster = downScaled;


    size_t minIndex = 0;
    size_t maxIndex = 0;
    Eigen::Vector3d  closestQSMPoint;
    getMinMax(minIndex, maxIndex, cluster, closestQSMPoint);


    SF_CloudNormal::Ptr seed(new SF_CloudNormal);
    seed->push_back(cluster->points.at(minIndex));
    SF_Dijkstra dijkstra(cluster, seed, m_params.m_clusterDownScale* 2, true);
    std::vector<int> parentIndices = dijkstra.getParentIndices();

    std::vector<SF_QSMDetectionCylinder> cylinders;
    pcl::ModelCoefficients::Ptr coeffStart (new pcl::ModelCoefficients);
    coeffStart->values.push_back(closestQSMPoint[0]);
    coeffStart->values.push_back(closestQSMPoint[1]);
    coeffStart->values.push_back(closestQSMPoint[2]);
    coeffStart->values.push_back(0.01f);
    pcl::ModelCoefficients::Ptr coeffEnd = coefficients(cluster->points.at(minIndex));
    SF_QSMDetectionCylinder cylinder(0.f, coeffStart, coeffEnd);
    cylinders.push_back(cylinder);
    int index = 0;
    for(auto parentIndex: parentIndices)
    {
        if(parentIndex > -1)
        {
            pcl::ModelCoefficients::Ptr coeffStart= coefficients(cluster->points.at(parentIndex));
            pcl::ModelCoefficients::Ptr coeffEnd = coefficients(cluster->points.at(index));
            SF_QSMDetectionCylinder cylinder(0, coeffStart, coeffEnd);
            cylinders.push_back(cylinder);
        }
        index++;
    }
    SF_BuildQSM builder(cylinders, 1000);
    auto childQSM = builder.getTree();

    auto bricks = childQSM->getBuildingBricks();
    for(auto brick : bricks)
    {
        if(brick != bricks.front())
        {
            brick->setFittingType(FittingType::DIJKSTRALIGHT);
        }
    }
    connectQSM(childQSM);
  }
}

void
SF_DijkstraLightRecursive::connectQSM(std::shared_ptr<SF_ModelQSM> childQSM)
{
  auto startChild = childQSM->getRootSegment()->getBuildingBricks().front()->getStart();
  auto start = startChild;
  auto bricks = m_qsm->getBuildingBricks();
  float bestDistance = std::numeric_limits<float>::max();
  auto segment = m_qsm->getRootSegment();
  for (auto brick : bricks) {
    auto distance = brick->getDistance(startChild);
    if (bestDistance > distance) {
      bestDistance = distance;
      start = brick->getEnd();
      segment = brick->getSegment();
    }
  }
  auto allChildSegments = childQSM->getSegments();
  for (auto segmentChild : allChildSegments) {
    segmentChild->setTree(m_qsm);
  }
  if (start == startChild) {
    std::cout << "SF_SpherefollowingRecursive::connectQSM(... childQSM) no cylinder to connect found" << std::endl;
    m_qsm->getRootSegment()->addChild(childQSM->getRootSegment());
  } else {
    segment->addChild(childQSM->getRootSegment());
  }
  setQsm(m_qsm); // recall to reconstruct kdtree
}

void
SF_DijkstraLightRecursive::getMinMax(size_t& min, size_t& max, SF_CloudNormal::Ptr cluster, Eigen::Vector3d &closestQSM)
{
  double minDistance = std::numeric_limits<double>::max();
  double maxDistance = 0;
  size_t index = 0;
  for (SF_PointNormal point : cluster->points) {
    int K = 1;
     std::vector<int> pointIdxNKNSearch(K);
     std::vector<float> pointNKNSquaredDistance(K);
     if ( _kdtreeQSM->nearestKSearch (point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
     {
         auto distance = std::sqrt(pointNKNSquaredDistance.front());
         if (distance < minDistance) {
           minDistance = distance;
           min = index;
           closestQSM = m_bricks[pointIdxNKNSearch.front()]->getCenter();
         }
     }
     ++index;
  }
  index = 0;
  for (SF_PointNormal point : cluster->points) {
    Eigen::Vector3d pointVec;
    pointVec[0] = point.x;
    pointVec[1] = point.y;
    pointVec[2] = point.z;
    auto distance = SF_Math<double>::distance(pointVec,closestQSM);
    if (distance > maxDistance) {
      maxDistance = distance;
      max = index;
    }
    ++index;
  }
}

Eigen::Vector3f
SF_DijkstraLightRecursive::cloudVector(SF_CloudNormal::Ptr& cloud, const size_t minIndex, const size_t maxIndex)
{
  SF_PointNormal pointMin = cloud->points.at(minIndex);
  SF_PointNormal pointMax = cloud->points.at(maxIndex);
  return Eigen::Vector3f(pointMax.x - pointMin.x, pointMax.y - pointMin.y, pointMax.z - pointMin.z);
}

SF_DijkstraLightRecursive::SF_DijkstraLightRecursive() {}

Eigen::Vector3d
SF_DijkstraLightRecursive::translateCloud(SF_CloudNormal::Ptr& cloud, const size_t index)
{
  SF_PointNormal pointMin = cloud->points.at(index);
  Eigen::Vector3f temp = pointMin.getVector3fMap();
  auto translation = temp.cast<double>();
  for (SF_PointNormal& point : cloud->points) {
    point.x = point.x - pointMin.x;
    point.y = point.y - pointMin.y;
    point.z = point.z - pointMin.z;
  }
  return translation;
}

void SF_DijkstraLightRecursive::initializeKdTree()
{
    _kdtreeQSM.reset(new pcl::KdTreeFLANN<SF_PointNormal>());
    pcl::PointCloud<SF_PointNormal>::Ptr centerCloud(new typename pcl::PointCloud<SF_PointNormal>());
    std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricks = m_qsm->getBuildingBricks();
    for (size_t i = 0; i < buildingBricks.size(); i++) {
      Eigen::Vector3d pointEigen = buildingBricks[i]->getCenter();
      SF_PointNormal point;
      point.getVector3fMap() = pointEigen.cast<float>();
      centerCloud->push_back(std::move(point));
    }
    _kdtreeQSM->setInputCloud(centerCloud);
}
