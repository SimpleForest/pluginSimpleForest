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

#include "sf_spherefollowingRecursive.h"

#include "qsm/algorithm/distance/sf_extractFittedPoints.h"
#include "qsm/model/sf_modelCylinderBuildingbrick.h"

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

void
SF_SpherefollowingRecursive::setParams(const SF_ParamSpherefollowingRecursive<SF_PointNormal>& params)
{
  m_params = params;
}

void
SF_SpherefollowingRecursive::setCloud(const SF_CloudNormal::Ptr& cloud)
{
  m_cloud = cloud;
}

void
SF_SpherefollowingRecursive::setQsm(const std::shared_ptr<SF_ModelQSM>& qsm)
{
  m_qsm = qsm;
  m_axis = stemAxis();
  m_bricks = m_qsm->getBuildingBricks();
  initializeKdTree();
}

std::shared_ptr<SF_ModelQSM>
SF_SpherefollowingRecursive::getQsm() const
{
  return m_qsm;
}

SF_CloudNormal::Ptr
SF_SpherefollowingRecursive::extractUnfittedPoints()
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
SF_SpherefollowingRecursive::clusters(SF_CloudNormal::Ptr cloud)
{
  size_t minSize = static_cast<size_t>(m_params.m_minPercentage * m_cloud->points.size());
  pcl::search::KdTree<SF_PointNormal>::Ptr tree(new pcl::search::KdTree<SF_PointNormal>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<SF_PointNormal> ec;
  ec.setClusterTolerance(m_params.m_clusteringDistance);
  ec.setMinClusterSize(minSize);
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

std::vector<SF_CloudNormal::Ptr>
SF_SpherefollowingRecursive::sortClusters(const std::vector<SF_CloudNormal::Ptr>& clusters)
{
  return clusters;
}

void
SF_SpherefollowingRecursive::compute()
{
  SF_CloudNormal::Ptr unfitted = extractUnfittedPoints();
  std::vector<SF_CloudNormal::Ptr> clouds = clusters(unfitted);
  std::vector<SF_CloudNormal::Ptr> sortedClusters = sortClusters(clouds);
  processClusters(sortedClusters);
}

void
SF_SpherefollowingRecursive::processClusters(const std::vector<SF_CloudNormal::Ptr>& clusters)
{
  for (SF_CloudNormal::Ptr cluster : clusters) {
    size_t minIndex = 0;
    size_t maxIndex = 0;
    getMinMax(minIndex, maxIndex, cluster);
    auto translation = translateCloud(cluster, minIndex);

    Eigen::Vector3f zAxis(0, 0, 1);
    Eigen::Vector3f cylinderAxisF = cloudVector(cluster, minIndex, maxIndex);
    cylinderAxisF = cylinderAxisF.normalized();
    Eigen::Vector3f rotationAxis = zAxis.cross(cylinderAxisF);
    rotationAxis = rotationAxis.normalized();
    float angle = std::acos(zAxis.dot(cylinderAxisF));

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(angle, rotationAxis));
    SF_CloudNormal::Ptr transformed(new SF_CloudNormal);
    pcl::transformPointCloud(*cluster, *transformed, transform);

    bool inverted = false;
    if (transformed->points[maxIndex].z < transformed->points[minIndex].z) {
      inverted = true;
      for (auto point : transformed->points) {
        point.z = -point.z;
      }
    }
    SF_SphereFollowingRasterSearch sphereFollowing;
    SF_ParamSpherefollowingRecursive<SF_PointNormal> params = m_params;
    sphereFollowing.setParams(params);
    sphereFollowing.setFire(false);
    sphereFollowing.setCloud(transformed);
    sphereFollowing.compute();
    m_params._stepProgress->fireComputation();
    auto qsm = sphereFollowing.getParamVec()[0]._qsm;
    if (!qsm) {
      continue;
    }
    if (inverted) {
      auto bricks = qsm->getBuildingBricks();
      for (std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick : bricks) {
        auto start = brick->getStart();
        auto end = brick->getEnd();
        start[2] = -start[2];
        end[2] = -end[2];
        brick->setStartEndRadius(start, end, brick->getRadius(), brick->getFittingType());
      }
    }
    qsm->sort(SF_ModelAbstractSegment::SF_SORTTYPE::GROWTH_VOLUME, 0.0001);
    Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
    transform2.rotate(Eigen::AngleAxisf(-angle, rotationAxis));
    qsm->transform(transform2);
    qsm->translate(translation);
    qsm->setTranslation(Eigen::Vector3d(0, 0, 0));
    connectQSM(qsm);
  }
}

void
SF_SpherefollowingRecursive::connectQSM(std::shared_ptr<SF_ModelQSM> childQSM)
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
  for (auto segment : allChildSegments) {
    segment->setTree(m_qsm);
  }
  if (start == startChild) {
    std::cout << "SF_SpherefollowingRecursive::connectQSM(std::shared_ptr<SF_ModelQSM> childQSM) no cylinder to connect found"
              << std::endl;
    m_qsm->getRootSegment()->addChild(childQSM->getRootSegment());
  } else {
    auto radius = childQSM->getRootSegment()->getBuildingBricks().front()->getRadius();
    std::shared_ptr<Sf_ModelCylinderBuildingbrick> brickConnect(new Sf_ModelCylinderBuildingbrick(start, startChild, radius));
    brickConnect->setFittingType(FittingType::CONNECTQSM);
    std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bricks;
    bricks.push_back(brickConnect);
    std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> oldBricks = childQSM->getRootSegment()->getBuildingBricks();
    for (auto oldBrick : oldBricks) {
      bricks.push_back(oldBrick);
    }
    childQSM->getRootSegment()->setBuildingBricks(bricks);
    segment->addChild(childQSM->getRootSegment());
  }
}

void
SF_SpherefollowingRecursive::getMinMax(size_t& min, size_t& max, SF_CloudNormal::Ptr cluster)
{
  double minDistance = std::numeric_limits<double>::max();
  double maxDistance = 0;
  Eigen::Vector3d closestQSM;
  size_t index = 0;
  for (SF_PointNormal point : cluster->points) {
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if (_kdtreeQSM->nearestKSearch(point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
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
    auto distance = SF_Math<double>::distance(pointVec, closestQSM);
    if (distance > maxDistance) {
      maxDistance = distance;
      max = index;
    }
    ++index;
  }
}

std::shared_ptr<Sf_ModelCylinderBuildingbrick>
SF_SpherefollowingRecursive::stemAxis()
{
  auto brick = m_qsm->crownStartBrick(0.5);
  auto start = m_qsm->getRootSegment()->getBuildingBricks().front()->getStart();
  auto end = brick->getEnd();
  std::shared_ptr<Sf_ModelCylinderBuildingbrick> result(new Sf_ModelCylinderBuildingbrick(start, end, 0.0001));
  return result;
}

Eigen::Vector3f
SF_SpherefollowingRecursive::cloudVector(SF_CloudNormal::Ptr& cloud, const size_t minIndex, const size_t maxIndex)
{
  SF_PointNormal pointMin = cloud->points.at(minIndex);
  SF_PointNormal pointMax = cloud->points.at(maxIndex);
  return Eigen::Vector3f(pointMax.x - pointMin.x, pointMax.y - pointMin.y, pointMax.z - pointMin.z);
}

SF_SpherefollowingRecursive::SF_SpherefollowingRecursive() {}

Eigen::Vector3d
SF_SpherefollowingRecursive::translateCloud(SF_CloudNormal::Ptr& cloud, const size_t index)
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

void
SF_SpherefollowingRecursive::initializeKdTree()
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
