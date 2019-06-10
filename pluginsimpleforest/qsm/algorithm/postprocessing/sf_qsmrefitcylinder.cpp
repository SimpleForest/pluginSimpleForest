#include "sf_qsmrefitcylinder.h"

#include "pcl/sf_math.h"

SF_QSMRefitCylinder::SF_QSMRefitCylinder() {}

void
SF_QSMRefitCylinder::setParams(const SF_ParamRefitCylinders& params)
{
  m_params = params;
}

void
SF_QSMRefitCylinder::setCloud(const SF_CloudNormal::Ptr& cloud)
{
  m_cloud = cloud;
}

void
SF_QSMRefitCylinder::initialize()
{
  m_kdtreeQSM.reset(new typename pcl::KdTreeFLANN<SF_PointNormal>());
  SF_CloudNormal::Ptr centerCloud(new SF_CloudNormal());
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricks = m_params._qsm->getBuildingBricks();
  for (size_t i = 0; i < buildingBricks.size(); i++) {
    Eigen::Vector3d pointEigen = buildingBricks[i]->getCenter();
    SF_PointNormal point;
    point.getVector3fMap() = pointEigen.cast<float>();
    point.intensity = i;
    centerCloud->push_back(std::move(point));
    SF_CloudNormal::Ptr cloud(new SF_CloudNormal());
    m_cylinderClusters.push_back(cloud);
  }
  m_kdtreeQSM->setInputCloud(centerCloud);

  for (size_t i = 0; i < m_cloud->points.size(); i++) {
    SF_PointNormal point = m_cloud->points[i];
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if (m_kdtreeQSM->nearestKSearch(point, m_params.m_knn, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
      float minDistance = m_params.m_inlierDistance;
      int index = -1;
      for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
        std::shared_ptr<Sf_ModelAbstractBuildingbrick> neighboringBrick = buildingBricks[pointIdxRadiusSearch[j]];
        float angle = SF_Math<float>::getAngleBetweenDeg(neighboringBrick->getAxis(),
                                                         Eigen::Vector3d(point.normal_x, point.normal_y, point.normal_z));
        if (90 - angle < m_params.m_angle) {
          float distance = std::sqrt(pointRadiusSquaredDistance[j]);
          if (distance < minDistance) {
            minDistance = distance;
            index = pointIdxRadiusSearch[j];
          }
        }
        if (index > -1) {
          m_cylinderClusters[index]->push_back(point);
        }
      }
    }
  }
}

void
SF_QSMRefitCylinder::compute()
{
  initialize();
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> buildingBricks = m_params._qsm->getBuildingBricks();
  for (size_t index = 0; index < buildingBricks.size(); index++) {
    pcl::SACSegmentationFromNormals<SF_PointNormal, SF_PointNormal> seg;
    SF_CloudNormal::Ptr cloud = m_cylinderClusters[index];
    if (cloud->points.size() > m_params.m_minPts) {
      pcl::PointIndices::Ptr inliersCylinder(new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficientsCylinder(new pcl::ModelCoefficients);
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_CYLINDER);
      seg.setMethodType(m_params.m_fittingType);
      seg.setMaxIterations(m_params.m_ransacIterations);
      seg.setDistanceThreshold(m_params.m_inlierDistance);
      seg.setInputCloud(cloud);
      seg.setInputNormals(cloud);
      seg.segment(*inliersCylinder, *coefficientsCylinder);
      if (coefficientsCylinder->values.size() == 7) {
        float angle = SF_Math<float>::getAngleBetweenDeg(
          buildingBricks[index]->getAxis(),
          Eigen::Vector3d(coefficientsCylinder->values[3], coefficientsCylinder->values[4], coefficientsCylinder->values[5]));
        if (angle < m_params.m_angle) {
          float radius = buildingBricks[index]->getRadius();
          double minRad = std::min(radius - m_params.m_minMaxDistance, radius * (1 - m_params.m_range));
          double maxRad = std::max(radius + m_params.m_minMaxDistance, radius * (1 + m_params.m_range));
          float fittedRadius = coefficientsCylinder->values[6];
          if (fittedRadius > minRad && fittedRadius < maxRad) {
            buildingBricks[index]->setCoefficients(coefficientsCylinder);
          }
        }
      }
    }
  }
}
