#ifndef SF_QSM_CYLINDER_H
#define SF_QSM_CYLINDER_H

#include <pcl/ModelCoefficients.h>

#include <boost/make_shared.hpp>

#include "pcl/sf_math.h"

struct SF_QSMDetectionCylinder
{
  float _distance;
  pcl::ModelCoefficients::Ptr _circleA;
  pcl::ModelCoefficients::Ptr _circleB;

  SF_QSMDetectionCylinder(float distance, pcl::ModelCoefficients circleA)
  {
    _distance = distance;
    _circleA = boost::make_shared<pcl::ModelCoefficients>(circleA);
  }

  SF_QSMDetectionCylinder(float distance, pcl::ModelCoefficients::Ptr circleA)
  {
    _distance = distance;
    _circleA = circleA;
  }

  SF_QSMDetectionCylinder(float distance, pcl::ModelCoefficients::Ptr circleA, pcl::ModelCoefficients::Ptr circleB)
  {
    Eigen::Vector3d pointA(circleA->values[0], circleA->values[1], circleA->values[2]);
    Eigen::Vector3d pointB(circleB->values[0], circleB->values[1], circleB->values[2]);
    _distance = distance + SF_Math<float>::distance(pointA, pointB);
    _circleA = circleA;
    _circleB = circleB;
  }

  void addSecondCircle(pcl::ModelCoefficients::Ptr circleB)
  {
    Eigen::Vector3d pointA(_circleA->values[0], _circleA->values[1], _circleA->values[2]);
    Eigen::Vector3d pointB(circleB->values[0], circleB->values[1], circleB->values[2]);
    _distance = _distance + SF_Math<float>::distance(pointA, pointB);
    _circleB = circleB;
  }

  void addSecondCircle(pcl::ModelCoefficients circleB)
  {
    Eigen::Vector3d pointA(_circleA->values[0], _circleA->values[1], _circleA->values[2]);
    Eigen::Vector3d pointB(circleB.values[0], circleB.values[1], circleB.values[2]);
    _distance = _distance + SF_Math<float>::distance(pointA, pointB);
    _circleB = boost::make_shared<pcl::ModelCoefficients>(circleB);
  }
};

#endif // SF_QSM_CYLINDER_H
