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

#ifndef SF_MODEL_CYLINDER_BUILDINGBRICK_H
#define SF_MODEL_CYLINDER_BUILDINGBRICK_H

#include "sf_modelAbstractBuildingbrick.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

class Sf_ModelCylinderBuildingbrick : public Sf_ModelAbstractBuildingbrick
{
  friend class SF_UnitTestCylinder;
  double m_radius;

protected:
  double getDistanceToAxis(const Eigen::Vector3d& point) override;
  double getProjectedDistanceToSegment(const Eigen::Vector3d& point);
  double getDistanceToInfinitHull(const Eigen::Vector3d& point);
  Eigen::Vector3d getProjectionOnAxis(const Eigen::Vector3d& point) override;

public:
  Sf_ModelCylinderBuildingbrick(pcl::ModelCoefficients::Ptr circleA, pcl::ModelCoefficients::Ptr circleB);
  Sf_ModelCylinderBuildingbrick(Eigen::Vector3d start, Eigen::Vector3d end, double radius);
  std::string toString() override;
  std::string toHeaderString() override;
  double getRadius() override;
  void transform(const Eigen::Affine3f& transform) override;
  void setRadius(double radius, FittingType type) override;
  void translate(Eigen::Vector3d translation) override;
  double getVolume() override;
  double getLength() override;
  double getDistance(const Eigen::Vector3d& point) override;
  virtual Eigen::Vector3d getCenter() override;
  virtual Eigen::Vector3d getAxis() override;
  virtual void setStartEndRadius(const Eigen::Vector3d& start, const Eigen::Vector3d& end, double radius, FittingType type) override;
  virtual void setCoefficients(pcl::ModelCoefficients::Ptr coefficients) override;
};

#endif // SF_MODEL_CYLINDER_BUILDINGBRICK_H
