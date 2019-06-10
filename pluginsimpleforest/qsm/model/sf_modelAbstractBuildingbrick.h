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

#ifndef SF_MODEL_ABSTRACT_BUILDINGBRICK_H
#define SF_MODEL_ABSTRACT_BUILDINGBRICK_H

#include <Eigen/Core>
#include <memory.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

enum FittingType
{
  UNKNOWN,
  SPHEREFOLLOWING,
  CYLINDERCORRECTION,
  TRUNCATEDCONECORRECTION,
  MEDIAN,
  ALLOMETRICGROWTHVOLUME,
  ALLOMETRICGROWTHLENGTH,
  CORRECTBRANCHJUNCTIONS,
  MINRADIUS
};

class SF_ModelAbstractSegment;

class Sf_ModelAbstractBuildingbrick : public std::enable_shared_from_this<Sf_ModelAbstractBuildingbrick>
{
protected:
  size_t _ID;
  size_t _indexVector;
  std::weak_ptr<SF_ModelAbstractSegment> _segment;
  Eigen::Vector3d _start;
  Eigen::Vector3d _end;
  FittingType _fittingType;
  virtual double getDistanceToAxis(const Eigen::Vector3d& point) = 0;
  virtual Eigen::Vector3d getProjectionOnAxis(const Eigen::Vector3d& point) = 0;
  virtual double getBoundingSphereRadius();

public:
  Sf_ModelAbstractBuildingbrick();
  std::shared_ptr<Sf_ModelAbstractBuildingbrick> getParent();
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> getChildren();

  double getGrowthLength();
  double getGrowthVolume();
  virtual double getLength() = 0;
  virtual double getVolume() = 0;
  virtual double getRadius() = 0;
  virtual void setStartEndRadius(const Eigen::Vector3d& start, const Eigen::Vector3d& end, double radius, FittingType type) = 0;
  virtual void setRadius(double radius, FittingType type) = 0;
  virtual void setCoefficients(pcl::ModelCoefficients::Ptr coefficients) = 0;
  virtual double getDistance(const Eigen::Vector3d& point) = 0;
  virtual void translate(Eigen::Vector3d translation) = 0;
  void remove();
  double getDistance(const pcl::PointXYZ& point);
  double getDistance(const pcl::PointXYZINormal& point);
  virtual Eigen::Vector3d getCenter() = 0;
  virtual Eigen::Vector3d getAxis() = 0;
  virtual std::string toString() = 0;
  virtual std::string toHeaderString() = 0;
  bool operator<(const Sf_ModelAbstractBuildingbrick& other);

  size_t getIndex() const;
  void setIndex(const size_t& index);
  size_t getID() const;
  void setID(const size_t& ID);
  Eigen::Vector3d getStart() const;
  Eigen::Vector3d getEnd() const;
  std::shared_ptr<SF_ModelAbstractSegment> getSegment();
  void setSegment(std::shared_ptr<SF_ModelAbstractSegment> segment);
  void setFittingType(FittingType fittingType);
  FittingType getFittingType() const;
};

#endif // SF_MODEL_ABSTRACT_BUILDINGBRICK_H
