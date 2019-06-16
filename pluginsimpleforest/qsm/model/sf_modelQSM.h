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

#ifndef SF_MODEL_TREE_H
#define SF_MODEL_TREE_H

#include "sf_modelAbstractSegment.h"
#include <pcl/common/transforms.h>

class SF_ModelQSM
{
  int _ID;
  std::string _species;
  std::shared_ptr<SF_ModelAbstractSegment> m_rootSegment;
  bool m_hasCorrectedParameters = false;

  double m_aGrowthLength = 0;
  double m_bGrowthLength = 0;
  double m_cGrowthLength = 0;

  double m_aGrowthVolume = 0;
  double m_bGrowthVolume = 0;
  double m_cGrowthVolume = 0;
  Eigen::Vector3d m_translation;

  double m_volumeCorrection = 0;

public:
  SF_ModelQSM(const int ID);
  virtual std::string toString();
  virtual std::string toHeaderString();

  void translate(const Eigen::Vector3d& translation);
  void transform(const Eigen::Affine3f& transform);

  Eigen::Vector3d translateToOrigin();
  std::shared_ptr<SF_ModelAbstractSegment> crownStartSegment(double minPercentage);
  std::shared_ptr<Sf_ModelAbstractBuildingbrick> crownStartBrick(double minPercentage);
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> getSegments();
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> getSegments(std::shared_ptr<SF_ModelAbstractSegment> segment);
  std::vector<std::shared_ptr<SF_ModelAbstractSegment>> getLeaveSegments();
  std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> getBuildingBricks();
  std::shared_ptr<SF_ModelAbstractSegment> getRootSegment() const;
  void setRootSegment(const std::shared_ptr<SF_ModelAbstractSegment>& rootSegment);
  double getVolume();
  void setBranchorder(float twigPercentage);
  void sort(SF_ModelAbstractSegment::SF_SORTTYPE type, float twigPercentage);
  bool getHasCorrectedParameters() const;
  void setHasCorrectedParameters(bool hasCorrectedParameters);
  double getAGrowthLength() const;
  void setAGrowthLength(double aGrowthLength);
  double getBGrowthLength() const;
  void setBGrowthLength(double bGrowthLength);
  double getCGrowthLength() const;
  void setCGrowthLength(double cGrowthLength);
  double getAGrowthVolume() const;
  void setAGrowthVolume(double aGrowthVolume);
  double getBGrowthVolume() const;
  void setBGrowthVolume(double bGrowthVolume);
  double getCGrowthVolume() const;
  void setCGrowthVolume(double cGrowthVolume);
  int getID() const;
  void setID(int ID);
  void setTranslation(const Eigen::Vector3d& translation);
};

#endif // SF_MODEL_TREE_H
