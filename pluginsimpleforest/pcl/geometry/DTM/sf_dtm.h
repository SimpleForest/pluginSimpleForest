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

#ifndef SF_DTM_H
#define SF_DTM_H
#include "sf_pyramidlayer.h"

template <typename PointType> class SF_DTM {
  typename pcl::PointCloud<PointType>::Ptr _groundCloud;
  float _maxAngle;
  float _minCellSize;
  CT_AutoRenameModels _outDTM;
  CT_ResultGroup *_outResult;
  std::shared_ptr<CT_Image2D<float>> _DTM;
  std::vector<std::shared_ptr<PyramidLayer<PointType>>> _pyramidDTM;
  bool isValid(const pcl::ModelCoefficients &parentCoeff,
               const pcl::ModelCoefficients &childCoeff,
               const Eigen::Vector2f &childHeights,
               const Eigen::Vector2f &parentHeights, const float gridSize);
  void updateCoeff(const size_t indexChild, const size_t indexParent,
                   std::shared_ptr<PyramidLayer<PointType>> currentChild,
                   std::shared_ptr<PyramidLayer<PointType>> currentParent);
  std::shared_ptr<PyramidLayer<PointType>> buildPyramid();
  void setHeightForLayer(std::shared_ptr<PyramidLayer<PointType>> layer);
  size_t getParentIndex(const size_t indexChild,
                        std::shared_ptr<PyramidLayer<PointType>> currentChild,
                        std::shared_ptr<PyramidLayer<PointType>> currentParent);

public:
  SF_DTM(typename pcl::PointCloud<PointType>::Ptr groundCloud, float maxAngle,
         float _minCellSize, CT_ResultGroup *outResult,
         CT_AutoRenameModels outDTM);
  std::shared_ptr<CT_Image2D<float>> DTM() const;
};

#include "sf_dtm.hpp"

#endif // SF_DTM_H
