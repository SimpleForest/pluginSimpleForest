/****************************************************************************

 Copyright (C) 2017-2017 Jan Hackenberg, free software developer
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

#ifndef SF_CELL_H
#define SF_CELL_H

#include <ct_itemdrawable/ct_image2d.h>
#include <memory>
#include <pcl/ModelCoefficients.h>

#include "pcl/sf_point.h"

template <typename PointType> struct SF_DTMCell {
  int _index;
  std::shared_ptr<CT_Image2D<float>> _dtm;
  SF_DTMCell(std::shared_ptr<CT_Image2D<float>> dtm, int index);
  Eigen::Vector2f getMinMaxHeight(const pcl::ModelCoefficients &coeff);
  float getHeight(const Eigen::Vector2d &coords,
                  const pcl::ModelCoefficients &coeff);

private:
  void updateMinMax(const float updateHeight, float &minHeight,
                    float &maxHeight);
  Eigen::Vector2d getCorner1();
  Eigen::Vector2d getCorner2();
  Eigen::Vector2d getCorner3();
  Eigen::Vector2d getCorner4();
};

#include "sf_dtmCell.hpp"

#endif // SF_CELL_H
