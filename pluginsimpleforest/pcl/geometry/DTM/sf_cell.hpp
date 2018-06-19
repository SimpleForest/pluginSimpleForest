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

#ifndef SF_CELL_HPP
#define SF_CELL_HPP

#include "sf_cell.h"

template<typename PointType>
void Cell<PointType>:: updateMinMax(const float updateHeight, float &minHeight, float& maxHeight) {
    if(updateHeight>maxHeight) {
        maxHeight = updateHeight;
    }
    if(updateHeight<minHeight) {
        minHeight = updateHeight;
    }
}

template<typename PointType>
Cell<PointType>::Cell(std::shared_ptr<CT_Image2D<float> > dtm, int index) {
    _dtm   = dtm;
    _index = index;
}

template<typename PointType>
Eigen::Vector2d Cell<PointType>::getCorner1(){
    Eigen::Vector2d bot, top;
    _dtm->getCellCoordinates(_index, bot,top);
    Eigen::Vector2d corner;
    corner[0] = bot[0];
    corner[1] = bot[1];
    return corner;
}

template<typename PointType>
Eigen::Vector2d Cell<PointType>::getCorner2(){
    Eigen::Vector2d bot, top;
    _dtm->getCellCoordinates(_index, bot,top);
    Eigen::Vector2d corner;
    corner[0] = bot[0];
    corner[1] = top[1];
    return corner;
}

template<typename PointType>
Eigen::Vector2d Cell<PointType>::getCorner3(){
    Eigen::Vector2d bot, top;
    _dtm->getCellCoordinates(_index, bot,top);
    Eigen::Vector2d corner;
    corner[0] = top[0];
    corner[1] = bot[1];
    return corner;
}

template<typename PointType>
Eigen::Vector2d Cell<PointType>::getCorner4(){
    Eigen::Vector2d bot, top;
    _dtm->getCellCoordinates(_index, bot,top);
    Eigen::Vector2d corner;
    corner[0] = top[0];
    corner[1] = top[1];
    return corner;
}

template<typename PointType>
float Cell<PointType>::getHeight(const Eigen::Vector2d &coords, const pcl::ModelCoefficients &coeff) {
    float height = (coords[0]*coeff.values[0] + coords[1]*coeff.values[1] + coeff.values[3])/-coeff.values[2];
    return height;
}

template<typename PointType>
Eigen::Vector2f Cell<PointType>::getMinMaxHeight(const pcl::ModelCoefficients &coeff) {
    Eigen::Vector2d corner1 = getCorner1();
    Eigen::Vector2d corner2 = getCorner2();
    Eigen::Vector2d corner3 = getCorner3();
    Eigen::Vector2d corner4 = getCorner4();

    float minHeight = std::numeric_limits<float>::max();
    float maxHeight = std::numeric_limits<float>::lowest();
    float height = getHeight(corner1, coeff);
    updateMinMax(height,minHeight,  maxHeight);
    height = getHeight(corner2, coeff);
    updateMinMax(height,minHeight,  maxHeight);
    height = getHeight(corner3, coeff);
    updateMinMax(height,minHeight,  maxHeight);
    height = getHeight(corner4, coeff);
    updateMinMax(height,minHeight,  maxHeight);

    Eigen::Vector2f minMax;
    minMax[0] = minHeight;
    minMax[1] = maxHeight;
    return minMax;
}

#endif // SF_CELL_HPP
