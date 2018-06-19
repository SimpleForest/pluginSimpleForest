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

#ifndef SF_DTM_HPP
#define SF_DTM_HPP

#include "sf_dtm.h"

template<typename PointType>
void SF_DTM<PointType>::setHeightForLayer(std::shared_ptr<PyramidLayer<PointType> > layer) {
    _DTM = layer->getDTM();
    for(size_t i = 0; i < _DTM->xArraySize(); i++) {
        for(size_t j = 0; j < _DTM->yArraySize(); j++) {
            size_t index;
            _DTM->index(i,j,index);
            float height = layer->getHeight(index);
            _DTM->setValueAtIndex(index,height);
        }
    }
}

template<typename PointType>
std::shared_ptr<PyramidLayer<PointType> > SF_DTM<PointType>::buildPyramid() {
    std::shared_ptr<PyramidLayer<PointType> > rootLayer(new PyramidLayer<PointType>(_ground_cloud, _out_result, _outDTM) );
    std::shared_ptr<PyramidLayer<PointType> > currentParent = rootLayer;
    float gridSize = currentParent->getGridSize();
    int depth = 1;
    while(gridSize > _min_cell_size) {
        depth ++;
        std::shared_ptr<PyramidLayer<PointType> > currentChild(new PyramidLayer<PointType>(_ground_cloud, depth, _out_result, _outDTM));
        gridSize = currentChild->getGridSize();
        std::shared_ptr<CT_Image2D<float> > childDTM = currentChild->getDTM();
        for(size_t i = 0; i < childDTM->xArraySize(); i++) {
            for(size_t j = 0; j < childDTM->yArraySize(); j++) {
                size_t indexChild;
                childDTM->index(i,j,indexChild);
                size_t indexParent = getParentIndex(indexChild, currentChild, currentParent);
                updateCoeff(indexChild, indexParent, currentChild, currentParent);
            }
        }
        currentParent = currentChild;
    }
    return currentParent;
}

template<typename PointType>
size_t SF_DTM<PointType>::getParentIndex(const size_t indexChild, std::shared_ptr<PyramidLayer<PointType> > currentChild,
                                                                  std::shared_ptr<PyramidLayer<PointType> > currentParent) {
    size_t indexParent;
    Eigen::Vector2d bot, top, center;
    currentChild->getDTM()->getCellCoordinates(indexChild, bot,top);
    center[0] = (bot[0] + top[0])/2;
    center[1] = (bot[1] + top[1])/2;
    currentParent->getDTM()->indexAtCoords(center[0],center[1],indexParent);
    return indexParent;
}

template<typename PointType>
void SF_DTM<PointType>::updateCoeff(const size_t indexChild, const size_t indexParent,
                                    std::shared_ptr<PyramidLayer<PointType> > currentChild, std::shared_ptr<PyramidLayer<PointType> > currentParent) {
    pcl::ModelCoefficients parentCoeff = currentParent->getPlaneCoeff(indexParent);
    if(currentChild->canComputePlane(indexChild)) {
        pcl::ModelCoefficients childCoeff = currentChild->computePlane(indexChild);
        Eigen::Vector2f parentHeights = currentChild->getMinMaxHeight(parentCoeff, indexChild);
        Eigen::Vector2f childHeights = currentChild->getMinMaxHeight(childCoeff, indexChild);
        float gridSize = currentChild->getGridSize();
        if(isValid(parentCoeff, childCoeff, childHeights, parentHeights, gridSize) ) {
            currentChild->setPlaneCoeff(childCoeff, indexChild);
        } else {
            currentChild->setPlaneCoeff(parentCoeff, indexChild);
        }
    } else {
        currentChild->setPlaneCoeff(parentCoeff, indexChild);
    }
}

template<typename PointType>
bool SF_DTM<PointType>::isValid(const pcl::ModelCoefficients &parentCoeff, const pcl::ModelCoefficients &childCoeff, const Eigen::Vector2f &childHeights, const Eigen::Vector2f &parentHeights, const float gridSize) {
    Eigen::Vector3f normal;
    normal(0) = childCoeff.values[0];
    normal(1) = childCoeff.values[1];
    normal(2) = childCoeff.values[2];
    Eigen::Vector3f zAxis;
    zAxis(0) = parentCoeff.values[0];
    zAxis(1) = parentCoeff.values[1];
    zAxis(2) = parentCoeff.values[2];
    float angle = SF_Math<float>::get_angle_between_DEG(normal,zAxis);
    float maxDelta = std::min(1.0f, gridSize/2);
    bool isHorizontal = ((angle < _maxAngle) || (angle > 180 - _maxAngle));
    bool isGoodHeight = ((childHeights[0]> (parentHeights[0]-maxDelta) ) && (childHeights[1] < (parentHeights[1]+maxDelta) ));
    return(isHorizontal && isGoodHeight);
}

template<typename PointType>
SF_DTM<PointType>::SF_DTM(typename pcl::PointCloud<PointType>::Ptr ground_cloud, float maxAngle, float min_cell_size, CT_ResultGroup *out_result, CT_AutoRenameModels outDTM) {
    _out_result = out_result;
    _outDTM = outDTM;
    _maxAngle = maxAngle;
    _min_cell_size = min_cell_size;
    _ground_cloud = ground_cloud;
    std::shared_ptr<PyramidLayer<PointType> > layer = buildPyramid();
    setHeightForLayer(layer);
}

template<typename PointType>
std::shared_ptr<CT_Image2D<float> > SF_DTM<PointType>::DTM() const {
    return _DTM;
}

#endif // SF_DTM_HPP
