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

#ifndef SF_PYRAMIDLAYER_HPP
#define SF_PYRAMIDLAYER_HPP

#include "sf_pyramidlayer.h"

template<typename PointType>
void PyramidLayer<PointType>::getMinMax() {
    float minX = std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float maxY = std::numeric_limits<float>::lowest();
    for(size_t i = 0; i < _groundCloud->points.size(); i++) {
        PointType p = _groundCloud->points[i];
        if(p.x < minX) minX = p.x;
        if(p.y < minY) minY = p.y;
        if(p.x > maxX) maxX = p.x;
        if(p.y > maxY) maxY = p.y;
    }
    _min[0] = minX;
    _min[1] = minY;
    _max[0] = maxX;
    _max[1] = maxY;
}

template<typename PointType>
float PyramidLayer<PointType>::getGridSize() {
    float dx = _max[0] - _min[0];
    float dy = _max[1] - _min[1];
    float delta = std::max(dx,dy);
    int div = std::pow(2,(_depth-1));
    if(div>0) {
        delta = delta/div;
    }
    return (delta+0.001);
}

template<typename PointType>
void PyramidLayer<PointType>::initialize() {
    _clouds.clear();
    getMinMax();
    float gridSize = getGridSize();
    _DTM.reset(CT_Image2D<float>::createImage2DFromXYCoords(_outDTM.completeName(),
                                                            _outResult,
                                                            _min[0],
                                                            _min[1],
                                                            _max[0],
                                                            _max[1],
                                                            gridSize,
                                                            0,
                                                            -1337,
                                                            0));
    for(size_t i = 0; i < _DTM->xArraySize(); i++) {
        for(size_t j = 0; j < _DTM->yArraySize(); j++) {
            SF_DTMCell<PointType> cell = SF_DTMCell<PointType>(_DTM, 0);
            _cells.push_back(cell);
        }
    }
    for(size_t i = 0; i < _DTM->xArraySize(); i++) {
        for(size_t j = 0; j < _DTM->yArraySize(); j++) {
            size_t index = -1;
            _DTM->index(i,j, index);
            typename pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
            _clouds.push_back(cloud);
            SF_DTMCell<PointType> cell = SF_DTMCell<PointType>(_DTM, index);
            _cells[index] = (cell);
            pcl::ModelCoefficients coeff;
            _planeCoeff.push_back(coeff);
        }
    }
    for(size_t i = 0; i < _groundCloud->points.size(); i++) {
        PointType p = _groundCloud->points[i];
        size_t index;
        _DTM->indexAtCoords(p.x,p.y,index);
        _clouds[index]->points.push_back(p);
        SF_DTMCell<PointType> cell = SF_DTMCell<PointType>(_DTM, index);
        _cells[index] = cell;
    }
}

template<typename PointType>
void PyramidLayer<PointType>::initializeRoot() {
    pcl::ModelCoefficients plane = computePlane(_groundCloud);
    for(size_t i = 0; i < _DTM->xArraySize(); i++) {
        for(size_t j = 0; j < _DTM->yArraySize(); j++) {
            size_t index = -1;
            _DTM->index(i,j, index);
            _planeCoeff[index] = plane;
        }
    }
}

template<typename PointType>
pcl::ModelCoefficients PyramidLayer<PointType>::computePlane(int index) {
    typename pcl::PointCloud<PointType>::Ptr cloud = _clouds[index];
    return computePlane(cloud);
}

template<typename PointType>
pcl::ModelCoefficients PyramidLayer<PointType>::computePlane(typename pcl::PointCloud<PointType>::Ptr cloud) {
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setEpsAngle(SF_Math<float>::_PI/8);
    seg.setMethodType (pcl::SAC_MLESAC);
    seg.setDistanceThreshold (0.05);
    seg.setMaxIterations(100);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, coefficients);
    return coefficients;
}

template<typename PointType>
PyramidLayer<PointType>::PyramidLayer(typename pcl::PointCloud<PointType>::Ptr groundCloud,
                                      CT_ResultGroup *outResult,
                                      CT_AutoRenameModels outDTM) {
    _outResult = outResult;
    _outDTM = outDTM;
    _depth = 1;
    _groundCloud = groundCloud;
    initialize();
    initializeRoot();
}

template<typename PointType>
PyramidLayer<PointType>::PyramidLayer(typename pcl::PointCloud<PointType>::Ptr groundCloud,
                                      int depth,
                                      CT_ResultGroup *outResult,
                                      CT_AutoRenameModels outDTM) {
    _outResult = outResult;
    _outDTM = outDTM;
    _depth = depth;
    _groundCloud = groundCloud;
    initialize();
}

template<typename PointType>
void PyramidLayer<PointType>::setPlaneCoeff(const pcl::ModelCoefficients &coeff, const size_t index) {
    _planeCoeff[index] = coeff;
}

template<typename PointType>
std::shared_ptr<CT_Image2D<float> > PyramidLayer<PointType>::getDTM() const {
    return _DTM;
}

template<typename PointType>
bool PyramidLayer<PointType>::canComputePlane(int index) {
    typename pcl::PointCloud<PointType>::Ptr cloud = _clouds[index];
    if(cloud->points.size()> 7) {
        return true;
    }
    return false;
}

template<typename PointType>
void PyramidLayer<PointType>::setDTM(const std::shared_ptr<CT_Image2D<float> > &DTM) {
    _DTM = DTM;
}

template<typename PointType>
std::vector<pcl::ModelCoefficients> PyramidLayer<PointType>::getPlaneCoeff() const {
    return _planeCoeff;
}

template<typename PointType>
pcl::ModelCoefficients PyramidLayer<PointType>::getPlaneCoeff(const size_t index) const {
    return _planeCoeff[index];
}

template<typename PointType>
Eigen::Vector2f PyramidLayer<PointType>::getMinMaxHeight(const pcl::ModelCoefficients &coeff, const size_t index) {
    SF_DTMCell<PointType> cell = _cells[index];
    return cell.getMinMaxHeight(coeff);
}

template<typename PointType>
float PyramidLayer<PointType>::getHeight(const size_t index) {
    Eigen::Vector2d coords;

    Eigen::Vector2d bot, top;
    _DTM->getCellCoordinates(index, bot,top);
    coords[0] = (bot[0] + top[0])/2;
    coords[1] = (bot[1] + top[1])/2;
    pcl::ModelCoefficients coeff =_planeCoeff[index];
    SF_DTMCell<PointType> cell = _cells[index];
    return cell.getHeight(coords,coeff);
}

template<typename PointType>
void PyramidLayer<PointType>::setPlaneCoeffs(const std::vector<pcl::ModelCoefficients> &plane_coeff) {
    _planeCoeff = plane_coeff;
}


#endif // SF_PYRAMIDLAYER_HPP
