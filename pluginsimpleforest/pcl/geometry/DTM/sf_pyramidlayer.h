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
#ifndef SF_PYRAMIDLAYER_H
#define SF_PYRAMIDLAYER_H

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "sf_cell.h"
#include "pcl/sf_math.h"

template <typename PointType>
class PyramidLayer {
    typename pcl::PointCloud<PointType>::Ptr _ground_cloud;
    std::vector<typename pcl::PointCloud<PointType>::Ptr> _clouds;
    std::vector<Cell<PointType> > _cells;
    std::shared_ptr<CT_Image2D<float> > _DTM;
    std::vector<pcl::ModelCoefficients> _plane_coeff;
    Eigen::Vector2f _min;
    Eigen::Vector2f _max;
    CT_AutoRenameModels     _outDTM;
    CT_ResultGroup * _out_result;
    void getMinMax();
    void initialize();
    void initializeRoot();
    pcl::ModelCoefficients computePlane(typename pcl::PointCloud<PointType>::Ptr cloud);
    int _depth;
public:
    PyramidLayer (typename pcl::PointCloud<PointType>::Ptr ground_cloud, CT_ResultGroup *out_result, CT_AutoRenameModels     outDTM);
    PyramidLayer (typename pcl::PointCloud<PointType>::Ptr ground_cloud, int depth, CT_ResultGroup *out_result, CT_AutoRenameModels     outDTM);
    float getGridSize();
    pcl::ModelCoefficients computePlane(int index);
    bool canComputePlane(int index);

    void setPlaneCoeff(const pcl::ModelCoefficients& coeff, const size_t index);
    void setDTM(const std::shared_ptr<CT_Image2D<float> > &DTM);
    void setPlaneCoeffs(const std::vector<pcl::ModelCoefficients> &plane_coeff);
    std::shared_ptr<CT_Image2D<float> > getDTM() const;
    std::vector<pcl::ModelCoefficients> getPlaneCoeff() const;
    pcl::ModelCoefficients getPlaneCoeff(const size_t index) const;
    Eigen::Vector2f getMinMaxHeight(const pcl::ModelCoefficients& coeff, const size_t index);
    float getHeight(const size_t index);
};

#include "sf_pyramidlayer.hpp"

#endif // SF_PYRAMIDLAYER_H

