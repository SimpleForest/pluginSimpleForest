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
#ifndef SF_CONVERTER_CT_TO_PCL_HPP
#define SF_CONVERTER_CT_TO_PCL_HPP

#include <pcl/common/centroid.h>

#include "sf_converter_ct_to_pcl.h"
#include <ct_itemdrawable/ct_grid3d_sparse.h>

template <typename PointType>
SF_Converter_CT_To_PCL<PointType>:: SF_Converter_CT_To_PCL() {
    reset();
}

template <typename PointType>
void SF_Converter_CT_To_PCL<PointType>::reset() {
    _cloud_original.reset(new pcl::PointCloud<PointType>);
    _cloud_translated.reset(new pcl::PointCloud<PointType>);
}

template <typename PointType>
typename
pcl::PointCloud<PointType>::Ptr SF_Converter_CT_To_PCL<PointType>::get_cloud_original() const {
    return _cloud_original;
}

template <typename PointType>
typename
pcl::PointCloud<PointType>::Ptr SF_Converter_CT_To_PCL<PointType>::get_cloud_translated() const {
    return _cloud_translated;
}

template <typename PointType>
void SF_Converter_CT_To_PCL<PointType>::convert_point(CT_PointIterator& it) {
    const CT_Point &internalPoint = it.next().currentPoint();
    PointType origin;
    origin.x = internalPoint[0];
    origin.y = internalPoint[1];
    origin.z = internalPoint[2];
    PointType translated;

    translated.x = internalPoint[0]-_centerOfMass[0];
    translated.y = internalPoint[1]-_centerOfMass[1];
    translated.z = internalPoint[2]-_centerOfMass[2];
    _cloud_original->push_back(origin);
    _cloud_translated->push_back(translated);
}

template <typename PointType>
void SF_Converter_CT_To_PCL<PointType>::iterate_cloud_and_convert(const CT_AbstractPointCloudIndex* index ) {
    CT_PointIterator it(index);
    while(it.hasNext()) {
        convert_point(it);
    }
}

template <typename PointType>
void SF_Converter_CT_To_PCL<PointType>::compute() {
    computeTranslationToOrigin();
    convert();
}

template <typename PointType>
void SF_Converter_CT_To_PCL<PointType>::convert() {
    const CT_AbstractPointCloudIndex* index =_itemCpy_cloud_in->getPointCloudIndex();
    assert( index->size() > 0);
    iterate_cloud_and_convert(index);
}

template<typename PointType>
std::vector<typename pcl::PointCloud<PointType>::Ptr> SF_Converter_CT_To_PCL<PointType>::mergeSubCloudsToVector(CT_Grid3D_Sparse<int>* indices) {
    const CT_AbstractPointCloudIndex* index =_itemCpy_cloud_in->getPointCloudIndex();
    assert( index->size() > 0);
    CT_PointIterator it(index);
    std::vector<pcl::PointCloud<PointType>::Ptr> clouds;
    int number_initialized_clouds = 0;
    while(it.hasNext()) {
        const CT_Point &ct_point = it.next().currentPoint();
        size_t index;
        indices->indexAtXYZ(ct_point(0),ct_point(1),ct_point(2),index);
        int value_at = indices->valueAtIndex(index);
        if(value_at <= -1) {
            pcl::PointCloud<PointType>::Ptr cell_cloud(new pcl::PointCloud<PointType>);
            value_at = number_initialized_clouds++;
            clouds.push_back(cell_cloud);
            indices->setValueAtIndex(index, value_at);
        }
        PointType p;
        p.x = ct_point(0) - _centerOfMass[0];
        p.y = ct_point(1) - _centerOfMass[1];
        p.z = ct_point(2) - _centerOfMass[2];
        clouds[value_at]->points.push_back(p);
    }
    return clouds;
}

template<typename PointType>
void SF_Converter_CT_To_PCL<PointType>::mergeSubCloudsToVector(CT_Grid3D_Sparse<int>* indices, std::vector<typename pcl::PointCloud<PointType>::Ptr> cloudsVec,
                                                               typename pcl::PointCloud<PointType>::Ptr downscaled_cloud) {
    for(size_t i = 0; i < indices->xArraySize(); i++) {
        for(size_t j = 0; j < indices->yArraySize(); j++) {
            for(size_t k = 0; k < indices->zArraySize(); k++) {
                int index = indices->value(i,j,k);
                if(index>=0) {
                    pcl::PointCloud<PointType>::Ptr cell_cloud = cloudsVec[index];
                    if(cell_cloud!=nullptr) {
                        Eigen::Vector4d centroid;
                        pcl::compute3DCentroid (*cell_cloud, centroid);
                        PointType p;
                        p.x = centroid(0);
                        p.y = centroid(1);
                        p.z = centroid(2);
                        p.intensity = cell_cloud->points.size();
                        downscaled_cloud->push_back(p);
                    }
                }
            }
        }
    }
}

template<typename PointType>
void SF_Converter_CT_To_PCL<PointType>::down_scale(float range,  typename pcl::PointCloud<PointType>::Ptr downscaled_cloud) {
    computeTranslationToOrigin();
    CT_Grid3D_Sparse<int>* indices = CT_Grid3D_Sparse<int>::createGrid3DFromXYZCoords(
                                                                     NULL,NULL,
                                                                     _itemCpy_cloud_in->minX(),_itemCpy_cloud_in->minY(),_itemCpy_cloud_in->minZ(),
                                                                     _itemCpy_cloud_in->maxX(),_itemCpy_cloud_in->maxY(),_itemCpy_cloud_in->maxZ(),
                                                                     range, -2, -1);
    std::vector<pcl::PointCloud<PointType>::Ptr> cloudsVec = mergeSubCloudsToVector(indices);
    mergeSubCloudsToVector(indices, cloudsVec, downscaled_cloud);
}

#endif // SF_CONVERTER_CT_TO_PCL_HPP
