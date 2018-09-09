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

#include "sf_stem_ransac_filter.h"
#include "pcl/sf_math.h"
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>

SF_Stem_RANSAC_Filter::SF_Stem_RANSAC_Filter() {
    Sf_Binary_Filter<pcl::PointXYZINormal>::reset();
}

void SF_Stem_RANSAC_Filter::computeNormals(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud) {
    pcl::NormalEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal> ne;
    ne.setInputCloud (cloud);
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (_params._radius_normal);
    ne.compute (*cloud);
}

void SF_Stem_RANSAC_Filter::segment(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, pcl::ModelCoefficients::Ptr coeffCylinder, pcl::PointIndices::Ptr inliersCylinder) {
    pcl::SACSegmentationFromNormals<pcl::PointXYZINormal, pcl::PointXYZINormal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (_params._inlierDistance);
    seg.setRadiusLimits (0, 2);
    seg.setInputCloud (cloud);
    seg.setInputNormals (cloud);
    seg.segment (*inliersCylinder, *coeffCylinder);
}

void SF_Stem_RANSAC_Filter::addInliers(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudFiltered, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                                       pcl::ModelCoefficients::Ptr coeffCylinder, pcl::ModelCoefficients::Ptr lastCoeffCylinder, pcl::PointIndices::Ptr inliersCylinder) {
    if(inliersCylinder->indices.size() > std::max<int>((int)(std::ceil(1/_params._voxel_size)/2),10) ) {
        Eigen::Vector3f zAxis;
        zAxis[0] = _params._x;
        zAxis[1] = _params._y;
        zAxis[2] = _params._z;
        if(lastCoeffCylinder->values.size()==7) {
            zAxis[0] = lastCoeffCylinder->values[3];
            zAxis[1] = lastCoeffCylinder->values[4];
            zAxis[2] = lastCoeffCylinder->values[5];
        }
        Eigen::Vector3f cylinderAxis;
        cylinderAxis[0] = coeffCylinder->values[3];
        cylinderAxis[1] = coeffCylinder->values[4];
        cylinderAxis[2] = coeffCylinder->values[5];
        float angle =  SF_Math<float>::getAngleBetweenDeg(zAxis, cylinderAxis);
        if(!(angle>_params._angle && angle < (180-_params._angle)) ) {
            *lastCoeffCylinder = *coeffCylinder;
            for (size_t j = 0; j < inliersCylinder->indices.size (); ++j)
                cloudFiltered->points.push_back(cloud->points[inliersCylinder->indices[j]]);
        }
    }
}

void SF_Stem_RANSAC_Filter::filterIteratively(pcl::PointCloud<pcl::PointXYZINormal>::Ptr downScaledCloudFiltered) {
    pcl::ModelCoefficients::Ptr lastcoeffCylinder (new pcl::ModelCoefficients);
    for(size_t i = 0; i < _clouds.size(); i++) {
        pcl::ModelCoefficients::Ptr coeffCylinder (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliersCylinder (new pcl::PointIndices);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud = _clouds[i];
        if(lastcoeffCylinder->values.size()==7) {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudInlier (new pcl::PointCloud<pcl::PointXYZINormal>());
            Eigen::Vector3d x1;
            x1[0] = lastcoeffCylinder->values[0];
            x1[1] = lastcoeffCylinder->values[1];
            x1[2] = lastcoeffCylinder->values[2];
            Eigen::Vector3d x2;
            x2[0] = lastcoeffCylinder->values[0] + lastcoeffCylinder->values[3];
            x2[1] = lastcoeffCylinder->values[1] + lastcoeffCylinder->values[4];
            x2[2] = lastcoeffCylinder->values[2] + lastcoeffCylinder->values[5];
            for(size_t i = 0; i < cloud->points.size(); i++) {
                Eigen::Vector3d x0;
                x0[0] = cloud->points[i].x;
                x0[1] = cloud->points[i].y;
                x0[2] = cloud->points[i].z;
                Eigen::Vector3d a = x0-x1;
                Eigen::Vector3d b = x0-x2;
                Eigen::Vector3d c = x2-x1;
                double dist = ((a.cross(b)).norm())/(c.norm());
                float radius = std::max<float>((float) lastcoeffCylinder->values[6] + _params._inlierDistance, (float) (lastcoeffCylinder->values[6])*1.5);
                if(dist<radius) {
                    cloudInlier->points.push_back(cloud->points[i]);
                }
            }
            cloud = cloudInlier;
        }
        segment(cloud, coeffCylinder, inliersCylinder);
        addInliers(downScaledCloudFiltered, cloud, coeffCylinder, lastcoeffCylinder, inliersCylinder);
    }
}

void SF_Stem_RANSAC_Filter::backScale(pcl::PointCloud<pcl::PointXYZINormal>::Ptr downScaledCloudFiltered) {
    _cloud_out_filtered_noise.reset(new typename pcl::PointCloud<pcl::PointXYZINormal>);
    _cloud_out_filtered.reset(new typename pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree;
    kdtree.setInputCloud (downScaledCloudFiltered);
    for(size_t i = 0; i < _cloud_in->points.size(); i++) {
        pcl::PointXYZINormal point = _cloud_in->points[i];
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        if ( kdtree.nearestKSearch (point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
            if(std::sqrt(pointNKNSquaredDistance[0]) < _params._voxel_size*2) {
                _cloud_out_filtered->points.push_back(point);
            } else {
                _cloud_out_filtered_noise->points.push_back(point);
            }
        } else {
            _cloud_out_filtered_noise->points.push_back(point);
        }
    }
}

void SF_Stem_RANSAC_Filter::compute() {
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr downScaledCloud = down_scale(_params._voxel_size);
    computeNormals(downScaledCloud);
    initializeClouds(downScaledCloud);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr downScaledCloudFiltered(new SF_Cloud_Normal());
    filterIteratively(downScaledCloudFiltered);
    backScale(downScaledCloudFiltered);
    create_indices();
}

void SF_Stem_RANSAC_Filter::initializeClouds(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud) {
    int size = cloud->points.size();
    float zMin = std::numeric_limits<float>::max();
    float zMax = std::numeric_limits<float>::lowest();
    for(int i = 0; i < size; i++) {
        pcl::PointXYZINormal point = cloud->points[i];
        if(point.z < zMin)
            zMin = point.z;
        if(point.z > zMax)
            zMax = point.z;
    }
    int sliceNumber = std::ceil(zMax-zMin);
    _clouds.clear();
    for(int i = 0; i < sliceNumber; i++) {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr slice(new pcl::PointCloud<pcl::PointXYZINormal>());
        _clouds.push_back(slice);
    }
    for(int i = 0; i < size; i++) {
        pcl::PointXYZINormal point = cloud->points[i];
        int index = std::floor(point.z-zMin);
        _clouds[index]->points.push_back(point);
    }
}
