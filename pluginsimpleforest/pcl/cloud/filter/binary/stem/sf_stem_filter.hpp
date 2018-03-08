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
#ifndef SF_STEM_FILTER_HPP
#define SF_STEM_FILTER_HPP

#include "pcl/cloud/filter/binary/stem/sf_stem_filter.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>



template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr SF_Stem_Filter<PointType>::down_scale(const SF_Param_Stem_Filter<PointType> &params) {
    typename pcl::PointCloud<PointType>::Ptr downscaled_cloud(new typename pcl::PointCloud<PointType>);
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud (SF_Stem_Filter<PointType>::_cloud_in);
    sor.setLeafSize (params._voxel_size, params._voxel_size, params._voxel_size);
    sor.filter (*downscaled_cloud);
    return downscaled_cloud;
}

template<typename PointType>
void SF_Stem_Filter<PointType>::compute_normals(const SF_Param_Stem_Filter<PointType> &params, typename pcl::PointCloud<PointType>::Ptr down_scaled_cloud) {
    pcl::NormalEstimation<PointType, PointType> ne;
    ne.setInputCloud (down_scaled_cloud);
    typename pcl::search::KdTree<PointType>::Ptr tree (new typename pcl::search::KdTree<PointType> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (params._radius_normal);
    ne.compute (*down_scaled_cloud);
}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr SF_Stem_Filter<PointType>::compute_growth_direction(const SF_Param_Stem_Filter<PointType> &params,typename  pcl::PointCloud<PointType>::Ptr down_scaled_cloud) {
    typename pcl::PointCloud<PointType>::Ptr cloud_with_growth_direction(new typename pcl::PointCloud<PointType>);
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud (down_scaled_cloud);
    for (size_t i = 0; i < down_scaled_cloud->points.size(); i++) {
        PointType point = down_scaled_cloud->points.at(i);
        typename pcl::PointCloud<PointType>::Ptr neighborhood(new typename pcl::PointCloud<PointType>);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if ( kdtree.radiusSearch (point, params._radius_growth_direction, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
            if(pointIdxRadiusSearch.size()>2) {
                extract_neighbornormals(down_scaled_cloud, neighborhood, pointIdxRadiusSearch);
                calc_growth_direction(cloud_with_growth_direction,neighborhood,point);
            } else {
                PointType p = down_scaled_cloud->points.at(i);
                p.normal_x = 0.33;
                p.normal_y = 0.33;
                p.normal_z = 0.33;
                cloud_with_growth_direction->points.push_back(p);
            }
        } else {
            PointType p = down_scaled_cloud->points.at(i);
            p.normal_x = 0.33;
            p.normal_y = 0.33;
            p.normal_z = 0.33;
            cloud_with_growth_direction->points.push_back(p);
        }
    }
    return cloud_with_growth_direction;
}

template<typename PointType>
void SF_Stem_Filter<PointType>::calc_growth_direction(typename pcl::PointCloud<PointType>::Ptr cloud_with_growth_direction,typename  pcl::PointCloud<PointType>::Ptr neighborhood, PointType point) {
    Eigen::Matrix3f covariance_matrix;
    Eigen::Vector4f xyz_centroid = {0,0,0,1};
    if(neighborhood->points.size()> 2) {
        pcl::computeCovarianceMatrix (*neighborhood, xyz_centroid, covariance_matrix);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(covariance_matrix);
        Eigen::Vector3f gd = eig.eigenvectors().col(0);
        gd.normalize();
        point.normal_x = gd[0];
        point.normal_y = gd[1];
        point.normal_z = gd[2];
        if(std::isnan(gd[0]) || std::isnan(gd[1]) || std::isnan(gd[2])) {
            point.normal_x = 0.33;
            point.normal_y = 0.33;
            point.normal_z = 0.33;
        }
        cloud_with_growth_direction->points.push_back(point);
    } else {
        point.normal_x = 0.33;
        point.normal_y = 0.33;
        point.normal_z = 0.33;
        cloud_with_growth_direction->points.push_back(point);
    }
}

template<typename PointType>
void SF_Stem_Filter<PointType>::extract_neighbornormals(typename pcl::PointCloud<PointType>::Ptr down_scaled_cloud, typename pcl::PointCloud<PointType>::Ptr neighborhood,
                             std::vector<int> pointIdxRadiusSearch){
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i) {
        PointType neighbor = down_scaled_cloud->points.at(pointIdxRadiusSearch.at(i));
        PointType neighbor_normal;
        neighbor_normal.x = neighbor.normal_x;
        neighbor_normal.y = neighbor.normal_y;
        neighbor_normal.z = neighbor.normal_z;
        if(!std::isnan( neighbor.normal_x) && !std::isnan( neighbor.normal_y) && !std::isnan( neighbor.normal_z)) {
            neighborhood->points.push_back(neighbor_normal);
        }
    }
}

template<typename PointType>
void SF_Stem_Filter<PointType>::transfer_stem(const SF_Param_Stem_Filter<PointType> &params, typename pcl::PointCloud<PointType>::Ptr down_scaled_cloud, typename pcl::PointCloud<PointType>::Ptr cloud_with_growth_direction) {
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud (down_scaled_cloud);
    for(size_t i = 0; i <  SF_Stem_Filter<PointType>::_cloud_in->points.size(); i++) {
        PointType p =  SF_Stem_Filter<PointType>::_cloud_in->points.at(i);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if ( kdtree.nearestKSearch ( p, 1, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
            PointType gd_point = cloud_with_growth_direction->points[pointIdxRadiusSearch[0]];
            Eigen::Vector3f axis1;
            axis1[0] = params._x;
            axis1[1] = params._y;
            axis1[2] = params._z;
            Eigen::Vector3f axis2;
            axis2[0] = gd_point.normal_x;
            axis2[1] = gd_point.normal_y;
            axis2[2] = gd_point.normal_z;

            axis1.normalize();
            axis2.normalize();
            double cos = axis1.dot(axis2);
            double rad = acos(cos);
            double deg = (rad*180.0)/3.1415926;
            if(deg < params._angle || deg > (180-params._angle)) { //|| deg > (180-params._angle)
                SF_Stem_Filter<PointType>::_cloud_out_filtered->points.push_back(p);
            } else {
                SF_Stem_Filter<PointType>::_cloud_out_filtered_noise->points.push_back(p);
            }
        }
    }
}

template<typename PointType>
void SF_Stem_Filter<PointType>::compute(const SF_Param_Stem_Filter<PointType> &params) {

    SF_Stem_Filter<PointType>::_cloud_out_filtered_noise.reset(new typename pcl::PointCloud<PointType>);
    SF_Stem_Filter<PointType>::_cloud_out_filtered.reset(new typename pcl::PointCloud<PointType>);
    SF_Cloud_Normal::Ptr down_scaled_cloud = down_scale(params);
    compute_normals(params,down_scaled_cloud);
    SF_Cloud_Normal::Ptr cloud_with_growth_direction = compute_growth_direction(params,down_scaled_cloud);
    transfer_stem(params,down_scaled_cloud,cloud_with_growth_direction);
    SF_Stem_Filter<PointType>::create_indices();
}

template<typename PointType>
SF_Stem_Filter<PointType>::SF_Stem_Filter() {

}

#endif // SF_STEM_FILTER_HPP
