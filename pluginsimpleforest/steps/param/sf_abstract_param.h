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
#ifndef SF_ABSTRACT_PARAM_H
#define SF_ABSTRACT_PARAM_H

#include <pcl/sample_consensus/method_types.h>
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_result/model/outModel/ct_outresultmodelgroup.h"
#include "ct_itemdrawable/ct_image2d.h"
#include "pcl/sf_point.h"
#include "qsm/algorithm/spherefollowing/sf_spherefollowing_parameters.h"
#include "qsm/algorithm/distance/sf_cloud_to_model_distance_parameters.h"
#include "qsm/sf_model_tree.h"

struct SF_Param_CT{
    CT_StandardItemGroup* _grpCpyGrp;
    LogInterface* _log;
    const CT_AbstractItemDrawableWithPointCloud* _itemCpyCloudIn;
    CT_ResultGroup* _resCpy_res;
    virtual QString toString() {
        QString str;
        return str;
    }

    virtual void log_import() {
        QString str = to_string_import();
        _log->addMessage(LogInterface::info, LogInterface::step, str);
    }

private:

    virtual QString to_string_import() {
        QString str = "A cloud with ";
        str.append(QString::number(_itemCpyCloudIn->getPointCloudIndex()->size()));
        str.append(" points was successfully converted.");
        return str;
    }
};

template <typename PointType>
struct SF_Param_Cloud: public SF_Param_CT{

    typename pcl::PointCloud<PointType>::Ptr _cloud_in;

    virtual void log_filter(double percentage) {
        QString str = to_string_filter(percentage);
        _log->addMessage(LogInterface::info, LogInterface::step, str);
    }

private:

    virtual QString to_string_filter(double percentage) {
        QString str = "From the cloud with ";
        str.append(QString::number(_cloud_in->points.size()));
        str.append(" points remain after filtering ");
        str.append(QString::number(percentage, 'f', 2));
        str.append(" percent remain.");
        return str;
    }
};

template <typename PointType>
struct SF_Param_Filter: public SF_Param_Cloud<PointType>{
    int _size_output;
    std::vector<int> _output_indices;
};

template <typename PointType>
struct SF_Param_Euclidean_Clustering: public SF_Param_Filter<PointType>{
    float _cellSize;
    float _euclideanDistance;
    int _minSize;
    std::vector<int> _output_indices;
};

template <typename PointType>
struct SF_Param_DTM_Height: public SF_Param_Filter<PointType>{
    float _cropHeight;
    CT_Image2D<float> * _dtmCT;
    std::vector<int> _output_indices;
};

template <typename PointType>
struct SF_Param_Growth_direction: public SF_Param_Cloud<PointType>{
    float gd_range   = 0.03f;
    float normal_range = 0.01f;
    virtual QString toString() {
        QString str = "The growth direction computation with parameter (";
        str.append(QString::number(gd_range));
        str.append(" growth direction range) has started.");
        return str;
    }
};

template <typename PointType>
struct SF_Param_Voxel_Grid_Downscale: public SF_Param_Cloud<PointType>{
    float voxel_size   = 0.01f;
    float voxel_size_x = 0.01f;
    float voxel_size_y = 0.01f;
    float voxel_size_z = 0.01f;
    virtual QString toString() {
        QString str = "The voxelgrid downscale filter with parameters (cell_size_x = ";
        str.append(QString::number(voxel_size_x));
        str.append("; cell_size_y = ");
        str.append(QString::number(voxel_size_y));
        str.append("; cell_size_z = ");
        str.append(QString::number(voxel_size_z));
        str.append(") is started.");
        return str;
    }
};

template <typename PointType, typename FeatureType>
struct SF_Param_Normals: public SF_Param_Cloud<PointType> {
    bool _use_radius = true;
    float _radius = 0.03f;
    int _kn = 25;
    virtual QString toString() {
        QString str = "The normal estimtion with a neighborhood size of ";
        if(_use_radius) {
            str.append(QString::number(_radius));
            str.append(" m ");
        } else {
            str.append(QString::number(_kn));
            str.append(" nearest neighbors ");
        }
        str.append("is started.");
        return str;
    }
};

template <typename PointType>
struct SF_Param_Ground_Filter : public SF_Param_Filter<PointType> {
    float _voxel_size = 0.04f;
    float _radius_normal = 0.2f;
    float _x = 0;
    float _y = 0;
    float _z = 1;
    int _angle = 20;
    virtual QString toString() {
        QString str = "The ground filter with parameters (angle = ";
        str.append(QString::number(_angle));
        str.append("; axis = (");
        str.append(QString::number(_x));
        str.append("; ");
        str.append(QString::number(_y));
        str.append("; ");
        str.append(QString::number(_z));
        str.append("); radius_normal = ");
        str.append(QString::number(_radius_normal));
        str.append(" and voxel_down_scale_size = ");
        str.append(QString::number(_voxel_size));
        str.append(") is started.");
        return str;
    }
};

template <typename PointType>
struct SF_Param_DTM : public SF_Param_Filter<PointType> {
    float _voxel_size = 0.03f;
    float _radius_normal = 0.1f;
    float _min_cell_size = 0.2;
    virtual QString toString() {
        QString str = "The DTM generation with parameters (voxel_size = ";
        str.append(QString::number(_voxel_size));
        str.append("; radius_normal = ");
        str.append(QString::number(_radius_normal));
        str.append(" and _min_size = ");
        str.append(QString::number(_min_cell_size));
        str.append(") is started.");
        return str;
    }
};

struct SF_Param_Stem_RANSAC_Filter : public SF_Param_Filter<pcl::PointXYZINormal> {
    float _voxel_size = 0.01f;
    float _radius_normal = 0.03f;
    float _inlierDistance = 0.1f;
    float _x = 0;
    float _y = 0;
    float _z = 1;
    int _angle = 20;
    virtual QString toString() {
        QString str = "The stem RANSAC filter ";
        str.append(" is started.");
        return str;
    }
};

template <typename PointType>
struct SF_Param_Stem_Filter : public SF_Param_Filter<PointType> {
    float _voxel_size = 0.02f;
    float _radius_normal = 0.05f;
    float _radius_growth_direction = 0.1f;
    float _x = 0;
    float _y = 0;
    float _z = 1;
    int _angle = 10;
    virtual QString toString() {
        QString str = "The stem filter with parameters (angle = ";
        str.append(QString::number(_angle));
        str.append("; axis = (");
        str.append(QString::number(_x));
        str.append("; ");
        str.append(QString::number(_y));
        str.append("; ");
        str.append(QString::number(_z));
        str.append("); radius_growth_direction =");
        str.append(QString::number(_radius_growth_direction));
        str.append("); radius_normal = ");
        str.append(QString::number(_radius_normal));
        str.append(" and voxel_down_scale_size = ");
        str.append(QString::number(_voxel_size));
        str.append(") is started.");
        return str;
    }
};

template <typename PointType>
struct SF_ParamSpherefollowingBasic : public SF_Param_Filter<PointType> {
    SF_SphereFollowingParameters _sphereFollowingParams;
    SF_CloudToModelDistanceParameters _distanceParams;
    Eigen::Vector3d _translation;
    float _voxelSize                        = 0.01;
    float _clusteringDistance               = 0.2;
    float _maxError;
    float _minError;
    int _fittedGeometries;
    std::vector<float> _errors;
    std::shared_ptr<SF_Model_Tree> _tree;

    QString methodToString(int method) {
        QString str;
        switch (method) {
        case pcl::SAC_RANSAC:
            str = "RANSAC";
            break;
        case pcl::SAC_LMEDS:
            str = "LMEDS";
            break;
        case pcl::SAC_MLESAC:
            str = "MLESAC";
            break;
        case pcl::SAC_MSAC:
            str = "MSAC";
            break;
        case pcl::SAC_PROSAC:
            str = "PROSAC";
            break;
        case pcl::SAC_RMSAC:
            str = "SAC_RMSAC";
            break;
        case pcl::SAC_RRANSAC:
            str = "SAC_RRANSAC";
            break;
        default:
            break;
        }
        return str;
    }

    virtual QString toString() {
        QString str = "The SphereFollwing method with parameters \n (_voxelSize = ";
        str.append(QString::number(_voxelSize));
        str.append("; _ransacCircleInlierDistance = ");
        str.append(QString::number(_sphereFollowingParams._inlierDistance));
        str.append("; Optimizable Parameters Output Following: \n {");
        for(size_t i = 0; i < _sphereFollowingParams._optimizationParams.size(); i++) {
            str.append(" { _euclideanClusteringDistance = ");
            str.append(QString::number(_sphereFollowingParams._optimizationParams.at(i)._euclideanClusteringDistance));
            str.append("; _sphereRadiusMultiplier = ");
            str.append(QString::number(_sphereFollowingParams._optimizationParams.at(i)._sphereRadiusMultiplier));
            str.append("; _minRadius = ");
            str.append(QString::number(_sphereFollowingParams._optimizationParams.at(i)._minRadius));
            str.append("; _epsilonSphere = ");
            str.append(QString::number(_sphereFollowingParams._optimizationParams.at(i)._epsilonSphere));
            str.append("}");
        }
        str.append("}\n Hyper parameters following: _minPtsCircle = ");
        str.append(QString::number(_sphereFollowingParams._minPtsGeometry));
        str.append("; _heightStartSphere = ");
        str.append(QString::number(_sphereFollowingParams._heightInitializationSlice));
        str.append("; _ransacCircleInlierDistance = ");
        str.append(QString::number(_sphereFollowingParams._inlierDistance));
        str.append("; _ransacIterations = ");
        str.append(QString::number(_sphereFollowingParams._RANSACIterations));
        str.append("; _minGlobalRadius = ");
        str.append(QString::number(_sphereFollowingParams._minGlobalRadius));
        str.append("; _fittingMethod = ");
        str.append(methodToString(_sphereFollowingParams._fittingMethod));
        str.append(") \n has been optimized. During the Parameter search ");
        str.append(QString::number(_sphereFollowingParams._optimizationParams.size()*6));
        str.append(" parameters have been optimized.");
        str.append(" The error has been reduced from \n  max:");
        str.append(QString::number(_maxError));
        str.append("\n to min:");
        str.append(QString::number(_minError));
        str.append(". \n During the procedure ");
        str.append(QString::number(_fittedGeometries));
        str.append(" geometries have been fitted.");
        return str;
    }
};

template <typename PointType>
struct SF_Param_Statistical_Outlier_Filter : public SF_Param_Filter<PointType> {
    int _k = 25;
    float _std_mult = 3.5;
    int _iterations = 15;
    virtual QString toString() {
        QString str = "The statistical outlier filter with parameters (kn = ";
        str.append(QString::number(_k));
        str.append("; std_mult = ");
        str.append(QString::number(_std_mult));
        str.append("; iterations = ");
        str.append(QString::number(_iterations));
        str.append("; iterations = ");
        str.append(") is started.");
        return str;
    }
};

template <typename PointType>
struct SF_Param_Radius_Outlier_Filter : public SF_Param_Filter<PointType> {
    int _min_Pts = 15;
    double _radius = 10;
    virtual QString toString() {
        QString str = "The radius outlier filter with parameters (_min_pts = ";
        str.append(QString::number(_min_Pts));
        str.append("; radius = ");
        str.append(QString::number(_radius));
        str.append(") is started.");
        return str;
    }


};

#endif // SF_ABSTRACT_PARAM_H
