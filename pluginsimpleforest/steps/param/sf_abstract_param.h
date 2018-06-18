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

#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_result/model/outModel/ct_outresultmodelgroup.h"
#include "pcl/sf_point.h"


//TODO Check standard parameters.


struct SF_Param_CT{
    CT_StandardItemGroup* _grpCpy_grp;
    LogInterface* _log;
    const CT_AbstractItemDrawableWithPointCloud* _itemCpy_cloud_in;
    CT_ResultGroup* _resCpy_res;
    virtual QString to_string() {
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
        str.append(QString::number(_itemCpy_cloud_in->getPointCloudIndex()->size()));
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
struct SF_Param_Growth_direction: public SF_Param_Cloud<PointType>{
    float gd_range   = 0.03f;
    float normal_range = 0.01f;
    virtual QString to_string() {
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
    virtual QString to_string() {
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
    virtual QString to_string() {
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
    virtual QString to_string() {
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
    virtual QString to_string() {
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

template <typename PointType>
struct SF_Param_Stem_Filter : public SF_Param_Filter<PointType> {
    float _voxel_size = 0.02f;
    float _radius_normal = 0.05f;
    float _radius_growth_direction = 0.1f;
    float _x = 0;
    float _y = 0;
    float _z = 1;
    int _angle = 10;
    virtual QString to_string() {
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
struct SF_Param_Statistical_Outlier_Filter : public SF_Param_Filter<PointType> {
    int _k = 25;
    float _std_mult = 3.5;
    int _iterations = 15;
    virtual QString to_string() {
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
    virtual QString to_string() {
        QString str = "The radius outlier filter with parameters (_min_pts = ";
        str.append(QString::number(_min_Pts));
        str.append("; radius = ");
        str.append(QString::number(_radius));
        str.append(") is started.");
        return str;
    }


};

#endif // SF_ABSTRACT_PARAM_H
