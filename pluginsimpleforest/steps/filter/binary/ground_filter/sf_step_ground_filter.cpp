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
#include "sf_step_ground_filter.h"

#include "sf_step_ground_filter_adapter.h"
#include <QtConcurrent/QtConcurrent>

SF_Step_Ground_Filter::SF_Step_Ground_Filter(CT_StepInitializeData &data_init): SF_Abstract_Filter_Binary_Step(data_init) {
    _non_expert_level.append(_less);
    _non_expert_level.append(_intermediate);
    _non_expert_level.append(_many);
}

SF_Step_Ground_Filter::~SF_Step_Ground_Filter() {

}

QString SF_Step_Ground_Filter::getStepDescription() const {
    return tr("Ground Filter");
}

QString SF_Step_Ground_Filter::getStepDetailledDescription() const {
    return tr("Ground Filter - This Filter estimates for each point the normal. The angle between the normal vector and the z axis is computed. "
              "If the angle is small, the point is detected as ground, if large the point is considered non ground.");
}

QString SF_Step_Ground_Filter::getStepURL() const {
    return tr("https://www.youtube.com/watch?v=5i6_Rtv-xEw");
}

CT_VirtualAbstractStep* SF_Step_Ground_Filter::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_Step_Ground_Filter(dataInit);
}

QStringList SF_Step_Ground_Filter::getStepRISCitations() const {
    QStringList _RIS_citation_list;
    _RIS_citation_list.append(QString("TY  - JOUR\n"
                                      "T1  - SimpleTree - an efficient open source tool to build tree models from TLS clouds\n"
                                      "A1  - Hackenberg, Jan\n"
                                      "A1  - Spiecker, Heinrich\n"
                                      "A1  - Calders, Kim\n"
                                      "A1  - Disney, Mathias\n"
                                      "A1  - Raumonen, Pasi\n"
                                      "JO  - Forests\n"
                                      "VL  - 6\n"
                                      "IS  - 11\n"
                                      "SP  - 4245\n"
                                      "EP  - 4294\n"
                                      "Y1  - 2015\n"
                                      "PB  - Multidisciplinary Digital Publishing Institute\n"
                                      "UL  - http://www.simpletree.uni-freiburg.de/\n"
                                      "ER  - \n"));


    _RIS_citation_list.append(QString("TY  - CONF\n"
                                      "T1  - 3d is here: Point cloud library (pcl)\n"
                                      "A1  - Rusu, Radu Bogdan\n"
                                      "A1  - Cousins, Steve\n"
                                      "JO  - Robotics and Automation (ICRA), 2011 IEEE International Conference on\n"
                                      "SP  - 1\n"
                                      "EP  - 4\n"
                                      "SN  - 1612843859\n"
                                      "Y1  - 2011\n"
                                      "PB  - IEEE\n"
                                      "UL  - http://pointclouds.org/documentation/tutorials/statistical_outlier.php\n"
                                      "ER  - \n"));
    return _RIS_citation_list;
}

void SF_Step_Ground_Filter::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *res_model = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
    assert(res_model != NULL);
    res_model->setZeroOrMoreRootGroup();
    res_model->addGroupModel("", DEF_IN_GRP_CLUSTER, CT_AbstractItemGroup::staticGetType(), tr("Point Cloud Grp In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    res_model->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, CT_Scene::staticGetType(), tr("Point Cloud"));
}

void SF_Step_Ground_Filter::createPostConfigurationDialogExpert(CT_StepConfigurableDialog *config_dialog) {
    config_dialog->addDouble("First the cloud is downscaled to a voxel size of  ",   " (m). " , 0.015,0.1,3,_voxel_size );
    config_dialog->addDouble("For each of the downscaled points its normal is computed with a range search of  ", "  (m). " , 0.025,0.2,3,_radius_normal );
    config_dialog->addDouble("The angle for each point between the normal and the adjusted z axis is computed and is not allowed to deviate more than ", " " , 0.5,180,1,_angle );
    config_dialog->addText("degrees.");
    config_dialog->addDouble("The x-component of the adjusted z-axis", " " , 0.01,1,2,_x );
    config_dialog->addDouble("The y-component of the adjusted z-axis", " " , 0.01,1,2,_y );
    config_dialog->addDouble("The z-component of the adjusted z-axis", " " , 0.01,1,2,_z );
}

void SF_Step_Ground_Filter::createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *config_dialog) {
    config_dialog->addStringChoice("Choose how many points should be removed","",_non_expert_level, _choice);
    config_dialog->addText("For bended trees select a weaker filter level.");
}

void SF_Step_Ground_Filter::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *res_modelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(res_modelw != NULL) {
        res_modelw->addGroupModel(DEF_IN_GRP_CLUSTER, _out_grp, new CT_StandardItemGroup(), tr ("ground filtered removal") );
        res_modelw->addGroupModel(_out_grp, _out_grp_cloud, new CT_StandardItemGroup(), tr ("ground") );
        res_modelw->addGroupModel(_out_grp, _out_grp_noise, new CT_StandardItemGroup(), tr ("not ground") );
        res_modelw->addItemModel(_out_grp_cloud, _out_cloud, new CT_Scene(), tr("cloud"));
        res_modelw->addItemModel(_out_grp_noise, _out_noise, new CT_Scene(), tr("cloud"));
    }
}

void SF_Step_Ground_Filter::adapt_parameters_to_expert_level() {
    if(!_is_expert) {
        if(_choice == _less) {
            _x = 0;
            _y = 0;
            _z = 1;
            _angle = 15;
            _radius_normal = 0.04;
            _voxel_size = 0.015;
            _size_output = 2;
        } else if(_choice == _intermediate) {
            _x = 0;
            _y = 0;
            _z = 1;
            _angle = 30;
            _radius_normal = 0.04;
            _voxel_size = 0.015;
            _size_output = 2;
        } else {
            _x = 0;
            _y = 0;
            _z = 1;
            _angle = 45;
            _radius_normal = 0.04;
            _voxel_size = 0.015;
            _size_output = 2;
        }
    }
}

void SF_Step_Ground_Filter::write_output_per_scence(CT_ResultGroup* out_result, size_t i) {
    SF_Param_Ground_Filter<SF_Point_N> param = _param_list.at(i);
    std::vector<CT_PointCloudIndexVector *> output_index_list = create_output_vectors(param._size_output);
    create_output_indices(output_index_list, param._output_indices, param._itemCpy_cloud_in);
    CT_StandardItemGroup* filter_grp = new CT_StandardItemGroup( _out_grp.completeName(), out_result);
    param._grpCpy_grp->addGroup(filter_grp);
    add_scene_in_subgrp_to_grp(filter_grp, _out_cloud.completeName(),_out_grp_cloud.completeName(), out_result, output_index_list[0]);
    add_scene_in_subgrp_to_grp(filter_grp, _out_noise.completeName(), _out_grp_noise.completeName(), out_result, output_index_list[1]);
}

void SF_Step_Ground_Filter::write_output(CT_ResultGroup* out_result) {
    size_t size = _param_list.size();
    for(size_t i = 0; i < size; i ++) {
        write_output_per_scence(out_result, i);
    }
}

void SF_Step_Ground_Filter::compute() {
    const QList<CT_ResultGroup*> &out_result_list = getOutResultList();
    CT_ResultGroup * out_result = out_result_list.at(0);
    identify_and_remove_corrupted_scenes(out_result);
    create_param_list(out_result);
    write_logger();

    QFuture<void> future = QtConcurrent::map(_param_list,SF_Step_Ground_Filter_Adapter() );
    set_progress_by_future(future,10,85);
    write_output(out_result);
}

void SF_Step_Ground_Filter::write_logger() {
    if(!_param_list.empty()) {
        QString str = _param_list[0].to_string();
        PS_LOG->addMessage(LogInterface::info, LogInterface::step, str);
    }
}

void SF_Step_Ground_Filter::create_param_list(CT_ResultGroup * out_result) {
    adapt_parameters_to_expert_level();
    CT_ResultGroupIterator out_res_it(out_result, this, DEF_IN_GRP_CLUSTER);
    while(!isStopped() && out_res_it.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) out_res_it.next();
        const CT_AbstractItemDrawableWithPointCloud* ct_cloud = (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
        SF_Param_Ground_Filter<SF_Point_N> param;

        param._log = PS_LOG;
        param._x = _x;
        param._y = _y;
        param._z = _z;
        param._angle = _angle;
        param._radius_normal = _radius_normal;
        param._voxel_size = _voxel_size;
        param._size_output = 2;
        param._itemCpy_cloud_in = ct_cloud;
        param._grpCpy_grp = group;
        _param_list.append(param);
    }
}
