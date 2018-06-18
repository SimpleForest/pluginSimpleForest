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

#include "sf_step_statistical_outlier_removal.h"
#include "steps/filter/binary/statistical_outlier_filter/sf_statistical_outlier_removal_adapter.h"
#include <QtConcurrent/QtConcurrent>

SF_Step_Statistical_Outlier_Removal::SF_Step_Statistical_Outlier_Removal(CT_StepInitializeData &data_init): SF_Abstract_Filter_Binary_Step(data_init) {
    _non_expert_level.append(_less);
    _non_expert_level.append(_intermediate);
    _non_expert_level.append(_many);
}

SF_Step_Statistical_Outlier_Removal::~SF_Step_Statistical_Outlier_Removal() {

}

QString SF_Step_Statistical_Outlier_Removal::getStepDescription() const {
    return tr("Statistical Outlier Removal - Frontend to PCL filter");
}

QString SF_Step_Statistical_Outlier_Removal::getStepDetailledDescription() const {
    return tr("Statistical Outlier Removal - Frontend to PCL filter. This step detects noise by analysing the density of each points neighborhood. Regions containing a small amount of point are "
              "likely to be tagged as noise. Can be repeated iteretivaly. A really high number of iterations (~50) combined with a high standard deviation multiplier (>= 4) performs most robust.");
}

QString SF_Step_Statistical_Outlier_Removal::getStepURL() const {
    return tr("http://pointclouds.org/documentation/tutorials/statistical_outlier.php");
}

CT_VirtualAbstractStep* SF_Step_Statistical_Outlier_Removal::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_Step_Statistical_Outlier_Removal(dataInit);
}

QStringList SF_Step_Statistical_Outlier_Removal::getStepRISCitations() const {
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

void SF_Step_Statistical_Outlier_Removal::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *res_model = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
    assert(res_model != NULL);
    res_model->setZeroOrMoreRootGroup();
    res_model->addGroupModel("", DEF_IN_GRP, CT_AbstractItemGroup::staticGetType(), tr("Point Cloud Grp In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    res_model->addItemModel(DEF_IN_GRP, DEF_IN_CLOUD, CT_Scene::staticGetType(), tr("Point Cloud"));
}

void SF_Step_Statistical_Outlier_Removal::createPostConfigurationDialogExpert(CT_StepConfigurableDialog *config_dialog) {
    config_dialog->addInt("Looks for each point at its ", " closest neighbors",1,1000,_k );
    config_dialog->addText("The average distance to the neighbor points is computed for each point." );
    config_dialog->addDouble("Assuming a normal distribution for all distances, all points further away from the mean distance than ", " " , 0.1,10,4,_std_mult );
    config_dialog->addText("times the standarddeviation are removed. As the standard distribution parameters change, the procedure can be repeated multiple times." );
    config_dialog->addInt("Please select the number of ", " iterations for the procedure",1,100,_iterations );
}

void SF_Step_Statistical_Outlier_Removal::createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *config_dialog) {
    config_dialog->addStringChoice("Choose how many points should be removed","",_non_expert_level, _choice);
    config_dialog->addText("Low resulted clouds are affected more.");
}

void SF_Step_Statistical_Outlier_Removal::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *res_modelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(res_modelw != NULL) {
        res_modelw->addGroupModel(DEF_IN_GRP, _out_grp, new CT_StandardItemGroup(), tr ("statistical outlier removal") );
        res_modelw->addGroupModel(_out_grp, _out_grp_cloud, new CT_StandardItemGroup(), tr ("filtered") );
        res_modelw->addGroupModel(_out_grp, _out_grp_noise, new CT_StandardItemGroup(), tr ("noise") );
        res_modelw->addItemModel(_out_grp_cloud, _out_cloud, new CT_Scene(), tr("cloud"));
        res_modelw->addItemModel(_out_grp_noise, _out_noise, new CT_Scene(), tr("cloud"));
    }
}

void SF_Step_Statistical_Outlier_Removal::adapt_parameters_to_expert_level() {
    if(!_is_expert) {
        if(_choice == _less) {
            _k = 9;
            _std_mult = 4;
            _iterations = 15;
        } else if(_choice == _intermediate) {
            _k = 9;
            _std_mult = 3;
            _iterations = 15;
        } else {
            _k = 9;
            _std_mult = 2.5;
            _iterations = 15;
        }
    }
}


void SF_Step_Statistical_Outlier_Removal::write_output_per_scence(CT_ResultGroup* out_result, size_t i) {
    SF_Param_Statistical_Outlier_Filter<SF_Point> param = _param_list.at(i);
    std::vector<CT_PointCloudIndexVector *> output_index_list = create_output_vectors(param._size_output);
    create_output_indices(output_index_list, param._output_indices, param._itemCpy_cloud_in);
    CT_StandardItemGroup* filter_grp = new CT_StandardItemGroup( _out_grp.completeName(), out_result);
    param._grpCpy_grp->addGroup(filter_grp);
    add_scene_in_subgrp_to_grp(filter_grp, _out_cloud.completeName(),_out_grp_cloud.completeName(), out_result, output_index_list[0]);
    add_scene_in_subgrp_to_grp(filter_grp, _out_noise.completeName(), _out_grp_noise.completeName(), out_result, output_index_list[1]);
}

void SF_Step_Statistical_Outlier_Removal::write_output(CT_ResultGroup* out_result) {
    size_t size = _param_list.size();
    for(size_t i = 0; i < size; i ++) {
        write_output_per_scence(out_result, i);
    }
}

void SF_Step_Statistical_Outlier_Removal::compute() {
    const QList<CT_ResultGroup*> &out_result_list = getOutResultList();
    CT_ResultGroup * out_result = out_result_list.at(0);
    identify_and_remove_corrupted_scenes(out_result);
    create_param_list(out_result);
    write_logger();
    QFuture<void> future = QtConcurrent::map(_param_list,SF_Statistical_Outlier_Removal_Adapter() );
    set_progress_by_future(future,10,85);
    write_output(out_result);
}

void SF_Step_Statistical_Outlier_Removal::write_logger() {
    if(!_param_list.empty()) {
        QString str = _param_list[0].to_string();
        PS_LOG->addMessage(LogInterface::info, LogInterface::step, str);
    }
}

void SF_Step_Statistical_Outlier_Removal::create_param_list(CT_ResultGroup * out_result) {
    adapt_parameters_to_expert_level();
    CT_ResultGroupIterator out_res_it(out_result, this, DEF_IN_GRP);
    while(!isStopped() && out_res_it.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) out_res_it.next();
        const CT_AbstractItemDrawableWithPointCloud* ct_cloud = (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD);
        SF_Param_Statistical_Outlier_Filter<SF_Point> param;
        param._log = PS_LOG;
        param._iterations = _iterations;
        param._k = _k;
        param._std_mult = _std_mult;
        param._size_output = 2;
        param._itemCpy_cloud_in = ct_cloud;
        param._grpCpy_grp = group;
        _param_list.append(param);
    }
}
