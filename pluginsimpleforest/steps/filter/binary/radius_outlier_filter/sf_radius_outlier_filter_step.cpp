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
#include "sf_radius_outlier_filter_step.h"
#include "steps/filter/binary/radius_outlier_filter/sf_radius_outlier_filter_adapter.h"
#include <QtConcurrent/QtConcurrent>

SF_Radius_Outlier_Filter_Step::SF_Radius_Outlier_Filter_Step(CT_StepInitializeData &data_init): SF_Abstract_Filter_Binary_Step(data_init) {    
    _non_expert_level.append(_less);
    _non_expert_level.append(_intermediate);
    _non_expert_level.append(_many);
}

SF_Radius_Outlier_Filter_Step::~SF_Radius_Outlier_Filter_Step() {

}

QString SF_Radius_Outlier_Filter_Step::getStepDescription() const {
    return tr("Radius Outlier Removal");
}

QString SF_Radius_Outlier_Filter_Step::getStepDetailledDescription() const {
    return tr("Radius Outlier Removal - Frontend to PCL filter. Points with not enough neighbors in a supported range are eliminated.");
}

QString SF_Radius_Outlier_Filter_Step::getStepURL() const {
    return tr("http://pointclouds.org/documentation/tutorials/remove_outliers.php");
}

CT_VirtualAbstractStep* SF_Radius_Outlier_Filter_Step::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_Radius_Outlier_Filter_Step(dataInit);
}

QStringList SF_Radius_Outlier_Filter_Step::getStepRISCitations() const {
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
void SF_Radius_Outlier_Filter_Step::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *res_model = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
    assert(res_model != NULL);
    res_model->setZeroOrMoreRootGroup();
    res_model->addGroupModel("", DEF_IN_GRP);
    res_model->addItemModel(DEF_IN_GRP, DEF_IN_CLOUD, CT_Scene::staticGetType(), tr("Point Cloud"));
}

void SF_Radius_Outlier_Filter_Step::createPostConfigurationDialog() {
    CT_StepConfigurableDialog *config_dialog = newStandardPostConfigurationDialog();
    if(!_is_expert) {
        config_dialog->addStringChoice("Choose how many points should be removed","",_non_expert_level, _choice);
        config_dialog->addText("Low resulted clouds are affected more.");
    } else {
        config_dialog->addDouble("Looks for each point at its numbers of neighbors in range ", "",0.01,0.1,3,_radius );
        config_dialog->addInt("A point is eliminated if it contains less than.", " Points",  2, 1000,_min_Pts);
    }

}

void SF_Radius_Outlier_Filter_Step::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *res_modelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(res_modelw != NULL) {
        res_modelw->addGroupModel(DEF_IN_GRP, _out_grp, new CT_StandardItemGroup(), tr ("radius outlier removal") );
        res_modelw->addGroupModel(_out_grp, _out_grp_cloud, new CT_StandardItemGroup(), tr ("filtered") );
        res_modelw->addGroupModel(_out_grp, _out_grp_noise, new CT_StandardItemGroup(), tr ("noise") );
        res_modelw->addItemModel(_out_grp_cloud, _out_cloud, new CT_Scene(), tr("cloud"));
        res_modelw->addItemModel(_out_grp_noise, _out_noise, new CT_Scene(), tr("cloud"));
    }
}

void SF_Radius_Outlier_Filter_Step::write_output_per_scence(CT_ResultGroup* out_result, size_t i) {
    SF_Param_Radius_Outlier_Filter<SF_Point> param = _param_list.at(i);
    std::vector<CT_PointCloudIndexVector *> output_index_list = create_output_vectors(param._size_output);
    create_output_indices(output_index_list, param._output_indices, param._itemCpy_cloud_in);
    CT_StandardItemGroup* filter_grp = new CT_StandardItemGroup( _out_grp.completeName(), out_result);
    param._grpCpy_grp->addGroup(filter_grp);
    add_scene_in_subgrp_to_grp(filter_grp, _out_cloud.completeName(),_out_grp_cloud.completeName(), out_result, output_index_list[0]);
    add_scene_in_subgrp_to_grp(filter_grp, _out_noise.completeName(), _out_grp_noise.completeName(), out_result, output_index_list[1]);
}

void SF_Radius_Outlier_Filter_Step::write_output(CT_ResultGroup* out_result) {
    size_t size = _param_list.size();
    for(size_t i = 0; i < size; i ++) {
        write_output_per_scence(out_result, i);
    }
}

void SF_Radius_Outlier_Filter_Step::compute() {
    const QList<CT_ResultGroup*> &out_result_list = getOutResultList();
    CT_ResultGroup * out_result = out_result_list.at(0);
    identify_and_remove_corrupted_scenes(out_result);
    create_param_list(out_result);
    write_logger();
    QFuture<void> future = QtConcurrent::map(_param_list, SF_Radius_Outlier_Filter_Adapter() );
    set_progress_by_future(future,10,85);
    write_output(out_result);
}

void SF_Radius_Outlier_Filter_Step::write_logger() {
    if(!_param_list.empty()) {
        QString str = _param_list[0].to_string();
        PS_LOG->addMessage(LogInterface::info, LogInterface::step, str);
    }
}

void SF_Radius_Outlier_Filter_Step::adapt_parameters_to_expert_level() {
    if(!_is_expert) {
        _radius = 0.03;
        if(_choice == _less) {
            _min_Pts = 5;
        } else if(_choice == _intermediate) {
            _min_Pts= 18;
        } else {
            _min_Pts = 40;
        }
    }
}

void SF_Radius_Outlier_Filter_Step::create_param_list(CT_ResultGroup * out_result) {
    adapt_parameters_to_expert_level();
    CT_ResultGroupIterator out_res_it(out_result, this, DEF_IN_GRP);
    while(!isStopped() && out_res_it.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) out_res_it.next();
        const CT_AbstractItemDrawableWithPointCloud* ct_cloud = (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD);
        SF_Param_Radius_Outlier_Filter<SF_Point> param;
        param._log = PS_LOG;
        param._radius = _radius;
        param._min_Pts = _min_Pts;
        param._size_output = 2;
        param._itemCpy_cloud_in = ct_cloud;
        param._grpCpy_grp = group;
        _param_list.append(param);
    }
}
