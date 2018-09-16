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

#include "sf_step_cut_cloud_above_dtm.h"
#include "sf_cut_above_dtm_adapter.h"

#include <QtConcurrent/QtConcurrent>

SF_Step_Cut_Cloud_Above_DTM::SF_Step_Cut_Cloud_Above_DTM(CT_StepInitializeData &data_init): SF_Abstract_Filter_Binary_Step(data_init) {
}

SF_Step_Cut_Cloud_Above_DTM::~SF_Step_Cut_Cloud_Above_DTM() {

}

QString SF_Step_Cut_Cloud_Above_DTM::getStepDescription() const {
    return tr("Cut Cloud Above DTM");
}

QString SF_Step_Cut_Cloud_Above_DTM::getStepDetailledDescription() const {
    return tr("Cut Cloud Above DTM - Takes an input Scene and a DTM model. Splits the scene into two clouds UPPER and LOWER."
              "The user selects a threshold height and each point with a height below the threshold is put into cloud "
              "LOWER, into UPPPER otherwise. ");
}

QString SF_Step_Cut_Cloud_Above_DTM::getStepURL() const {
    return tr("https://www.youtube.com/watch?v=5i6_Rtv-xEw");
}

CT_VirtualAbstractStep* SF_Step_Cut_Cloud_Above_DTM::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_Step_Cut_Cloud_Above_DTM(dataInit);
}

QStringList SF_Step_Cut_Cloud_Above_DTM::getStepRISCitations() const {
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

void SF_Step_Cut_Cloud_Above_DTM::createInResultModelListProtected() {
    CT_InResultModelGroup *resModelDTM = createNewInResultModel(DEF_IN_RESULT_DTM, tr("DTM"));
    assert(resModelDTM != NULL);
    resModelDTM->setZeroOrMoreRootGroup();
    resModelDTM->addGroupModel("", DEF_IN_DTMGRP, CT_AbstractItemGroup::staticGetType(), tr("DTM Grp In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resModelDTM->addItemModel(DEF_IN_DTMGRP, DEF_IN_DTM, CT_Image2D<float>::staticGetType(), tr("DTM"));
    CT_InResultModelGroupToCopy *res_model = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
    assert(res_model != NULL);
    res_model->setZeroOrMoreRootGroup();
    res_model->addGroupModel("", DEF_IN_GRP_CLUSTER, CT_AbstractItemGroup::staticGetType(), tr("Point Cloud Grp In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    res_model->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, CT_Scene::staticGetType(), tr("Point Cloud"));
}

void SF_Step_Cut_Cloud_Above_DTM::createPostConfigurationDialogExpert(CT_StepConfigurableDialog *config_dialog) {
    config_dialog->addDouble("Over the input cloud is iterated. For each point the height <b>h</b> above the DTM is computed. If <b>h</b> is lower than ",   " (m). " , -0.1, 50,2, _cutHeight );
    config_dialog->addText(" the point is put into the lower cloud, into the upper otherwise.");
}

void SF_Step_Cut_Cloud_Above_DTM::createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *config_dialog) {
}

void SF_Step_Cut_Cloud_Above_DTM::createPreConfigurationDialog() {
    _isExpert = true;
}

void SF_Step_Cut_Cloud_Above_DTM::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *res_modelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(res_modelw != NULL) {
        res_modelw->addGroupModel(DEF_IN_GRP_CLUSTER, _out_grp, new CT_StandardItemGroup(), tr ("cut above DTM") );
        res_modelw->addGroupModel(_out_grp, _out_grp_cloud, new CT_StandardItemGroup(), tr ("lower") );
        res_modelw->addGroupModel(_out_grp, _out_grp_noise, new CT_StandardItemGroup(), tr ("upper") );
        res_modelw->addItemModel(_out_grp_cloud, _out_cloud, new CT_Scene(), tr("lower cloud"));
        res_modelw->addItemModel(_out_grp_noise, _out_noise, new CT_Scene(), tr("upper cloud"));
    }
}

void SF_Step_Cut_Cloud_Above_DTM::adaptParametersToExpertLevel() {
}

void SF_Step_Cut_Cloud_Above_DTM::write_output_per_scence(CT_ResultGroup* out_result, size_t i) {
    SF_Param_DTM_Height<pcl::PointXYZ> param = _paramList.at(i);
    std::vector<CT_PointCloudIndexVector *> output_index_list = createOutputVectors(param._size_output);
    createOutputIndices(output_index_list, param._output_indices, param._itemCpyCloudIn);
    CT_StandardItemGroup* filter_grp = new CT_StandardItemGroup( _out_grp.completeName(), out_result);
    param._grpCpyGrp->addGroup(filter_grp);
    addSceneInSubgrpToGrp(filter_grp, _out_cloud.completeName(),_out_grp_cloud.completeName(), out_result, output_index_list[0]);
    addSceneInSubgrpToGrp(filter_grp, _out_noise.completeName(), _out_grp_noise.completeName(), out_result, output_index_list[1]);
}

void SF_Step_Cut_Cloud_Above_DTM::write_output(CT_ResultGroup* out_result) {
    size_t size = _paramList.size();
    for(size_t i = 0; i < size; i ++) {
        write_output_per_scence(out_result, i);
    }
}

void SF_Step_Cut_Cloud_Above_DTM::compute() {
    const QList<CT_ResultGroup*> &out_result_list = getOutResultList();
    CT_ResultGroup * out_result = out_result_list.at(0);
    identifyAndRemoveCorruptedScenes(out_result);
    create_param_list(out_result);
    QFuture<void> future = QtConcurrent::map(_paramList,SF_Step_Cut_Above_DTM_Adapter() );
    setProgressByFuture(future,10,85);
    writeLogger();
    write_output(out_result);
}

void SF_Step_Cut_Cloud_Above_DTM::writeLogger() {
    QString str = "The DTM has been cut above DTM.";
    PS_LOG->addMessage(LogInterface::info, LogInterface::step, str);
}

void SF_Step_Cut_Cloud_Above_DTM::create_param_list(CT_ResultGroup * out_result) {
    CT_ResultGroup * inDTMResult = getInputResults().at(0);
    CT_ResultItemIterator iterDTM(inDTMResult, this, DEF_IN_DTM);
    CT_Image2D<float> * dtm = (CT_Image2D<float> *) iterDTM.next();
    adaptParametersToExpertLevel();
    CT_ResultGroupIterator out_res_it(out_result, this, DEF_IN_GRP_CLUSTER);
    while(!isStopped() && out_res_it.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) out_res_it.next();
        const CT_AbstractItemDrawableWithPointCloud* ct_cloud = (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
        SF_Param_DTM_Height<pcl::PointXYZ> param;
        param._log = PS_LOG;
        param._dtmCT = dtm;
        param._cropHeight = _cutHeight;
        param._size_output = 2;
        param._itemCpyCloudIn = ct_cloud;
        param._grpCpyGrp = group;
        _paramList.append(param);
    }
}
