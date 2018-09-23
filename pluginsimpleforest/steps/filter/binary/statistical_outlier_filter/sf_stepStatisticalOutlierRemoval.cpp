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

#include "sf_stepStatisticalOutlierRemoval.h"
#include "steps/filter/binary/statistical_outlier_filter/sf_statisticalOutlierRemovalAdapter.h"
#include <QtConcurrent/QtConcurrent>

SF_StepStatisticalOutlierRemoval::SF_StepStatisticalOutlierRemoval(CT_StepInitializeData &dataInit):
    SF_AbstractFilterBinaryStep(dataInit) {
    _pointDensities.append(_less);
    _pointDensities.append(_intermediate);
    _pointDensities.append(_many);
}

SF_StepStatisticalOutlierRemoval::~SF_StepStatisticalOutlierRemoval() {

}

QString SF_StepStatisticalOutlierRemoval::getStepDescription() const {
    return tr("Statistical Outlier Removal - Frontend to PCL filter");
}

QString SF_StepStatisticalOutlierRemoval::getStepDetailledDescription() const {
    return tr("Statistical Outlier Removal - Frontend to PCL filter. This step detects noise by analysing the density of each points neighborhood. Regions containing a small amount of point are "
              "likely to be tagged as noise. Can be repeated iteretivaly. A really high number of iterations (~50) combined with a high standard deviation multiplier (>= 4) performs most robust.");
}

QString SF_StepStatisticalOutlierRemoval::getStepURL() const {
    return tr("http://pointclouds.org/documentation/tutorials/statistical_outlier.php");
}

CT_VirtualAbstractStep* SF_StepStatisticalOutlierRemoval::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_StepStatisticalOutlierRemoval(dataInit);
}

QStringList SF_StepStatisticalOutlierRemoval::getStepRISCitations() const {
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

void SF_StepStatisticalOutlierRemoval::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *res_model = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
    assert(res_model != NULL);
    res_model->setZeroOrMoreRootGroup();
    res_model->addGroupModel("", DEF_IN_GRP_CLUSTER, CT_AbstractItemGroup::staticGetType(), tr("Point Cloud Grp In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    res_model->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, CT_Scene::staticGetType(), tr("Point Cloud"));
}

void SF_StepStatisticalOutlierRemoval::createPostConfigurationDialogExpert(CT_StepConfigurableDialog *configDialog) {
    configDialog->addInt("Looks for each point at its ",
                         " closest neighbors",
                         1,
                         1000,
                         _k);
    configDialog->addText("The average distance to the neighbor points is computed for each point." );
    configDialog->addDouble("Assuming a normal distribution for all distances, all points further away from the mean distance than ",
                            " ",
                            0.1,
                            10,
                            4,
                            _std_mult);
    configDialog->addText("times the standarddeviation are removed. As the standard distribution parameters change, the procedure can be repeated multiple times." );
    configDialog->addInt("Please select the number of ",
                         " iterations for the procedure",
                         1,
                         100,
                         _iterations);
}

void SF_StepStatisticalOutlierRemoval::createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *configDialog) {
    configDialog->addStringChoice("Choose how many points should be removed",
                                   "",
                                   _pointDensities,
                                   _choicePointDensity);
    configDialog->addText("Low resulted clouds are affected more.");
}

void SF_StepStatisticalOutlierRemoval::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(resModelw != NULL) {
        resModelw->addGroupModel(DEF_IN_GRP_CLUSTER,
                                 _outGrp,
                                 new CT_StandardItemGroup(),
                                 tr ("statistical outlier removal") );
        resModelw->addGroupModel(_outGrp,
                                 _outGrpCloud,
                                 new CT_StandardItemGroup(),
                                 tr ("filtered") );
        resModelw->addGroupModel(_outGrp,
                                 _outGrpNoise,
                                 new CT_StandardItemGroup(),
                                 tr ("noise") );
        resModelw->addItemModel(_outGrpCloud,
                                _outCloud,
                                new CT_Scene(),
                                tr("cloud"));
        resModelw->addItemModel(_outGrpNoise,
                                _outNoise,
                                new CT_Scene(),
                                tr("cloud"));
    }
}

void SF_StepStatisticalOutlierRemoval::adaptParametersToExpertLevel() {
    if(!_isExpert) {
        if(_choicePointDensity == _less) {
            _k = 9;
            _std_mult = 4;
            _iterations = 15;
        } else if(_choicePointDensity == _intermediate) {
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


void SF_StepStatisticalOutlierRemoval::writeOutputPerScence(CT_ResultGroup* outResult,
                                                            size_t i) {
    SF_ParamStatisticalOutlierFilter<SF_Point> param = _paramList.at(i);
    std::vector<CT_PointCloudIndexVector *> output_index_list = createOutputVectors(param._sizeOutput);
    createOutputIndices(output_index_list,
                        param._outputIndices,
                        param._itemCpyCloudIn);
    CT_StandardItemGroup* filter_grp = new CT_StandardItemGroup(_outGrp.completeName(),
                                                                outResult);
    param._grpCpyGrp->addGroup(filter_grp);
    addSceneInSubgrpToGrp(filter_grp,
                          outResult,
                          output_index_list[0],
                          _outCloud.completeName(),
                          _outGrpCloud.completeName());
    addSceneInSubgrpToGrp(filter_grp,
                          outResult,
                          output_index_list[1],
                          _outNoise.completeName(),
                          _outGrpNoise.completeName());
}

void SF_StepStatisticalOutlierRemoval::writeOutput(CT_ResultGroup* outResult) {
    size_t size = _paramList.size();
    for(size_t i = 0; i < size; i ++) {
        writeOutputPerScence(outResult, i);
    }
}

void SF_StepStatisticalOutlierRemoval::compute() {
    const QList<CT_ResultGroup*> &outResultList = getOutResultList();
    CT_ResultGroup * outResult = outResultList.at(0);
    identifyAndRemoveCorruptedScenes(outResult);
    createParamList(outResult);
    writeLogger();
    QFuture<void> future = QtConcurrent::map(_paramList,
                                             SF_StatisticalOutlierRemovalAdapter());
    setProgressByFuture(future,10,85);
    writeOutput(outResult);
}

void SF_StepStatisticalOutlierRemoval::writeLogger() {
    if(!_paramList.empty()) {
        QString str = _paramList[0].toString();
        PS_LOG->addMessage(LogInterface::info, LogInterface::step, str);
    }
}

void SF_StepStatisticalOutlierRemoval::createParamList(CT_ResultGroup * outResult) {
    adaptParametersToExpertLevel();
    CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
    while(!isStopped() && outResIt.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) outResIt.next();
        const CT_AbstractItemDrawableWithPointCloud* ct_cloud =
                (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
        SF_ParamStatisticalOutlierFilter<SF_Point> param;
        param._log = PS_LOG;
        param._iterations = _iterations;
        param._k = _k;
        param._stdMult = _std_mult;
        param._sizeOutput = 2;
        param._itemCpyCloudIn = ct_cloud;
        param._grpCpyGrp = group;
        _paramList.append(param);
    }
}
