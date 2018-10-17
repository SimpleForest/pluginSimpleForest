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
}

SF_StepStatisticalOutlierRemoval::~SF_StepStatisticalOutlierRemoval() {

}

QString SF_StepStatisticalOutlierRemoval::getStepDescription() const {
    return tr("Statistical Outlier Filter");
}

QString SF_StepStatisticalOutlierRemoval::getStepDetailledDescription() const {
    return tr("Statistical Outlier Filter - Frontend to PCL filter. This step detects noise by analysing the density of each points neighborhood. Regions containing a small amount of point are "
              "likely to be tagged as noise. Can be repeated iteretivaly. A really high number of iterations (~50) combined with a high standard deviation multiplier (>= 4) performs most robust.");
}

QString SF_StepStatisticalOutlierRemoval::getStepURL() const {
    return tr("http://pointclouds.org/documentation/tutorials/statistical_outlier.php");
}

CT_VirtualAbstractStep* SF_StepStatisticalOutlierRemoval::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_StepStatisticalOutlierRemoval(dataInit);
}

QStringList SF_StepStatisticalOutlierRemoval::getStepRISCitations() const {
    QStringList _risCitationList;
    _risCitationList.append(getRISCitationSimpleTree());
    _risCitationList.append(getRISCitationPCL());
    return _risCitationList;
}

void SF_StepStatisticalOutlierRemoval::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *resModel = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
    assert(resModel != NULL);
    resModel->setZeroOrMoreRootGroup();
    resModel->addGroupModel("", DEF_IN_GRP_CLUSTER, CT_AbstractItemGroup::staticGetType(),
                             tr("Group to be denoised"),
                             "",
                             CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resModel->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED,
                            CT_Scene::staticGetType(),
                            tr("Cloud to be denoised"));
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
                                   _numberPoints,
                                   _choiceNumberPoints);
    configDialog->addText("Low resulted clouds are affected more.");
}

void SF_StepStatisticalOutlierRemoval::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(resModelw != NULL) {
        resModelw->addGroupModel(DEF_IN_GRP_CLUSTER,
                                 _outGrp,
                                 new CT_StandardItemGroup(),
                                 tr ("Statistical Outlier Filter"));
        resModelw->addItemModel(_outGrp,
                                _outCloud,
                                new CT_Scene(),
                                tr("Cloud"));
        resModelw->addItemModel(_outGrp,
                                _outNoise,
                                new CT_Scene(),
                                tr("Noise"));
    }
}

void SF_StepStatisticalOutlierRemoval::adaptParametersToExpertLevel() {
    if(!_isExpert) {
        _k = 9;
        _iterations = 15;
        if(_choiceNumberPoints == _few) {
            _std_mult = 4;
        } else if(_choiceNumberPoints == _intermediate) {
            _std_mult = 3;
        } else if(_choiceNumberPoints == _many){
            _std_mult = 2.5;
        }
    }
}


void SF_StepStatisticalOutlierRemoval::writeOutputPerScence(CT_ResultGroup* outResult,
                                                            size_t i) {
    SF_ParamStatisticalOutlierFilter<SF_Point> param = _paramList.at(i);
    std::vector<CT_PointCloudIndexVector *> outputIndexList = createOutputVectors(param._sizeOutput);
    createOutputIndices(outputIndexList,
                        param._outputIndices,
                        param._itemCpyCloudIn);
    CT_StandardItemGroup *filterGrp = new CT_StandardItemGroup(_outGrp.completeName(),
                                                               outResult);
    param._grpCpyGrp->addGroup(filterGrp);
    addSceneToFilterGrp(filterGrp,
                        outResult,
                        outputIndexList[0],
                        _outCloud.completeName());
    addSceneToFilterGrp(filterGrp,
                        outResult,
                        outputIndexList[1],
                        _outNoise.completeName());
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
    QFuture<void> future = QtConcurrent::map(_paramList,
                                             SF_StatisticalOutlierRemovalAdapter());
    setProgressByFuture(future,10,85);
    writeOutput(outResult);
    writeLogger();
    _paramList.clear();
}

void SF_StepStatisticalOutlierRemoval::writeLogger() {
    if(!_paramList.empty()) {
        auto strList = _paramList[0].toStringList();
        for(auto &str : strList) {
            PS_LOG->addMessage(LogInterface::info,
                               LogInterface::step,
                               str);
        }
        size_t filtered = 0;
        size_t total = 0;
        for(auto const &param : _paramList) {
            auto vector = param._outputIndices;
            for(auto i : vector) {
                total++;
                filtered += static_cast<size_t> (i);
            }
        }
        auto str2 = _paramList[0].toFilterString(total,
                                                    filtered);
        PS_LOG->addMessage(LogInterface::info,
                           LogInterface::step,
                           str2);
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
