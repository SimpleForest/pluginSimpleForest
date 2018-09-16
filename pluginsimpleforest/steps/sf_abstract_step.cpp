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

#include "sf_abstract_step.h"
#include <pcl/sample_consensus/method_types.h>
#include "qsm/algorithm/distance/sf_cloud_to_model_distance.h"

void  SF_AbstractStep::recursiveRemoveIfEmpty(CT_AbstractItemGroup *parent,
                                              CT_AbstractItemGroup *group) {
    if(parent != NULL) {
        parent->removeGroup(group);
        if(parent->isEmpty()) {
            recursiveRemoveIfEmpty(parent->parentGroup(), parent);
        }
    } else {
        ((CT_ResultGroup*)group->result())-> removeGroupSomethingInStructure(group);
    }
}

void SF_AbstractStep::setProgressByFuture(QFuture<void> &future,
                                          float percentageIntervalStart,
                                          float percentageIntervalSize) {
    float progressMin = future.progressMinimum();
    float progressSize = future.progressMaximum()-progressMin;
    while(!future.isFinished()) {
        setProgress(percentageIntervalStart + (percentageIntervalSize*(future.progressValue() - progressMin)/progressSize ));
    }
}

CT_Scene* SF_AbstractStep::mergeIndices(CT_ResultGroup *outResult,
                                        CT_StandardItemGroup* root,
                                        const QString defInnGrp,
                                        const QString defInCloud) {
    CT_ResultGroupIterator outResIt(outResult, this, defInnGrp);
    CT_PointCloudIndexVector *mergedClouds = new CT_PointCloudIndexVector();
    mergedClouds->setSortType(CT_AbstractCloudIndex::NotSorted);
    std::vector<size_t> indices;
    while(!isStopped() && outResIt.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) outResIt.next();
        const CT_AbstractItemDrawableWithPointCloud* ctCloud = (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, defInCloud);
        CT_PointIterator iter(ctCloud->getPointCloudIndex());
        while(iter.hasNext() && ! isStopped()) {
            iter.next();
            size_t index = iter.currentGlobalIndex();
            indices.push_back(index);
        }
    }
    std::sort(indices.begin(), indices.end());
    for(size_t i = 0; i < indices.size(); i++) {
        mergedClouds->addIndex(indices.at(i));
    }
    mergedClouds->setSortType(CT_PointCloudIndexVector::SortedInAscendingOrder);
    CT_Scene* scene = new CT_Scene(defInCloud , outResult, PS_REPOSITORY->registerPointCloudIndex(mergedClouds));
    return scene;
}

void SF_AbstractStep::createPreConfigurationDialog() {
    CT_StepConfigurableDialog *configDialog = newStandardPreConfigurationDialog();
    configDialog->addBool("Uncheck to deactivate parameterization possibilities of this step. Only recommended for beginners","","expert", _isExpert);
}

Eigen::Vector3f SF_AbstractStep::getMin(const CT_Scene* ctCloud) {
    Eigen::Vector3f min(ctCloud->minX(), ctCloud->minY(), ctCloud->minZ());
    return min;
}

Eigen::Vector3f SF_AbstractStep::getMax(const CT_Scene *ctCloud) {
    Eigen::Vector3f max(ctCloud->maxX(), ctCloud->maxY(), ctCloud->maxZ());
    return max;
}

void SF_AbstractStep::identifyAndRemoveCorruptedScenes(CT_ResultGroup* outResult) {
    identifyCorruptedScenes(outResult);
    removeCorruptedScenes();
}


SF_AbstractStep::SF_AbstractStep(CT_StepInitializeData &dataInit):
                 CT_AbstractStep(dataInit) {
    _SF_methodList.push_back(_RANSAC);
    _SF_methodList.push_back(_LMEDS);
    _SF_methodList.push_back(_MSAC);
    _SF_methodList.push_back(_RRANSAC);
    _SF_methodList.push_back(_RMSAC);
    _SF_methodList.push_back(_MLESAC);
    _SF_methodList.push_back(_PROSAC);

    _CMD_methodList.push_back(_ZEROMOMENTUMORDER);
    _CMD_methodList.push_back(_FIRSTMOMENTUMORDERMSAC);
    _CMD_methodList.push_back(_FIRSTMOMENTUMORDER);
    _CMD_methodList.push_back(_SECONDMOMENTUMORDERMSAC);
    _CMD_methodList.push_back(_SECONDMOMENTUMORDER);
}

void SF_AbstractStep::checkIsEmpty(CT_StandardItemGroup* group,
                                   const CT_AbstractItemDrawableWithPointCloud* ctCloud) {
    if(ctCloud->getPointCloudIndex()->size() <=  0) {
        _groupsToBeRemoved.push_back(group);
    }
}

void SF_AbstractStep::checkIsNullOrEmpty(const CT_AbstractItemDrawableWithPointCloud* ctCloud,
                                         CT_StandardItemGroup* group) {
    if(ctCloud!= NULL){
        checkIsEmpty(group, ctCloud);
    } else {
        _groupsToBeRemoved.push_back(group);
    }
}

void SF_AbstractStep::checkGrpAndCloud(CT_StandardItemGroup* group) {
    if(group!=NULL) {
        const CT_AbstractItemDrawableWithPointCloud* ctCloud =
       (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
        checkIsNullOrEmpty(ctCloud, group);
    } else {
        _groupsToBeRemoved.push_back(group);
    }
}

void SF_AbstractStep::identifyCorruptedScenes( CT_ResultGroup* outResult,
                                               int progress) {
    _groupsToBeRemoved.clear();
    CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
    while(!isStopped() && outResIt.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) outResIt.next();
        checkGrpAndCloud(group);
    }    
    setProgress(progress);
}

void SF_AbstractStep::createPostConfigurationDialog() {
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    if(!_isExpert) {
        createPostConfigurationDialogBeginner(configDialog);
    } else {
        createPostConfigurationDialogExpert(configDialog);
    }
    createPostConfigurationDialogCitation(configDialog);
}

void SF_AbstractStep::createPostConfigurationDialogCitation(CT_StepConfigurableDialog *configDialog) {
    configDialog->addEmpty();
    configDialog->addText(QObject::tr("For general usage of the SimpleForest plugin please cite the following:")
                            ,         "Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P.");
    configDialog->addText("",         "<em>SimpleTree - An Efficient Open Source Tool to Build Tree Models from TLS Clouds.</em>");
    configDialog->addText("",         "Forests <b>2015</b>, 6, 4245-4294.");
    configDialog->addEmpty();
    createPostConfigurationDialogCitationSecond(configDialog);
}

void SF_AbstractStep::removeCorruptedScenes(int progress) {
    while(!_groupsToBeRemoved.isEmpty()) {
        CT_AbstractItemGroup *group = _groupsToBeRemoved.takeLast();
        recursiveRemoveIfEmpty(group->parentGroup(), group);
    }    
    setProgress(progress);
}


std::vector<CT_PointCloudIndexVector *> SF_AbstractStep::createOutputVectors(size_t numberOutput) {
    std::vector<CT_PointCloudIndexVector *> result;
    for(size_t i = 0; i < numberOutput; i++) {
        CT_PointCloudIndexVector * vec = new CT_PointCloudIndexVector();
        result.push_back(vec);
    }
    return result;
}

void SF_AbstractStep::createOutputIndex(std::vector<CT_PointCloudIndexVector *> &indexVectors,
                                        const std::vector<int> &indices,
                                        size_t counter,
                                        CT_PointIterator & pointIt) {
    pointIt.next();
    size_t indexCt = pointIt.currentGlobalIndex();
    if(counter <indices.size()) {
        size_t indexCloud = indices.at(counter);
        if(indexCloud<indexVectors.size()) {
            indexVectors[indexCloud]->addIndex(indexCt);
        } else {
            qDebug() << "TODO void SF_Abstract_Step::create_output_index";
        }
    } else {
        qDebug() << "TODO void SF_Abstract_Step::create_output_index2";
    }
}

void SF_AbstractStep::createOutputIndices(std::vector<CT_PointCloudIndexVector *> &indexVectors,
                                          const std::vector<int> &indices,
                                          const CT_AbstractItemDrawableWithPointCloud * itemCpyCloudIn) {
    const CT_AbstractPointCloudIndex * pointCloudIndex = itemCpyCloudIn->getPointCloudIndex();
    CT_PointIterator pointIt(pointCloudIndex);
    size_t counter = 0;
    while(pointIt.hasNext()) {
        createOutputIndex(indexVectors, indices, counter++, pointIt);
    }
}

void SF_AbstractStep::writeLogger() {
    if(!_paramList.empty()) {
        QString str = _paramList[0].toString();
        PS_LOG->addMessage(LogInterface::info, LogInterface::step, str);
    }
}

