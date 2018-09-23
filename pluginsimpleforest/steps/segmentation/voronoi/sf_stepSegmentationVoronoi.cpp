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

#include "sf_stepSegmentationVoronoi.h"

SF_StepSegmentationVoronoi::SF_StepSegmentationVoronoi(CT_StepInitializeData &dataInit):
    SF_AbstractStepSegmentation(dataInit) {
}

SF_StepSegmentationVoronoi::~SF_StepSegmentationVoronoi() {
}

QString SF_StepSegmentationVoronoi::getStepDescription() const {
    return tr("Voronoi Based Tree Segmentation from seeds");
}

QString SF_StepSegmentationVoronoi::getStepDetailledDescription() const {
    return tr("Rather than computing the Voronoi regions for seeds, distance for the input points to the clustered seeds are computed."
              "If a closest point pair between input and seeds has distance smaller than a threshold, the input point gets the ID of its closest seed point.");
}

QString SF_StepSegmentationVoronoi::getStepURL() const {
    return tr("");
}

CT_VirtualAbstractStep* SF_StepSegmentationVoronoi::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_StepSegmentationVoronoi(dataInit);
}

QStringList SF_StepSegmentationVoronoi::getStepRISCitations() const {
    QStringList _risCitationList;
    _risCitationList.append(QString("TY  - JOUR\n"
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

    _risCitationList.append(QString("TY  - CONF\n"
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
    return _risCitationList;
}


void SF_StepSegmentationVoronoi::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *resModel = createNewInResultModelForCopy(DEF_IN_RESULT,
                                                                           tr("Point Cloud"));
    resModel->setZeroOrMoreRootGroup();
    resModel->addGroupModel("", DEF_IN_GRP_CLUSTER,
                             CT_AbstractItemGroup::staticGetType(),
                             tr("Vegetation Group"),
                             "",
                             CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resModel->addItemModel(DEF_IN_GRP_CLUSTER,
                            DEF_IN_CLOUD_SEED,
                            CT_Scene::staticGetType(),
                            tr("Vegetation Cloud"));
    resModel->addGroupModel("",
                             DEF_IN_SCENE,
                             CT_AbstractItemGroup::staticGetType(),
                             tr("Seed Group"),
                             "",
                             CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resModel->addItemModel(DEF_IN_SCENE,
                            DEF_IN_SCENE_CLOUD,
                            CT_Scene::staticGetType(),
                            tr("Seed Cloud"));
}

void SF_StepSegmentationVoronoi::createPostConfigurationDialog() {
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addDouble("The cloud is scaled along the z axis with a factor of  ",
                             ".",
                             0.1,
                             1,
                             1,
                             _zFactor);
    configDialog->addDouble("If an input point has a distance to its closest seed point smaller   ",
                             " (m).",
                             0.01,
                             10,
                             4,
                             _euclideanDistance);
    configDialog->addText("it gets the according seed segment ID.");
    createPostConfigurationDialogCitation(configDialog);
}

void SF_StepSegmentationVoronoi::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(resModelw != NULL) {
        resModelw->addItemModel(DEF_IN_SCENE,
                                _outCloudCluster,
                                new CT_Scene(),
                                tr("Voronoi Segmented"));
    }
}

void SF_StepSegmentationVoronoi::compute() {
    const QList<CT_ResultGroup*> &outResultList = getOutResultList();
    CT_ResultGroup * outResult = outResultList.at(0);
    identifyAndRemoveCorruptedScenes(outResult);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCL(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustersPCL(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<size_t> indices;
    std::vector<size_t> indicesCluster;
    std::vector<CT_PointCloudIndexVector *> indexVec;
    createPCLCloud(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED,
                   outResult,
                   cloudPCL,
                   indices,
                   _zFactor);
    createPCLCloud(DEF_IN_SCENE,
                   DEF_IN_SCENE_CLOUD,
                   outResult,
                   clustersPCL,
                   indicesCluster,
                   _zFactor);
    int size = getClusterNumber(clustersPCL);
    initializeIndexVec(size,
                       indexVec);
    fillIndexVec(cloudPCL,
                 indexVec,
                 indices,
                 clustersPCL,
                 _euclideanDistance);
    CT_ResultGroupIterator resultGrpIterator(outResult,
                                             this,
                                             DEF_IN_SCENE);
    int i = 0;
    while(!isStopped() && resultGrpIterator.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) resultGrpIterator.next();
        indexVec[i]->setSortType(CT_PointCloudIndexVector::SortedInAscendingOrder);
        CT_Scene* outScene = new CT_Scene(_outCloudCluster.completeName(),
                                          outResult,
                                          PS_REPOSITORY->registerPointCloudIndex(indexVec[i]) );
        outScene->updateBoundingBox();
        group->addItemDrawable(outScene);
        i++;
    }
}

void SF_StepSegmentationVoronoi::createParamList(CT_ResultGroup * outResult) {
    outResult->getName();
}
