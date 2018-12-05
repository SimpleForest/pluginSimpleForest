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

#include "sf_stepSegemtationDijkstra.h"
#include <pcl/cloud/segmentation/dijkstra/sf_dijkstra.h>

#include "ct_itemdrawable/ct_pointsattributescolor.h"

SF_StepSegmentationDijkstra::SF_StepSegmentationDijkstra(CT_StepInitializeData &dataInit):
    SF_AbstractStepSegmentation(dataInit) {
}

SF_StepSegmentationDijkstra::~SF_StepSegmentationDijkstra() {
}

QString SF_StepSegmentationDijkstra::getStepDescription() const {
    return tr("Dikstra Based Tree Segmentation from seeds");
}

QString SF_StepSegmentationDijkstra::getStepDetailledDescription() const {
    return tr("The step takes vegetation points and seed clusters as input. All clusters are tagged with an own id and zero distance, remaining non cluster points are"
              "initialized with infinity distance. Then a competitive dijkstra is applied. Later all points are tagged with the id from the seed cluster they are connected to."
              "Before the routine the cloud can be scaled in z-axis to enable easier vertical growth of the dijkstra routine."
              "If points remain not reached by the dijkstra they are alligned to the closest pre cluster tree by nearest neighbor check.");
}

QString SF_StepSegmentationDijkstra::getStepURL() const {
    return tr("");
}

CT_VirtualAbstractStep* SF_StepSegmentationDijkstra::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_StepSegmentationDijkstra(dataInit);
}

QStringList SF_StepSegmentationDijkstra::getStepRISCitations() const {
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

    _RIS_citation_list.append(QString("TY  - JOUR\n"
                                      "T1  - STRUCTURING LASER-SCANNED TREES USING 3D MATHEMATICAL MORPHOLOGY\n"
                                      "A1  - Gorte, Ben\n"
                                      "A1  - Pfeifer, Norbert\n"
                                      "JO  - Section of Photogrammetry and Remote Sensing\n"
                                      "Y1  - 2004\n"
                                      "PB  - TU Delft\n"
                                      "UL  - http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.438.1610&rep=rep1&type=pdf\n"
                                      "ER  - \n"));

    _RIS_citation_list.append(QString("TY  - JOUR\n"
                                      "T1  - A note on two problems in connexion with graphs\n"
                                      "A1  - Dijkstra, Edsger W\n"
                                      "JO  - Numerische mathematik\n"
                                      "Y1  - 1959\n"
                                      "PB  - Springer\n"
                                      "UL  - https://link.springer.com/article/10.1007%2FBF01386390\n"
                                      "ER  - \n"));
    return _RIS_citation_list;
}


void SF_StepSegmentationDijkstra::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *resModel = createNewInResultModelForCopy(DEF_IN_RESULT,
                                                                           tr("Point Cloud"));
    resModel->setZeroOrMoreRootGroup();
    resModel->addGroupModel("",
                             DEF_IN_GRP_CLUSTER,
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

void SF_StepSegmentationDijkstra::createPostConfigurationDialog() {
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addDouble("The cloud is scaled along the z axis with a factor of  ",
                            ".",
                            0.1,
                            1,
                            1,
                            _zFactor);
    configDialog->addDouble("The cloud is downscaled with voxel size  ",
                            " (m).",
                            0.01,
                            10,
                            4,
                            _voxelSize);
    configDialog->addDouble("Then a Dijkstra based Segmentation is performed with neighbors connection with range  ",
                            " (m)." ,
                            0.01,
                            10,
                            4,
                            _euclideanDistance);
    configDialog->addText("Voxel size should be at minimum 2 or 3 times smaller than Dijkstra range.");
    createPostConfigurationDialogCitation(configDialog);
}

void SF_StepSegmentationDijkstra::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(resModelw != NULL) {
        resModelw->addItemModel(DEF_IN_SCENE,
                                _outCloudCluster,
                                new CT_Scene(),
                                tr("Dijsktra Segmented"));
        resModelw->addItemModel(DEF_IN_GRP_CLUSTER,
                                  m_outCloudItem,
                                  new CT_PointsAttributesColor(),
                                  tr("Dijkstra distance"));
    }
}

void SF_StepSegmentationDijkstra::compute() {
    const QList<CT_ResultGroup*> &outResultList = getOutResultList();
    CT_ResultGroup * outResult = outResultList.at(0);
    identifyAndRemoveCorruptedScenes(outResult);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCL(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCLDownscaled(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustersPCL(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustersPCLDownscaled(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<size_t> indices;
    std::vector<size_t> indicesCluster;
    std::vector<CT_PointCloudIndexVector *> indexVec;

    createPCLCloud(DEF_IN_GRP_CLUSTER,
                   DEF_IN_CLOUD_SEED,
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
    downscale(cloudPCL,
              _voxelSize,
              cloudPCLDownscaled);
    downscale(clustersPCL,
              _voxelSize,
              clustersPCLDownscaled);
    SF_Dijkstra djik(cloudPCLDownscaled,
                 clustersPCLDownscaled,
                 _euclideanDistance);
    int size = getClusterNumber(clustersPCL);
    initializeIndexVec(size, indexVec);
    fillIndexVec(cloudPCL,
                 indexVec,
                 indices,
                 cloudPCLDownscaled,
                 _voxelSize*3);



    CT_ColorCloudStdVector *_colors = new CT_ColorCloudStdVector(cloudPCL->points.size());
    std::vector<float> distances = djik.getDistances();
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZI>);
    kdtree->setInputCloud (cloudPCLDownscaled);
    float _maxDistance = djik.getMaxDistance();
    for(size_t i = 0; i < cloudPCL->points.size(); i++) {
        pcl::PointXYZI point = cloudPCL->points[i];
        CT_Color &col = _colors->colorAt(i);
        std::vector<int>   pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        if ( kdtree->nearestKSearch (point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
                float distance = distances[pointIdxNKNSearch[0] ];
                if(distance == 1000)
                {
                    col.r() = (0);
                    col.g() = (0);
                    col.b() = (255);
                }
                else
                {
                    float perc = (_maxDistance == 0) ? 0 : distance/_maxDistance;
                    perc = std::min(1.0f,perc);
                    col.r() = (std::abs(255*perc));
                    col.g() = (std::abs(255 - 255*perc));
                    col.b() = (0);
                }
        }
    }
    CT_ResultGroupIterator resultGrpIterator2(outResult,
                                             this,
                                             DEF_IN_GRP_CLUSTER);
    while(!isStopped() && resultGrpIterator2.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) resultGrpIterator2.next();
        const CT_AbstractItemDrawableWithPointCloud* ctCloud =
                (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);

        CT_PointsAttributesColor* colorAttribute = new CT_PointsAttributesColor(m_outCloudItem.completeName(),
                                                                                outResult,
                                                                                ctCloud->getPointCloudIndexRegistered(),
                                                                                _colors);
        group->addItemDrawable(colorAttribute);
    }


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

void SF_StepSegmentationDijkstra::createParamList(CT_ResultGroup * outResult) {
    outResult->getName();
}
