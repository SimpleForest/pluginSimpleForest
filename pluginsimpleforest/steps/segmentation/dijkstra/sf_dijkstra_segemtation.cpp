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

#include "sf_dijkstra_segemtation.h"
#include <pcl/cloud/segmentation/dijkstra/sf_dijkstra.h>

SF_Dijkstra_Segmentation_Step::SF_Dijkstra_Segmentation_Step(CT_StepInitializeData &data_init): SF_SegmentationStep(data_init) {
}

SF_Dijkstra_Segmentation_Step::~SF_Dijkstra_Segmentation_Step() {
}

QString SF_Dijkstra_Segmentation_Step::getStepDescription() const {
    return tr("Dikstra Based Tree Segmentation from seeds");
}

QString SF_Dijkstra_Segmentation_Step::getStepDetailledDescription() const {
    return tr("The step takes vegetation points and seed clusters as input. All clusters are tagged with an own id and zero distance, remaining non cluster points are"
              "initialized with infinity distance. Then a competitive dijkstra is applied. Later all points are tagged with the id from the seed cluster they are connected to."
              "Before the routine the cloud can be scaled in z-axis to enable easier vertical growth of the dijkstra routine."
              "If points remain not reached by the dijkstra they are alligned to the closest pre cluster tree by nearest neighbor check.");
}

QString SF_Dijkstra_Segmentation_Step::getStepURL() const {
    return tr("");
}

CT_VirtualAbstractStep* SF_Dijkstra_Segmentation_Step::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_Dijkstra_Segmentation_Step(dataInit);
}

QStringList SF_Dijkstra_Segmentation_Step::getStepRISCitations() const {
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


void SF_Dijkstra_Segmentation_Step::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *res_model = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
    res_model->setZeroOrMoreRootGroup();
    res_model->addGroupModel("", DEF_IN_GRP_CLUSTER, CT_AbstractItemGroup::staticGetType(), tr("Vegetation Group"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    res_model->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, CT_Scene::staticGetType(), tr("Vegetation Cloud"));
    res_model->addGroupModel("", DEF_IN_SCENE, CT_AbstractItemGroup::staticGetType(), tr("Seed Group"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    res_model->addItemModel(DEF_IN_SCENE, DEF_IN_SCENE_CLOUD, CT_Scene::staticGetType(), tr("Seed Cloud"));
}

void SF_Dijkstra_Segmentation_Step::createPostConfigurationDialog() {
    CT_StepConfigurableDialog *config_dialog = newStandardPostConfigurationDialog();    
    config_dialog->addDouble("The cloud is scaled along the z axis with a factor of  ", ".", 0.1, 1, 1, _zFactor);
    config_dialog->addDouble("The cloud is downscaled with voxel size  ", " (m).", 0.01, 10, 4, _voxelSize);
    config_dialog->addDouble("Than a Dijkstra based Segmentation is performed with neighbors connection with range  ", " (m)." , 0.01, 10, 4, _euclideanDistance);
    config_dialog->addText("Voxel size should be at minimum 2 or 3 times smaller than Dijkstra range.");
    createPostConfigurationDialogCitation(config_dialog);
}

void SF_Dijkstra_Segmentation_Step::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *res_modelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(res_modelw != NULL) {
        res_modelw->addItemModel(DEF_IN_SCENE, _outCloudCluster, new CT_Scene(), tr("Dijsktra Segmented"));
    }
}

void SF_Dijkstra_Segmentation_Step::compute() {
    const QList<CT_ResultGroup*> &out_result_list = getOutResultList();
    CT_ResultGroup * out_result = out_result_list.at(0);
    identifyAndRemoveCorruptedScenes(out_result);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCL(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCLDownscaled(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustersPCL(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustersPCLDownscaled(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<size_t> indices;
    std::vector<size_t> indicesCluster;
    std::vector<CT_PointCloudIndexVector *> indexVec;

    createPCLCloud(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, out_result, cloudPCL, indices, _zFactor);
    createPCLCloud(DEF_IN_SCENE, DEF_IN_SCENE_CLOUD, out_result, clustersPCL, indicesCluster, _zFactor);
    downscale(cloudPCL, _voxelSize, cloudPCLDownscaled);
    downscale(clustersPCL, _voxelSize, clustersPCLDownscaled);

    SF_Dijkstra dijk(cloudPCLDownscaled, clustersPCLDownscaled, _euclideanDistance);
    int size = getClusterNumber(cloudPCLDownscaled);
    initializeIndexVec(size, indexVec);
    fillIndexVec(cloudPCL, indexVec, indices, cloudPCLDownscaled,_voxelSize*3);

    CT_ResultGroupIterator resultGrpIterator(out_result, this, DEF_IN_SCENE);
    int i = 0;
    while(!isStopped() && resultGrpIterator.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) resultGrpIterator.next();
        indexVec[i]->setSortType(CT_PointCloudIndexVector::SortedInAscendingOrder);
        CT_Scene* outScene = new CT_Scene(_outCloudCluster.completeName(), out_result, PS_REPOSITORY->registerPointCloudIndex(indexVec[i]) );
        outScene->updateBoundingBox();
        group->addItemDrawable(outScene);
        i++;
    }
}

void SF_Dijkstra_Segmentation_Step::createParamList(CT_ResultGroup * out_result) {

}
