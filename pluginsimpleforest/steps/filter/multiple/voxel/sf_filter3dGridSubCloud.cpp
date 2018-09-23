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

#include "sf_filter3dGridSubCloud.h"

#include <algorithm>

SF_Filter3dGridSubCloud::SF_Filter3dGridSubCloud(CT_StepInitializeData &data_init):
    SF_AbstractFilterMultipleStep(data_init) {

}

SF_Filter3dGridSubCloud::~SF_Filter3dGridSubCloud() {

}

QString SF_Filter3dGridSubCloud::getStepDescription() const {
    return tr("Voxelises the cloud into a 3d Grid.");
}

QString SF_Filter3dGridSubCloud::getStepDetailledDescription() const {
    return tr("Voxelises the cloud into a 3d Grid. Improves various filters.");
}

QString SF_Filter3dGridSubCloud::getStepURL() const {
    return tr("");
}

CT_VirtualAbstractStep* SF_Filter3dGridSubCloud::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_Filter3dGridSubCloud(dataInit);
}

QStringList SF_Filter3dGridSubCloud::getStepRISCitations() const {
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
    return _RIS_citation_list;
}

void SF_Filter3dGridSubCloud::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *res_model = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
    res_model->setZeroOrMoreRootGroup();
    res_model->addGroupModel("", DEF_IN_GRP_CLUSTER);
    res_model->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, CT_Scene::staticGetType(), tr("Point Cloud"));
}

void SF_Filter3dGridSubCloud::createPostConfigurationDialog() {
    CT_StepConfigurableDialog *config_dialog = newStandardPostConfigurationDialog();
    config_dialog->addDouble("The x,y,z voxelsize size of the output clouds ",
                             " ",
                             0.1,
                             10,
                             4,
                             _voxel_size);
    createPostConfigurationDialogCitation(config_dialog);
}

void SF_Filter3dGridSubCloud::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *res_modelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(res_modelw != NULL) {
        res_modelw->addGroupModel(DEF_IN_GRP_CLUSTER,
                                  _outGrp,
                                  new CT_StandardItemGroup(),
                                  tr ("voxel") );
        res_modelw->addGroupModel(_outGrp,
                                  _outGrpCluster,
                                  new CT_StandardItemGroup(),
                                  tr ("cluster") );
        res_modelw->addItemModel(_outGrpCluster,
                                 _outCloudCluster,
                                 new CT_Scene(),
                                 tr("cloud"));
    }
}

CT_Grid3D_Sparse<int> * SF_Filter3dGridSubCloud::createGrid3dFromScene(const CT_Scene *ctCloud,
                                                                           double voxelSize) {
    Eigen::Vector3f min = getMin(ctCloud);
    Eigen::Vector3f max = getMax(ctCloud);
    CT_Grid3D_Sparse<int>* hit_grid = CT_Grid3D_Sparse<int>::createGrid3DFromXYZCoords(NULL,
                                                                                       NULL,
                                                                                       min(0),
                                                                                       min(1),
                                                                                       min(2),
                                                                                       max(0),
                                                                                       max(1),
                                                                                       max(2),
                                                                                       voxelSize,
                                                                                       -2,
                                                                                       -1);
    return hit_grid;
}

void SF_Filter3dGridSubCloud::createGridCluster(int& val,
                                                      std::vector<CT_PointCloudIndexVector *> &clusters,
                                                      const CT_Point &point,
                                                      CT_Grid3D_Sparse<int>* hit_grid) {
    val = static_cast<int>(clusters.size());
    hit_grid->setValueAtXYZ(point(0),point(1),point(2), val);
    CT_PointCloudIndexVector * cluster = new CT_PointCloudIndexVector();
    clusters.push_back(cluster);
}

void SF_Filter3dGridSubCloud::createGridClusterIfNeeded(int &val,
                                                                std::vector<CT_PointCloudIndexVector *> &clusters,
                                                                const CT_Point &point,
                                                                CT_Grid3D_Sparse<int> *hit_grid) {
    if(val < 0) {
        createGridCluster(val, clusters, point, hit_grid);
    }
}

void SF_Filter3dGridSubCloud::addPointToGridCluster(CT_Grid3D_Sparse<int>* hit_grid,
                                                            std::vector<CT_PointCloudIndexVector *> &clusters,
                                                            CT_PointIterator &it) {

    const CT_Point &point = it.next().currentPoint();
    int val = hit_grid->valueAtXYZ(point(0),point(1),point(2));
    createGridClusterIfNeeded(val, clusters, point, hit_grid);
    size_t index_ct = it.currentGlobalIndex();
    clusters.at(val)->addIndex(index_ct);
}

void SF_Filter3dGridSubCloud::addCloudToGridCluster(const CT_Scene* ct_cloud,
                                                            std::vector<CT_PointCloudIndexVector *>& clusters) {
    CT_Grid3D_Sparse<int>* hit_grid = createGrid3dFromScene(ct_cloud,_voxel_size);
    const CT_AbstractPointCloudIndex *point_index = ct_cloud->getPointCloudIndex();
    CT_PointIterator it(point_index);
    while(it.hasNext()) {
        addPointToGridCluster(hit_grid, clusters, it);
    }
}

void SF_Filter3dGridSubCloud::compute() {
    const QList<CT_ResultGroup*> &out_result_list = getOutResultList();
    CT_ResultGroup * out_result = out_result_list.at(0);
    identifyAndRemoveCorruptedScenes(out_result);
    CT_ResultGroupIterator out_res_it(out_result, this, DEF_IN_GRP_CLUSTER);
    while(!isStopped() && out_res_it.hasNext()) {
        std::vector<CT_PointCloudIndexVector *> clusters;
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) out_res_it.next();
        const CT_Scene* ct_cloud = (const CT_Scene*) group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
        addCloudToGridCluster(ct_cloud, clusters);
        std::sort(clusters.begin(),clusters.end(),sfCompareCTCloudsBySize);
        writeOutput(out_result, clusters, group);
    }
}
