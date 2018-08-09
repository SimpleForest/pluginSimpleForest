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
#include "sf_filter_3d_grid_sub_cloud.h"

#include <algorithm>

SF_Filter_3d_Grid_Sub_Cloud::SF_Filter_3d_Grid_Sub_Cloud(CT_StepInitializeData &data_init): SF_Abstract_Filter_Multiple_Step(data_init) {

}

SF_Filter_3d_Grid_Sub_Cloud::~SF_Filter_3d_Grid_Sub_Cloud() {

}

QString SF_Filter_3d_Grid_Sub_Cloud::getStepDescription() const {
    return tr("Voxelises the cloud into a 3d Grid. ");
}

QString SF_Filter_3d_Grid_Sub_Cloud::getStepDetailledDescription() const {
    return tr("Voxelises the cloud into a 3d Grid. Improves various filters.");
}

QString SF_Filter_3d_Grid_Sub_Cloud::getStepURL() const {
    return tr("");
}

CT_VirtualAbstractStep* SF_Filter_3d_Grid_Sub_Cloud::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_Filter_3d_Grid_Sub_Cloud(dataInit);
}

QStringList SF_Filter_3d_Grid_Sub_Cloud::getStepRISCitations() const {
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

void SF_Filter_3d_Grid_Sub_Cloud::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *res_model = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
    //    assert(res_model != NULL);
    res_model->setZeroOrMoreRootGroup();
    res_model->addGroupModel("", DEF_IN_GRP_CLUSTER);
    res_model->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, CT_Scene::staticGetType(), tr("Point Cloud"));
}

void SF_Filter_3d_Grid_Sub_Cloud::createPostConfigurationDialog() {
    CT_StepConfigurableDialog *config_dialog = newStandardPostConfigurationDialog();
    config_dialog->addDouble("The x,y,z voxelsize size of the output clouds ", " " , 0.1,10,4,_voxel_size );
    createPostConfigurationDialogCitation(config_dialog);
}

void SF_Filter_3d_Grid_Sub_Cloud::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *res_modelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(res_modelw != NULL) {
        res_modelw->addGroupModel(DEF_IN_GRP_CLUSTER, _out_grp, new CT_StandardItemGroup(), tr ("voxel") );
        res_modelw->addGroupModel(_out_grp, _out_grp_cluster, new CT_StandardItemGroup(), tr ("cluster") );
        res_modelw->addItemModel(_out_grp_cluster, _out_cloud_cluster, new CT_Scene(), tr("cloud"));
    }
}

CT_Grid3D_Sparse<int> * SF_Filter_3d_Grid_Sub_Cloud::create_grid3d_from_scene(const CT_Scene* ct_cloud, double voxel_size) {
    Eigen::Vector3f min = get_min(ct_cloud);
    Eigen::Vector3f max = get_max(ct_cloud);
    CT_Grid3D_Sparse<int>* hit_grid = CT_Grid3D_Sparse<int>::createGrid3DFromXYZCoords( NULL,NULL,min(0),min(1),min(2)
                                                                                        ,max(0), max(1), max(2),voxel_size, -2,-1);
    return hit_grid;
}

void SF_Filter_3d_Grid_Sub_Cloud::create_grid_cluster(int& val, std::vector<CT_PointCloudIndexVector *> &clusters, const CT_Point &point, CT_Grid3D_Sparse<int>* hit_grid) {
    val = clusters.size();
    hit_grid->setValueAtXYZ(point(0),point(1),point(2), val);
    CT_PointCloudIndexVector * cluster = new CT_PointCloudIndexVector();
    clusters.push_back(cluster);
}

void SF_Filter_3d_Grid_Sub_Cloud::create_grid_cluster_if_needed(int &val, std::vector<CT_PointCloudIndexVector *> &clusters, const CT_Point &point, CT_Grid3D_Sparse<int> *hit_grid) {
    if(val < 0) {
        create_grid_cluster(val, clusters, point, hit_grid);
    }
}

void SF_Filter_3d_Grid_Sub_Cloud::add_point_to_grid_cluster( CT_Grid3D_Sparse<int>* hit_grid,
                                                             std::vector<CT_PointCloudIndexVector *> &clusters, CT_PointIterator &it) {

    const CT_Point &point = it.next().currentPoint();
    int val = hit_grid->valueAtXYZ(point(0),point(1),point(2));
    create_grid_cluster_if_needed(val, clusters, point, hit_grid);
    size_t index_ct = it.currentGlobalIndex();
    clusters.at(val)->addIndex(index_ct);
}

void SF_Filter_3d_Grid_Sub_Cloud::add_cloud_to_grid_cluster(const CT_Scene* ct_cloud, std::vector<CT_PointCloudIndexVector *>& clusters) {
    CT_Grid3D_Sparse<int>* hit_grid = create_grid3d_from_scene(ct_cloud,_voxel_size);
    const CT_AbstractPointCloudIndex *point_index = ct_cloud->getPointCloudIndex();
    CT_PointIterator it(point_index);
    while(it.hasNext()) {
        add_point_to_grid_cluster(hit_grid, clusters, it);
    }
}

void SF_Filter_3d_Grid_Sub_Cloud::compute() {
    const QList<CT_ResultGroup*> &out_result_list = getOutResultList();
    CT_ResultGroup * out_result = out_result_list.at(0);
    identify_and_remove_corrupted_scenes(out_result);
    CT_ResultGroupIterator out_res_it(out_result, this, DEF_IN_GRP_CLUSTER);
    while(!isStopped() && out_res_it.hasNext()) {
        std::vector<CT_PointCloudIndexVector *> clusters;
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) out_res_it.next();
        const CT_Scene* ct_cloud = (const CT_Scene*) group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
        add_cloud_to_grid_cluster(ct_cloud, clusters);
        std::sort(clusters.begin(),clusters.end(),sfCompareCTCloudsBySize);
        write_output(out_result, clusters, group);
    }
}
