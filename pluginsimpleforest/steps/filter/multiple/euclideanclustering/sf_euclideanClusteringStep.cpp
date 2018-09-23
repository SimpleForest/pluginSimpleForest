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

#include "sf_euclideanClusteringStep.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

SF_EuclideanClusteringStep::SF_EuclideanClusteringStep(CT_StepInitializeData &dataInit):
    SF_AbstractFilterMultipleStep(dataInit) {

}

SF_EuclideanClusteringStep::~SF_EuclideanClusteringStep() {

}

QString SF_EuclideanClusteringStep::getStepDescription() const {
    return tr("Euclidean Clustering");
}

QString SF_EuclideanClusteringStep::getStepDetailledDescription() const {
    return tr("Performs an Euclidean Clustering Operation.");
}

QString SF_EuclideanClusteringStep::getStepURL() const {
    return tr("");
}

CT_VirtualAbstractStep* SF_EuclideanClusteringStep::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_EuclideanClusteringStep(dataInit);
}

QStringList SF_EuclideanClusteringStep::getStepRISCitations() const {
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


void SF_EuclideanClusteringStep::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *resModel = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
    resModel->setZeroOrMoreRootGroup();
    resModel->addGroupModel("", DEF_IN_GRP_CLUSTER, CT_AbstractItemGroup::staticGetType(), tr("Input Cloud Group"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resModel->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, CT_Scene::staticGetType(), tr("Point Cloud"));
    resModel->addGroupModel("", DEF_IN_SCENE, CT_AbstractItemGroup::staticGetType(), tr("Input Scene Group"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resModel->addItemModel(DEF_IN_SCENE, DEF_IN_SCENE_CLOUD, CT_Scene::staticGetType(), tr("Input Scene"));
}

void SF_EuclideanClusteringStep::createPostConfigurationDialog() {
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addDouble("The cloud is firstly downscaled with voxel size  ", " (m).", 0.01, 10, 4, _voxelSize);
    configDialog->addDouble("Than an euclidean clustering routine is performed with threshold  ", " (m)." , 0.01, 10, 4, _euclideanDistance);
    configDialog->addInt("A cluster has to contain at minimum ", " points.", 1, 9999, _minPts);
    createPostConfigurationDialogCitation(configDialog);
}

void SF_EuclideanClusteringStep::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(resModelw != NULL) {
        resModelw->addGroupModel(DEF_IN_SCENE, _outGrpCluster, new CT_StandardItemGroup(), tr ("Euclidean Clustering") );
        resModelw->addItemModel(_outGrpCluster, _outCloudCluster, new CT_Scene(), tr("Tree seeds"));
    }
}

void SF_EuclideanClusteringStep::compute() {
    const QList<CT_ResultGroup*> &outResultList = getOutResultList();
    CT_ResultGroup * outResult = outResultList.at(0);
    identifyAndRemoveCorruptedScenes(outResult);

    CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCL(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<size_t> indices;
    bool first = true;
    Eigen::Vector3d centerOfMass;

    while(!isStopped() && outResIt.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) outResIt.next();
        const CT_AbstractItemDrawableWithPointCloud* ctCloud =
                (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
        if(first) {
            centerOfMass = ctCloud->getCenterCoordinate();
            first = false;
        }
        CT_PointIterator iter(ctCloud->getPointCloudIndex());
        while(iter.hasNext() && ! isStopped()) {
            iter.next();
            size_t index = iter.currentGlobalIndex();
            CT_Point ctp = iter.currentPoint();
            indices.push_back(index);
            pcl::PointXYZI p;
            p.x = ctp[0] - centerOfMass[0];
            p.y = ctp[1] - centerOfMass[1];
            p.z = ctp[2] - centerOfMass[2];
            cloudPCL->push_back(p);
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCLDownscaled(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (cloudPCL);
    sor.setLeafSize (_voxelSize, _voxelSize, _voxelSize);
    sor.filter (*cloudPCLDownscaled);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (_euclideanDistance);
    ec.setMinClusterSize (1);
    ec.setMaxClusterSize (std::numeric_limits<int>::max());
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloudPCLDownscaled);
    ec.extract (clusterIndices);

    int index = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it) {
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
          cloudPCLDownscaled->points[*pit].intensity = index;
      index++;
    }

    std::vector<CT_PointCloudIndexVector *> indexVec;
    for(size_t i = 0; i < clusterIndices.size(); i++) {
        CT_PointCloudIndexVector *mergedClouds = new CT_PointCloudIndexVector();
        indexVec.push_back(mergedClouds);
    }

    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZI>);
    kdtree->setInputCloud (cloudPCLDownscaled);
    for(size_t i = 0; i < cloudPCL->points.size(); i++) {
        pcl::PointXYZI point = cloudPCL->points[i];
        std::vector<int>   pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        if ( kdtree->nearestKSearch (point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
            int index = cloudPCLDownscaled->points[pointIdxNKNSearch[0] ].intensity;
            indexVec[index]->addIndex(indices[i]);
        }
    }

    CT_ResultGroupIterator outResItOut(outResult,
                                       this,
                                       DEF_IN_SCENE);
    while(!isStopped() && outResItOut.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) outResItOut.next();
        for(size_t i = 0; i < indexVec.size(); i++) {
            if(indexVec[i]->size() > _minPts ) {
                CT_StandardItemGroup* cloudGrp = new CT_StandardItemGroup(_outGrpCluster.completeName(),
                                                                          outResult);
                group->addGroup(cloudGrp);
                indexVec[i]->setSortType(CT_PointCloudIndexVector::SortedInAscendingOrder);
                CT_Scene* outScene = new CT_Scene(_outCloudCluster.completeName(),
                                                  outResult,
                                                  PS_REPOSITORY->registerPointCloudIndex(indexVec[i]));
                outScene->updateBoundingBox();
                cloudGrp->addItemDrawable(outScene);
            }
        }
    }
}

void SF_EuclideanClusteringStep::createParamList(CT_ResultGroup *outResult) {
    adaptParametersToExpertLevel();
    CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_SCENE);
    while(!isStopped() && outResIt.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) outResIt.next();
        const CT_AbstractItemDrawableWithPointCloud* ct_cloud =
                (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
        SF_ParamEuclideanClustering<SF_PointNormal> param;
        param._log = PS_LOG;
        param._cellSize = _voxelSize;
        param._euclideanDistance = _euclideanDistance;
        param._minSize = _minPts;
        param._itemCpyCloudIn = ct_cloud;
        param._grpCpyGrp = group;
        _paramList.append(param);
    }
}
