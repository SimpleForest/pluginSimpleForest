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

#include "sf_segmentation_step.h"

SF_Segmentation_Step::SF_Segmentation_Step(CT_StepInitializeData &data_init): SF_Abstract_Filter_Multiple_Step(data_init) {
}

void SF_Segmentation_Step::initializeIndexVec(size_t size, std::vector<CT_PointCloudIndexVector *>& indexVec) {
    for(size_t i = 0; i < size; i++) {
        CT_PointCloudIndexVector *mergedClouds = new CT_PointCloudIndexVector();
        indexVec.push_back(mergedClouds);
    }
}

void SF_Segmentation_Step::createPCLCloud(const QString& clusterGrpStr, const QString& clusterStr, CT_ResultGroup* out_result,
                                          pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCL, std::vector<size_t>& indices, float factor) {
    CT_ResultGroupIterator out_res_it(out_result, this, clusterGrpStr);
    int clusterID = 0;
    while(!isStopped() && out_res_it.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) out_res_it.next();
        const CT_AbstractItemDrawableWithPointCloud* ct_cloud = (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, clusterStr);
        if(_first) {
            _centerOfMass = ct_cloud->getCenterCoordinate();
            _first = false;
        }
        CT_PointIterator iter(ct_cloud->getPointCloudIndex());
        while(iter.hasNext() && ! isStopped()) {
            iter.next();
            size_t index = iter.currentGlobalIndex();
            CT_Point ctp = iter.currentPoint();
            indices.push_back(index);
            pcl::PointXYZI p;
            p.x = ctp[0] - _centerOfMass[0];
            p.y = ctp[1] - _centerOfMass[1];
            p.z = (ctp[2] - _centerOfMass[2])*factor;
            p.intensity = clusterID;
            cloudPCL->push_back(p);
        }
        clusterID++;
    }
}

int SF_Segmentation_Step::getClusterNumber(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCLDownscaled) {
    int size = 0;
    for(int i = 0; i < cloudPCLDownscaled->points.size(); i++) {
        if(cloudPCLDownscaled->points[i].intensity + 1 > size) {
            size = cloudPCLDownscaled->points[i].intensity + 1;
        }
    }
    return size;
}

void SF_Segmentation_Step::downscale(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCL, float voxelSize, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCLDownscaled) {
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (cloudPCL);
    sor.setLeafSize (voxelSize, voxelSize, voxelSize);
    sor.filter (*cloudPCLDownscaled);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZI>);
    kdtree->setInputCloud (cloudPCL);
    for(size_t i = 0; i < cloudPCLDownscaled->points.size(); i++) {
        pcl::PointXYZI point = cloudPCLDownscaled->points[i];
        std::vector<int>   pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        if ( kdtree->nearestKSearch (point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
            cloudPCLDownscaled->points[i].intensity = cloudPCL->points[pointIdxNKNSearch[0] ].intensity;
        }
    }
}

void SF_Segmentation_Step::fillIndexVec(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCL, std::vector<CT_PointCloudIndexVector *>& indexVec,
                                        std::vector<size_t>& indices, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCLDownscaled, float maxRange) {
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZI>);
    kdtree->setInputCloud (cloudPCLDownscaled);
    for(size_t i = 0; i < cloudPCL->points.size(); i++) {
        pcl::PointXYZI point = cloudPCL->points[i];
        std::vector<int>   pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        if ( kdtree->nearestKSearch (point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
            if(std::sqrt(pointNKNSquaredDistance[0]) < maxRange ) {
                int index = cloudPCLDownscaled->points[pointIdxNKNSearch[0] ].intensity;
                if(index >= 0)
                    indexVec[index]->addIndex(indices[i]);
            }
        }
    }
}
