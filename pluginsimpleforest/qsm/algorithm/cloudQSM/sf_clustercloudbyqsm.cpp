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

#include "sf_clustercloudbyqsm.h"

#include <cmath>

std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> SF_ClusterCloudByQSM::clusters() const
{
    return m_clusters;
}

SF_ClusterCloudByQSM::SF_ClusterCloudByQSM(SF_CloudToModelDistanceParameters params, std::shared_ptr<SF_ModelQSM> qsm, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, int numCltrs):
    m_params(params), m_qsm(qsm), m_cloud(cloud), m_numClstrs(numCltrs)
{

}

void SF_ClusterCloudByQSM::compute()
{
    SF_CloudToModelDistanceParameters params = m_params;
    params._method = SF_CLoudToModelDistanceMethod::FIRSTMOMENTUMORDERMSAC;
    Sf_CloudToModelDistance cmd(m_qsm, m_cloud, params);
    std::vector<float> distances = cmd.getCloudToModelDistances();
    size_t index = 0;
    std::vector<pcl::PointXYZINormal> points;
    std::for_each(distances.begin(),distances.end(), [this, &index, &points](float distance){
        pcl::PointXYZINormal point = m_cloud->points[index++];
        point.intensity = distance;
        if(point.intensity!= std::numeric_limits<float>::max()){
            points.push_back(point);
        }
    });
    std::sort(points.begin(),points.end(),[](const pcl::PointXYZINormal &first, const pcl::PointXYZINormal &second){
        return first.intensity > second.intensity;
    });
    for(size_t i = 0; i < m_numClstrs; i++) {
        m_clusters.push_back(pcl::PointCloud<pcl::PointXYZINormal>::Ptr (new pcl::PointCloud<pcl::PointXYZINormal> ));
    }
    for(size_t index = 0; index < points.size(); index++) {
        size_t clusterIndex = index/std::ceil(points.size()/m_numClstrs);
        clusterIndex = std::min(clusterIndex, static_cast<size_t>(m_numClstrs-1) );

        m_clusters[clusterIndex]->points.push_back(points[index]);
    }
}

