/****************************************************************************

 Copyright (C) 2017-2019 Dr. Jan Hackenberg, free software developer
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

#include "sf_djikstra.h"

#include "boost/tuple/tuple.hpp"

#include <pcl/segmentation/extract_clusters.h>

SF_Djikstra::SF_Djikstra() {}

void
SF_Djikstra::setCloud(const SF_CloudNormal::Ptr& cloud)
{
  m_cloud = cloud;
}

SF_CloudNormal::Ptr
SF_Djikstra::cloud() const
{
  return m_cloud;
}

void
SF_Djikstra::setRange(float range)
{
  m_range = range;
}

void
SF_Djikstra::computeStartIndex()
{
  float minZ = std::numeric_limits<float>::max();
  int index = 0;
  m_startIndex = index;
  for (SF_PointNormal point : m_cluster->points) {
    if (point.z < minZ) {
      minZ = point.z;
      m_startIndex = index;
    }
    index++;
  }
}
void
SF_Djikstra::initialize()
{
  initializeKDTree();
  computeStartIndex();
  for (SF_PointNormal& point : m_cluster->points) {
    point.intensity = std::numeric_limits<float>::max();
    point.normal_x = 0;
    point.normal_y = 1;
    point.normal_z = 0;
  }
  SF_PointNormal& point = m_cluster->points[m_startIndex];
  point.intensity = 0;
  point.normal_x = 0;
  point.normal_y = 0;
  point.normal_z = 1;
}

void
SF_Djikstra::initializeKDTree()
{
  m_kdTree.reset(new pcl::search::KdTree<SF_PointNormal>());
  m_kdTree->setInputCloud(m_cluster);
}

void
SF_Djikstra::computeCluster()
{
  initialize();
  SF_GraphDjikstra cloudGraph;
  SF_GraphDjikstra djikstraGraph;
  size_t cloudSize = m_cluster->size();
  for (size_t indexSource = 0; indexSource < cloudSize; indexSource++) {
    std::vector<int> indices;
    std::vector<float> distancesNeighbors;
    if (m_kdTree->radiusSearch(m_cluster->points[indexSource], m_range, indices, distancesNeighbors)) {
      if (indices.empty()) {
        continue;
      }
      for (size_t indexInternal = 0; indexInternal < indices.size(); indexInternal++) {
        size_t indexTarget = indices[indexInternal];
        float distance = std::sqrt(distancesNeighbors[indexInternal]);
        boost::add_edge(indexSource, indexTarget, Weight(distance), cloudGraph);
        try {
        } catch (...) {
          std::cout << "distance" << distance << std::endl;
          std::cout << "indexSource" << indexSource << std::endl;
          std::cout << "indexTarget" << indexTarget << std::endl;
        }
      }
    }
  }
  std::vector<VertexDjikstra> cloudPredecessorVertices(boost::num_vertices(cloudGraph));
  std::vector<float> cloudDistances(boost::num_vertices(cloudGraph));
  VertexDjikstra start = boost::vertex(m_startIndex, cloudGraph);
  boost::dijkstra_shortest_paths(
    cloudGraph,
    start,
    boost::distance_map(boost::make_iterator_property_map(cloudDistances.begin(), get(boost::vertex_index, cloudGraph)))
      .predecessor_map(boost::make_iterator_property_map(cloudPredecessorVertices.begin(), get(boost::vertex_index, cloudGraph))));
  boost::graph_traits<SF_GraphDjikstra>::vertex_iterator vi, vend;
  std::vector<std::string> names(boost::num_vertices(cloudGraph));
  VertexIndexMap indexMap = get(boost::vertex_index, cloudGraph);
  for (boost::tie(vi, vend) = boost::vertices(cloudGraph); vi != vend; ++vi) {
    SF_PointNormal& targetPoint = m_cluster->points[*vi];
    SF_PointNormal& sourcePoint = m_cluster->points[cloudPredecessorVertices[*vi]];
    auto distBetween = SF_Math<float>::distancef(sourcePoint.getVector3fMap(), targetPoint.getVector3fMap());
    boost::add_edge(cloudPredecessorVertices[*vi], *vi, Weight(distBetween), djikstraGraph);
    names.at(indexMap[*vi]) = *vi;
    names.at(indexMap[cloudPredecessorVertices[*vi]]) = cloudPredecessorVertices[*vi];
  }
  SF_DjikstraVisitor visitorDijkstra(names, m_cluster);
  boost::depth_first_search(djikstraGraph, boost::visitor(visitorDijkstra).root_vertex(m_startIndex));
  *m_cloudProcessed += *m_cluster;
}

void
SF_Djikstra::compute()
{
  m_cloudProcessed.reset(new SF_CloudNormal);

  pcl::search::KdTree<SF_PointNormal>::Ptr tree(new pcl::search::KdTree<SF_PointNormal>);
  tree->setInputCloud(m_cloud);
  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<SF_PointNormal> ec;
  ec.setClusterTolerance(m_range); // 2cm
  ec.setMinClusterSize(2);
  ec.setMaxClusterSize(std::numeric_limits<int>::max());
  ec.setSearchMethod(tree);
  ec.setInputCloud(m_cloud);
  ec.extract(clusterIndices);

//  m_cluster = m_cloud;
//  computeCluster();
  for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
    pcl::PointCloud<SF_PointNormal>::Ptr cloudCluster(new pcl::PointCloud<SF_PointNormal>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
      cloudCluster->points.push_back(m_cloud->points[*pit]);
    }
    m_cluster = cloudCluster;
    computeCluster();
  }
  *m_cloud = *m_cloudProcessed;
}
