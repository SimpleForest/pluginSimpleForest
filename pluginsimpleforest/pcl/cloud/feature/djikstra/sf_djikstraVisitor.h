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

#ifndef SF_DJIKSTRAVISITOR_H
#define SF_DJIKSTRAVISITOR_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
#include <pcl/features/boost.h>

#include "pcl/cloud/feature/sf_abstractFeature.h"
#include "pcl/sf_math.h"

// see Computree class normalsestimator for other boost graph visitor implementations.

typedef boost::property<boost::edge_weight_t, float> Weight;
typedef boost::property<boost::vertex_name_t, int> Index;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Index, Weight> SF_GraphDjikstra;

typedef boost::graph_traits<SF_GraphDjikstra>::vertex_descriptor VertexDjikstra;
typedef boost::graph_traits<SF_GraphDjikstra>::edge_descriptor EdgeDjikstra;
typedef boost::property_map<SF_GraphDjikstra, boost::vertex_index_t>::type VertexIndexMap;
typedef boost::property_map<SF_GraphDjikstra, boost::vertex_index_t>::type WeightMap;

class SF_DjikstraVisitor : public boost::default_dfs_visitor
{
  std::vector<std::string> m_vertexNames;
  SF_CloudNormal::Ptr m_cloud;

public:
  SF_DjikstraVisitor(std::vector<std::string> vertexNames, SF_CloudNormal::Ptr cloud);
  template<typename Edge, typename Graph>
  void tree_edge(Edge e, const Graph& g) const
  {
    SF_PointNormal& sourcePoint = m_cloud->points[boost::source(e, g)];
    SF_PointNormal& targetPoint = m_cloud->points[boost::target(e, g)];
    sourcePoint.normal_x = targetPoint.x - sourcePoint.x;
    sourcePoint.normal_y = targetPoint.y - sourcePoint.y;
    sourcePoint.normal_z = targetPoint.z - sourcePoint.z;
    auto distBetween = SF_Math<float>::distancef(sourcePoint.getVector3fMap(), targetPoint.getVector3fMap());
    targetPoint.intensity = sourcePoint.intensity + distBetween;
  }
};

#endif // SF_DJIKSTRAVISITOR_H
