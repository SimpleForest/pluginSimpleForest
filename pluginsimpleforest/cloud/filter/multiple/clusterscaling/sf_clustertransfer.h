/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_clustertransfer.h is part of SimpleForest - a plugin for the
    Computree platform.

    SimpleForest is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SimpleForest is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with
    SimpleForest. If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/

#ifndef SF_CLUSTERTRANSFER_H
#define SF_CLUSTERTRANSFER_H

#include "../sf_abstractmultiplefilter.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <utility>

template<typename PointType>
class SF_ClusterTransfer : public SF_AbstractMultipleFilter
{
public:
  /**
   * @brief Standard constructor.
   */
  SF_ClusterTransfer();
  /**
   * @brief compute Does the actual transform of of \ref m_clusterIn to original
   * resoluted cloud.
   */
  void compute() override;

private:
  /**
   * @brief initialize creates \ref m_cloudInMerged out of \ref m_clusterIn.
   */
  void initialize();
  /**
   * @brief m_cloudInMerged All input clusters merged to one cloud.
   */
  pcl::PointCloud<PointType>::Ptr m_cloudInMerged;
  /**
   * @brief m_ClusterIndices For each point in \ref m_cloudInMerged the input
   * clusters index is stored.
   */
  std::vector<size_t> m_ClusterIndices;
  /**
   * @brief m_kdtree The search structure storing the merged clusters.
   */
  pcl::KdTreeFLANN<PointType>::Ptr m_kdtree;
};

#include "sf_clustertransfer.hpp"

#endif // SF_CLUSTERTRANSFER_H
