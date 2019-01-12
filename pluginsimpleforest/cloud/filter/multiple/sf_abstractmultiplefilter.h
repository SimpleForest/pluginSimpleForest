/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_abstractmultiplefilter.h is part of SimpleForest - a plugin for the
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

#ifndef SF_ABSTRACTMULTIPLEFILTER_H
#define SF_ABSTRACTMULTIPLEFILTER_H

#include "cloud/filter/sf_abstractfilter.h"

/**
 * @brief The SF_AbstractMultipleFilter class Abstract class for manipulating
 * a templated PCL cloud producing n clusters
 */
template<typename PointType>
class SF_AbstractMultipleFilter : public SF_AbstractFilterI<PointType>
{
public:
  /**
   * @brief Standard constructor.
   */
  SF_AbstractMultipleFilter();
  /**
   * @brief clusterOut Getter for \ref m_clusterOut.
   * @return \ref m_clusterOut
   */
  std::vector<std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>> clusterOut() const;
  /**
   * @brief setClusterIn Setter for \ref clusterIn.
   * @param clusterIn The clusters for which \ref m_cloudIn is transferred to.
   */
  void setClusterIn(const std::vector<std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>>& clusterIn);

protected:
  /**
   * @brief m_clusterOut Output vector of clusters. Each cluster is a pair of a
   * PCL cloud and a CT index vector.
   */
  std::vector<std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>> m_clusterOut;
  /**
   * @brief m_clusterIn Input vector of clusters. Each cluster is a pair of a
   * PCL cloud and a CT index vector.
   */
  std::vector<std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>> m_clusterIn;

private:
  /**
   * @brief merge Merges \ref cluster into a single pcl PointCloud and a single
   * CT index Vector
   * @param clusters An input vector containing \ref std::pair of a pcl
   * PointCloud and a CT index vector
   * @return The merged pair.
   */
  std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>> merge(
    const std::vector<std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>> clusters);
};

template<typename PointType>
SF_AbstractMultipleFilter<PointType>::SF_AbstractMultipleFilter()
{}

template<typename PointType>
std::vector<std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>>
SF_AbstractMultipleFilter<PointType>::clusterOut() const
{
  return std::move(m_clusterOut);
}

template<typename PointType>
void
SF_AbstractMultipleFilter<PointType>::setClusterIn(
  const std::vector<std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>>& clusterIn)
{
  m_clusterIn = (std::move(clusterIn));
}

template<typename PointType>
std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>
SF_AbstractMultipleFilter<PointType>::merge(
  const std::vector<std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>> clusters)
{
  typename pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
  std::vector<size_t> indices;
  for (auto cluster : clusters) {
    *cloud += *(cluster.first);
    indices.insert(indices.end(), cluster.second.begin(), cluster.second.end());
  }
  return std::make_pair(cloud, indices);
}

#endif // SF_ABSTRACTMULTIPLEFILTER_H
