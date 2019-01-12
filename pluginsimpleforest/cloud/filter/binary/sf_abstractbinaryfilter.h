/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_abstractbinaryfilter.h is part of SimpleForest - a plugin for the
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

#ifndef SF_ABSTRACTBINARYFILTER_H
#define SF_ABSTRACTBINARYFILTER_H

#include "cloud/filter/sf_abstractfilter.h"

/**
 * @brief The SF_AbstractBinaryFilter class for producing
 * out of a templated PCL cloud 2 clouds, e.g. for de-noising
 * a noise and a denoised cluster.
 */
template<typename PointType>
class SF_AbstractBinaryFilter : public SF_AbstractFilterI<PointType>
{
public:
  /**
   * @brief Standard constructor receiving as input \ref m_cloudIn.
   * @param cloudIn \ref m_cloudIn
   */
  SF_AbstractBinaryFilter();
  /**
   * @brief clusterOut Getter for \ref m_clusterOut.
   * @return \ref m_clusterOut
   */
  std::pair<std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>,
            std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>>
  clusterOut() const;

protected:
  /**
   * @brief m_clusterOut A pair of two clusters. Each cluster is a pair of a PCL
   * cloud and a CT index vector.
   */
  std::pair<std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>,
            std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>>
    m_clusterOut;
};

template<typename PointType>
std::pair<std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>,
          std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>>
SF_AbstractBinaryFilter<PointType>::clusterOut() const
{
  return m_clusterOut;
}

template<typename PointType>
SF_AbstractBinaryFilter<PointType>::SF_AbstractBinaryFilter()
{}

#endif // SF_ABSTRACTBINARYFILTER_H
