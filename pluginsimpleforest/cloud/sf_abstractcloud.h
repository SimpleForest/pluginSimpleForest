/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_abstractcloud.h is part of SimpleForest - a plugin for the
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

#ifndef SF_ABSTRACTCLOUD_H
#define SF_ABSTRACTCLOUD_H

#include "cloud/sf_includestypedefs.h"

/**
 * @brief The SF_AbstractCloud class Abstract class for manipulating
 * a templated PCL cloud
 */
template <typename PointType> class SF_AbstractCloudI {
public:
  /**
   * @brief SF_AbstractCloud Standard constructor.
   */
  SF_AbstractCloudI();
  /**
   * @brief compute virtual method to be implemented
   */
  virtual void compute() = 0;

  void setCoudIn(
      std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>
          cloudIn);

protected:
  /**
   * @brief m_cloudIn The templated input cloud stored in a pair. The first
   * member is a shared pointer to the PCL cloud itself, the second member is a
   * vector of accordingly sortedCT indices
   */
  std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>
      m_cloudIn;
};

template <typename PointType>
SF_AbstractCloudI<PointType>::SF_AbstractCloudI() {}

template <typename PointType>
void SF_AbstractCloudI<PointType>::setCoudIn(
    std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>
        cloudIn) {
  m_cloudIn = cloudIn;
}

#endif // SF_ABSTRACTCLOUD_H
