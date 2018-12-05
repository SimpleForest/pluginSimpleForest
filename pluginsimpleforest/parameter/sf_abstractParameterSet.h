/****************************************************************************

    Copyright (C) 2017-2018, Dr. Jan Hackenberg

    sf_converterCTCloudToPCLCluster.h is part of SimpleForest - a plugin for the
    Computree platform.

    SimpleForestPlugin is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SimpleForestPlugin is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with
    SimpleForestPlugin. If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/

#ifndef SF_ABSTRACTPARAMETERSET_H
#define SF_ABSTRACTPARAMETERSET_H

#include "pcl/sf_point.h"
#include <QStringList>

template <typename T> struct SF_AbstractParameterSet {
  /**
   * @brief m_cloud The first member of m_cloud is the point cloud itself,
   * the second a vector of the Computree indices
   */
  std::pair<typename pcl::PointCloud<T>::Ptr, std::vector<size_t>> m_cloud;
  /**
   * @brief SF_AbstractParameterSet Default contructor
   */
  SF_AbstractParameterSet() {}
  /**
   * @brief paramsToString A QStringList is created with a QString
   * per line. The list will contain a description of all parameters.
   * @return The parameter list
   */
  virtual QStringList paramsToString() = 0;
};

#endif // SF_ABSTRACTPARAMETERSET_H
