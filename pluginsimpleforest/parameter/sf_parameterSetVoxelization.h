/****************************************************************************

    Copyright (C) 2017-2018, Dr. Jan Hackenberg

    sf_parametersetvoxelization.h is part of SimpleForest - a plugin for the
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

#ifndef SF_PARAMETERSETVOXELIZATION_H
#define SF_PARAMETERSETVOXELIZATION_H

#include "sf_abstractParameterSet.h"

/*! \brief SF_ParameterSetVoxelization.
 *
 *  Parameter set to convert a cloud into subclouds by voxelization.
 */
template<typename T>
struct SF_ParameterSetVoxelization : public SF_AbstractParameterSet<T>
{
  /**
   * @brief m_voxelSize For \ref  m_cloud a 3d Raster of voxelsize m_voxelSize
   * is created. For each cell all contained points build a sub cloud in the
   * \ref  m_clusters output.
   */
  float m_voxelSize;
  /**
   * @brief m_clustersOut Contains subclouds and their according CT indices.
   */
  std::vector<std::pair<typename pcl::PointCloud<T>::Ptr, std::vector<size_t>>> m_clustersOut;

  SF_ParameterSetVoxelization() {}
  QStringList paramsToString() override
  {
    QStringList list;
    QString str = "To enable multithreaded processing the input point cloud "
                  "was clustered with voxelization with (";
    list.push_back(str);
    str = ("voxelSize                = ");
    str.append(QString::number(m_voxelSize));
    str.append("(m)");
    list.push_back(str);
    str = (" ). into ");
    str.append(QString::number(m_clustersOut.size()));
    str.append(" number of clusters.");
    list.push_back(str);
    return list;
  }
};

#endif // SF_PARAMETERSETVOXELIZATION_H
