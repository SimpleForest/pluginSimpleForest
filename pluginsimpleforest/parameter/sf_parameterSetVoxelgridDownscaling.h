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

#ifndef SF_PARAMETERSETVOXELGRIDDOWNSCALING_H
#define SF_PARAMETERSETVOXELGRIDDOWNSCALING_H

#include "sf_abstractParameterSet.h"

/*! \brief SF_ParameterSetVoxelization.
 *
 *  Parameter set to convert a cloud into subclouds by voxelization.
 */
template <typename T>
struct SF_ParameterSetVoxelgridDownscaling:
        public SF_AbstractParameterSet<T>
{
    /**
     * @brief m_voxelSize The \ref  m_cloud is downscaled and the expected distance between two neighbor points is
     * \ref m_voxelSize afterwards.
     */
    float m_voxelSize;
    /**
     * @brief m_cloudOut Contains one cloud containing the remaining points and one cloud containing the filtered out points.
     */
    std::pair<std::pair<typename pcl::PointCloud<T>::Ptr, std::vector<size_t> >, std::pair<typename pcl::PointCloud<T>::Ptr, std::vector<size_t> > > m_cloudOut;

    SF_ParameterSetVoxelgridDownscaling() {}

    QStringList paramsToString(size_t remaining, size_t filtered) {
        float percentage = 0;
        if((remaining + filtered) > 0)
        {
            percentage = static_cast<float>(remaining)/(static_cast<float>(remaining+ filtered))*100.0f;
        }
        QStringList list;
        QString str = "The input cloud was downscaled with a  (";
        list.push_back(str);
        str = ("voxelSize                = ");
        str.append(QString::number(m_voxelSize));
        str.append("(m)");
        list.push_back(str);
        str = (" ). into a cloud of ");
        str.append(QString::number(remaining));
        str.append( " / ");
        str.append(QString::number(remaining+ filtered));
        str.append( " (");
        str.append(QString::number(percentage));
        str.append( " %) points.");
        list.push_back(str);
        return list;
    }

private:
    QStringList paramsToString() override {
        return QStringList();
    }
};
#endif // SF_PARAMETERSETVOXELGRIDDOWNSCALING_H
