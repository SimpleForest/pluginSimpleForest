/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_filtervoxelclustering.h is part of SimpleForest - a plugin for the
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

#ifndef SF_FILTERVOXELCLUSTERING_H
#define SF_FILTERVOXELCLUSTERING_H

#include "cloud/filter/multiple/sf_abstractmultiplefilter.h"
#include "parameter/sf_parameterSetVoxelization.h"

/**
 * @brief The SF_VoxelClustering class  This class is producing a voxel
 * over \ref m_cloudIn. All points contained in one raster cell build
 * a cluster. The VoxelSize is passed by the input parameter set.
 */
template <typename PointType>
class SF_VoxelClustering:
        public SF_AbstractMultipleFilter<PointType>
{
public:
    /**
     * @brief Standard constructor.
     */
    SF_VoxelClustering();
    /**
     * @brief compute Does the actual voxelization of \ref m_cloudIn.
     */
    void compute() override;
    /**
     * @brief setParam Setter for \ref m_param.
     * @param param \ref m_param.
     */
    void setParam(const SF_ParameterSetVoxelization<PointType> &param);

private:
    /**
     * @brief m_param The parameter set storing \ref m_cloudIn and
     * the cell size of the voxels.
     */
    SF_ParameterSetVoxelization<PointType> m_param;
    /**
     * @brief initialize
     */
    void initialize();
    /**
     * @brief m_min Stores minX, minY and minZ of \ref m_cloudIn.
     */
    PointType m_min;
    /**
     * @brief m_max Stores maxX, maxY and maxZ of \ref m_cloudIn.
     */
    PointType m_max;
};

#include "sf_filtervoxelclustering.hpp"

#endif // SF_FILTERVOXELCLUSTERING_H

