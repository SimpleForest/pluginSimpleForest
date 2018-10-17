/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_abstractfilter.h is part of SimpleForest - a plugin for the
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

#ifndef SF_ABSTRACTFILTER_H
#define SF_ABSTRACTFILTER_H

#include "cloud/sf_includestypedefs.h"

/**
 * @brief The SF_AbstractFilter class Abstract class for manipulating
 * a templated PCL cloud
 */
template <typename PointType>
class SF_AbstractFilter
{
public:
    /**
     * @brief SF_AbstractFilter Standard constructor receiving as input \ref m_cloudIn.
     * @param cloudIn \ref m_cloudIn
     */
    SF_AbstractFilter(std::pair<pcl::PointCloud<PointType>::Ptr,std::vector<size_t> > cloudIn);
    /**
     * @brief compute virtual method to be implemented
     */
    virtual void compute() = 0;

protected:
    /**
     * @brief m_cloudIn The templated input cloud stored in a pair. The first member
     * is a shared pointer to the PCL cloud itself, the second member is a vector
     * of accordingly sortedCT indices
     */
     std::pair<pcl::PointCloud<PointType>::Ptr,std::vector<size_t> > m_cloudIn;
};

#endif // SF_ABSTRACTFILTER_H

template<typename PointType>
SF_AbstractFilter<PointType>::SF_AbstractFilter(std::pair<pcl::PointCloud<PointType>::Ptr, std::vector<size_t> > cloudIn):
    m_cloudIn(cloudIn)
{

}
