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
template <typename PointType>
class SF_AbstractMultipleFilter:
        public SF_AbstractFilter
{
public:
    /**
     * @brief Standard constructor receiving as input \ref m_cloudIn.
     * @param cloudIn \ref m_cloudIn
     */
    SF_AbstractMultipleFilter(std::pair<pcl::PointCloud<PointType>::Ptr,std::vector<size_t> > cloudIn);
    /**
     * @brief clusterOut Getter for \ref m_clusterOut.
     * @return \ref m_clusterOut
     */
    std::vector<std::pair<pcl::PointCloud<PointType>::Ptr, std::vector<size_t> > > clusterOut() const;
protected:
    /**
     * @brief m_clusterOut A vector of clusters. Each cluster is a pair of a PCL cloud and a CT index vector.
     */
    std::vector<std::pair<pcl::PointCloud<PointType>::Ptr, std::vector<size_t> > m_clusterOut;
};

#endif // SF_ABSTRACTMULTIPLEFILTER_H

template<typename PointType>
SF_AbstractMultipleFilter<PointType>::SF_AbstractMultipleFilter(std::pair<pcl::PointCloud<PointType>::Ptr, std::vector<size_t> > cloudIn):
    SF_AbstractFilter(cloudIn)
{

}

template<typename PointType>
std::vector<std::pair<pcl::PointCloud<PointType>::Ptr, std::vector<size_t> > >  SF_AbstractMultipleFilter<PointType>::clusterOut() const
{
     return m_clusterOut;
}
