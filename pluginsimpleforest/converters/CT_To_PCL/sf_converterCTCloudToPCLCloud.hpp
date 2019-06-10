/****************************************************************************

    Copyright (C) 2017-2018 , Jan Hackenberg

    sf_converterctcloudtopclcloud.hpp is part of SimpleForest - a plugin for the
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

#ifndef SF_CONVERTERCTCLOUDTOPCLCLOUD_HPP
#define SF_CONVERTERCTCLOUDTOPCLCLOUD_HPP

#include "sf_converterCTCloudToPCLCloud.h"

template<typename PointType>
SF_ConverterCTCloudToPCLCloud<PointType>::SF_ConverterCTCloudToPCLCloud(CT_AbstractItemDrawableWithPointCloud* itemCpyCloudIn)
{
  m_itemCpyCloudIn = itemCpyCloudIn;
  initialize(true);
}

template<typename PointType>
SF_ConverterCTCloudToPCLCloud<PointType>::SF_ConverterCTCloudToPCLCloud(CT_AbstractItemDrawableWithPointCloud* itemCpyCloudIn,
                                                                        Eigen::Vector3d translation)
{
  m_itemCpyCloudIn = itemCpyCloudIn;
  m_translation = translation;
  initialize(false);
}

template<typename PointType>
std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>
SF_ConverterCTCloudToPCLCloud<PointType>::cloudOut()
{
  return std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>>(m_cloudOut, m_CTIndices);
}

template<typename PointType>
void
SF_ConverterCTCloudToPCLCloud<PointType>::compute()
{
  const CT_AbstractPointCloudIndex* indices = m_itemCpyCloudIn->getPointCloudIndex();
  CT_PointIterator it(indices);
  size_t index = 0;
  while (it.hasNext()) {
    const CT_Point& internalPoint = it.next().currentPoint();
    m_CTIndices[index] = it.currentGlobalIndex();
    PointType translated;
    translated.x = internalPoint[0] - m_translation[0];
    translated.y = internalPoint[1] - m_translation[1];
    translated.z = internalPoint[2] - m_translation[2];
    m_cloudOut->points[index++] = std::move(translated);
  }
}

template<typename PointType>
void
SF_ConverterCTCloudToPCLCloud<PointType>::initialize(bool computeTranslation)
{
  m_cloudOut.reset(new pcl::PointCloud<PointType>());
  size_t size = m_itemCpyCloudIn->getPointCloudIndexSize();
  m_cloudOut->points.resize(size);
  m_cloudOut->width = size;
  m_cloudOut->height = 1;
  m_CTIndices.resize(size);
  if (computeTranslation) {
    computeTranslationToOrigin();
  }
}

#endif // SF_CONVERTERCTCLOUDTOPCLCLOUD_HPP
