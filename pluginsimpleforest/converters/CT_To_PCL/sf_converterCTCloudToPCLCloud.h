/****************************************************************************

    Copyright (C) 2017-2018, Jan Hackenberg

    sf_converterCTCloudToPCLCluster.h is part of SimpleForestPlugin.

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

#ifndef SF_CONVERTERCTCLOUDTOPCLCLOUD_H
#define SF_CONVERTERCTCLOUDTOPCLCLOUD_H

#include "converters/sf_abstractConverter.h"
#include "ct_iterator/ct_pointiterator.h"
#include "pcl/sf_point.h"

template<typename PointType>
/**
 * @brief The SF_ConverterCTCloudToPCLCloud class Class to convert a CT input
 * Cloud to a PCL cloud. The PCL point type is a templated parameter. The Cloud
 * is shifted to the origin (0,0,0) during conversion.
 */
class SF_ConverterCTCloudToPCLCloud : public SF_AbstractConverter
{
public:
  /**
   * @brief SF_ConverterCTCloudToPCLCloud The constructor receiving a CT cloud
   * @param itemCpyCloudIn The CT cloud to be converted to \ref m_cloudOut
   */
  SF_ConverterCTCloudToPCLCloud(CT_AbstractItemDrawableWithPointCloud* itemCpyCloudIn);
  /**
   * @brief computes first a \ref m_translation of \ref m_itemCpyCloudIn and
   * then converts the translated cloud to \ref m_cloudOut.
   */
  void compute();
  /**
   * @brief cloudOut Getter method for a \ref std::pair consisting of the PCL
   * converted cloud \ref m_cloudOut and the vector of according CT indices \ref
   * m_CTIndices.
   * @return The described getter pair.
   */
  std::pair<typename pcl::PointCloud<PointType>::Ptr, std::vector<size_t>> cloudOut();

private:
  /**
   * @brief m_cloudOut The templated PCL output cloud.
   */
  typename pcl::PointCloud<PointType>::Ptr m_cloudOut;
  /**
   * @brief m_CTIndices Stores the internal CT indices in same order as \ref
   * m_cloudOut.
   */
  std::vector<size_t> m_CTIndices;
  /**
   * @brief initialize Allocates memory for templated \ref m_cloudOut, as well
   * as for \ref m_CTIndices. Computes \ref _translation.
   */
  void initialize();
};

#include "sf_converterCTCloudToPCLCloud.hpp"

#endif // SF_CONVERTERCTCLOUDTOPCLCLOUD_H
