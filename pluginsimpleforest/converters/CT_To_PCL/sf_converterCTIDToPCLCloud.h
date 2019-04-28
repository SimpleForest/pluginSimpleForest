/****************************************************************************

 Copyright (C) 2017-2019, Dr. Jan Hackenberg, free software developer
 All rights reserved.

 Contact : https://github.com/SimpleForest

 Developers : Jan Hackenberg

 This file is part of SimpleForest plugin Version 1 for Computree.

 SimpleForest plugin is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 SimpleForest plugin is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with SimpleForest plugin.  If not, see <http://www.gnu.org/licenses/>.

 PluginSimpleForest is an extended version of the SimpleTree platform.

*****************************************************************************/

#ifndef SF_CONVERTERCTIDTOPCLCLOUD_H
#define SF_CONVERTERCTIDTOPCLCLOUD_H

#include <pcl/sf_point.h>

#include "ct_iterator/ct_pointiterator.h"
#include <converters/sf_abstractConverter.h>
#include <ct_itemdrawable/ct_grid3d_sparse.h>
#include <ct_itemdrawable/ct_pointsattributesscalartemplated.h>

template<typename PointType>
class Sf_ConverterCTIDToPCLCloud : public SF_AbstractConverter
{
public:
  Sf_ConverterCTIDToPCLCloud();
  void compute();
  typename pcl::PointCloud<PointType>::Ptr cloud() const;
  void setCloudAndID(typename pcl::PointCloud<PointType>::Ptr cloud, CT_PointsAttributesScalarTemplated<int>* IDs);
  std::vector<typename pcl::PointCloud<PointType>::Ptr> clusters() const;

private:
  typename pcl::PointCloud<PointType>::Ptr m_cloud;
  typename std::vector<typename pcl::PointCloud<PointType>::Ptr> m_clusters;
  CT_PointsAttributesScalarTemplated<int>* m_IDs;
  CT_AbstractItemDrawableWithPointCloud* m_itemCpyCloudIn;
  int m_numClusters = -1;
  void retrieveNumberClusters();
  void initializeClusters();
  void writeIdsAndClusters();
  void revertClusters();
};

#include "converters/CT_To_PCL/sf_converterCTIDToPCLCloud.hpp"

#endif // SF_CONVERTERCTIDTOPCLCLOUD_H
