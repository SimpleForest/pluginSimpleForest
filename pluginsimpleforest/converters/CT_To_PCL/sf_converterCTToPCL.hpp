/****************************************************************************

 Copyright (C) 2017-2017 Jan Hackenberg, free software developer
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

#ifndef SF_CONVERTER_CT_TO_PCL_HPP
#define SF_CONVERTER_CT_TO_PCL_HPP

#include <pcl/common/centroid.h>

#include "sf_converterCTToPCL.h"

template<typename PointType>
Sf_ConverterCTToPCL<PointType>::Sf_ConverterCTToPCL()
{
  _cloudOriginal.reset(new pcl::PointCloud<PointType>);
  _cloudTranslated.reset(new pcl::PointCloud<PointType>);
}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr
Sf_ConverterCTToPCL<PointType>::getCloudOriginal() const
{
  return _cloudOriginal;
}

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr
Sf_ConverterCTToPCL<PointType>::cloudTranslated() const
{
  return _cloudTranslated;
}

template<typename PointType>
void
Sf_ConverterCTToPCL<PointType>::convertPoint(CT_PointIterator& it)
{
  const CT_Point& internalPoint = it.next().currentPoint();
  PointType origin;
  origin.x = internalPoint[0];
  origin.y = internalPoint[1];
  origin.z = internalPoint[2];
  PointType translated;

  translated.x = internalPoint[0] - m_translation[0];
  translated.y = internalPoint[1] - m_translation[1];
  translated.z = internalPoint[2] - m_translation[2];
  _cloudOriginal->push_back(origin);
  _cloudTranslated->push_back(translated);
}

template<typename PointType>
void
Sf_ConverterCTToPCL<PointType>::iterateCloudAndConvert(const CT_AbstractPointCloudIndex* index)
{
  CT_PointIterator it(index);
  while (it.hasNext()) {
    convertPoint(it);
  }
}

template<typename PointType>
void
Sf_ConverterCTToPCL<PointType>::compute()
{
  computeTranslationToOrigin();
  convert();
}

template<typename PointType>
void
Sf_ConverterCTToPCL<PointType>::convert()
{
  const CT_AbstractPointCloudIndex* index = m_itemCpyCloudIn->getPointCloudIndex();
  assert(index->size() > 0);
  iterateCloudAndConvert(index);
}

template<typename PointType>
std::vector<typename pcl::PointCloud<PointType>::Ptr>
Sf_ConverterCTToPCL<PointType>::mergeSubCloudsToVector(CT_Grid3D_Sparse<int>* indices)
{
  const CT_AbstractPointCloudIndex* index = m_itemCpyCloudIn->getPointCloudIndex();
  assert(index->size() > 0);
  CT_PointIterator it(index);
  std::vector<typename pcl::PointCloud<PointType>::Ptr> clouds;
  int number_initialized_clouds = 0;
  while (it.hasNext()) {
    const CT_Point& ct_point = it.next().currentPoint();
    size_t index{};
    indices->indexAtXYZ(ct_point(0), ct_point(1), ct_point(2), index);
    int value_at = indices->valueAtIndex(index);
    if (value_at <= -1) {
      typename pcl::PointCloud<PointType>::Ptr cell_cloud(new pcl::PointCloud<PointType>);
      value_at = number_initialized_clouds++;
      clouds.push_back(cell_cloud);
      indices->setValueAtIndex(index, value_at);
    }
    PointType p;
    p.x = ct_point(0) - m_translation[0];
    p.y = ct_point(1) - m_translation[1];
    p.z = ct_point(2) - m_translation[2];
    clouds[value_at]->points.push_back(p);
  }
  return clouds;
}

template<typename PointType>
void
Sf_ConverterCTToPCL<PointType>::mergeSubCloudsToVector(CT_Grid3D_Sparse<int>* indices,
                                                       std::vector<typename pcl::PointCloud<PointType>::Ptr> cloudsVec,
                                                       typename pcl::PointCloud<PointType>::Ptr downscaled_cloud)
{
  for (size_t i = 0; i < indices->xArraySize(); i++) {
    for (size_t j = 0; j < indices->yArraySize(); j++) {
      for (size_t k = 0; k < indices->zArraySize(); k++) {
        int index = indices->value(i, j, k);
        if (index >= 0) {
          typename pcl::PointCloud<PointType>::Ptr cell_cloud = cloudsVec[index];
          if (cell_cloud != nullptr) {
            Eigen::Vector4d centroid;
            pcl::compute3DCentroid(*cell_cloud, centroid);
            PointType p;
            p.x = centroid(0);
            p.y = centroid(1);
            p.z = centroid(2);
            p.intensity = cell_cloud->points.size();
            downscaled_cloud->push_back(p);
          }
        }
      }
    }
  }
}

template<typename PointType>
void
Sf_ConverterCTToPCL<PointType>::downScale(float range, typename pcl::PointCloud<PointType>::Ptr downscaledCloud)
{
  computeTranslationToOrigin();
  CT_Grid3D_Sparse<int>* indices = CT_Grid3D_Sparse<int>::createGrid3DFromXYZCoords(NULL,
                                                                                    NULL,
                                                                                    m_itemCpyCloudIn->minX(),
                                                                                    m_itemCpyCloudIn->minY(),
                                                                                    m_itemCpyCloudIn->minZ(),
                                                                                    m_itemCpyCloudIn->maxX(),
                                                                                    m_itemCpyCloudIn->maxY(),
                                                                                    m_itemCpyCloudIn->maxZ(),
                                                                                    range,
                                                                                    -2,
                                                                                    -1);
  std::vector<typename pcl::PointCloud<PointType>::Ptr> cloudsVec = mergeSubCloudsToVector(indices);
  mergeSubCloudsToVector(indices, cloudsVec, downscaledCloud);
}

#endif // SF_CONVERTER_CT_TO_PCL_HPP
