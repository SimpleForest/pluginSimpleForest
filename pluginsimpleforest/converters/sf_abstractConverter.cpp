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

#include "sf_abstractConverter.h"

SF_AbstractConverter::SF_AbstractConverter() {}

Eigen::Vector3d
SF_AbstractConverter::translation() const
{
  return m_translation;
}

void
SF_AbstractConverter::setItemCpyCloudInDeprecated(const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in)
{
  m_itemCpyCloudIn = itemCpy_cloud_in;
}

void
SF_AbstractConverter::computeTranslationToOrigin()
{
  const CT_AbstractPointCloudIndex* index = m_itemCpyCloudIn->getPointCloudIndex();
  assert(index->size() > 0);
  m_translation[0] = m_itemCpyCloudIn->minX() + 0.5 * (m_itemCpyCloudIn->maxX() - m_itemCpyCloudIn->minX());
  m_translation[1] = m_itemCpyCloudIn->minY() + 0.5 * (m_itemCpyCloudIn->maxY() - m_itemCpyCloudIn->minY());
  m_translation[2] = m_itemCpyCloudIn->minZ() + 0.5 * (m_itemCpyCloudIn->maxZ() - m_itemCpyCloudIn->minZ());
}
