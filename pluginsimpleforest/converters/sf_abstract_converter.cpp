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
#include "sf_abstract_converter.h"


SF_Abstract_Converter::SF_Abstract_Converter(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in):
    _itemCpy_cloud_in(itemCpy_cloud_in)
{
    assert(_itemCpy_cloud_in!=NULL);
}

void SF_Abstract_Converter::add_point_vec(CT_PointIterator &it)
{

    const CT_Point &internalPoint = it.next().currentPoint();
    _center_of_mass[0] += internalPoint(0);
    _center_of_mass[1] += internalPoint(1);
    _center_of_mass[2] += internalPoint(2);
}

void SF_Abstract_Converter::sum_vector(CT_PointIterator &it)
{
    while(it.hasNext())
    {
        add_point_vec(it);
    }
}

Eigen::Vector3d SF_Abstract_Converter::get_translation_matrix() const
{
    return _center_of_mass;
}

void SF_Abstract_Converter::normalize_sum_vector_by_size(size_t size)
{
    _center_of_mass[0]/=size;
    _center_of_mass[1]/=size;
    _center_of_mass[2]/=size;
}

void SF_Abstract_Converter::compute_center_of_mass(size_t size, const CT_AbstractPointCloudIndex* index)
{
    assert(size > 0);
    CT_PointIterator it(index);
    sum_vector(it);
    normalize_sum_vector_by_size(size);
}

void SF_Abstract_Converter::compute_translation_to_origin()
{
    const CT_AbstractPointCloudIndex* index = _itemCpy_cloud_in->getPointCloudIndex();
    assert(index->size() > 0);
    compute_center_of_mass(index->size(), index);
}
