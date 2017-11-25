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
#include "sf_converter_ct_to_pcl.h"

template <typename PointType>
SF_Converter_CT_To_PCL<PointType>:: SF_Converter_CT_To_PCL(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in):
    SF_Abstract_Converter(itemCpy_cloud_in)
{
    reset();
}

template <typename PointType>
void SF_Converter_CT_To_PCL<PointType>::reset()
{
    _cloud_original.reset(new pcl::PointCloud<PointType>);
    _cloud_translated.reset(new pcl::PointCloud<PointType>);

}

template <typename PointType>
typename
pcl::PointCloud<PointType>::Ptr SF_Converter_CT_To_PCL<PointType>::get_cloud_original() const
{
    return _cloud_original;
}
template <typename PointType>
typename
pcl::PointCloud<PointType>::Ptr SF_Converter_CT_To_PCL<PointType>::get_cloud_translated() const
{
    return _cloud_translated;
}

template <typename PointType>
void SF_Converter_CT_To_PCL<PointType>::convert_point(CT_PointIterator& it)
{
    const CT_Point &internalPoint = it.next().currentPoint();
    PointType origin(internalPoint[0],internalPoint[1],internalPoint[2]);
    PointType translated(internalPoint[0]-_center_of_mass[0],internalPoint[1]-_center_of_mass[1],internalPoint[2]-_center_of_mass[2]);
    _cloud_original->push_back(origin);
    _cloud_translated->push_back(translated);
}

template <typename PointType>
void SF_Converter_CT_To_PCL<PointType>::iterate_cloud_and_convert(const CT_AbstractPointCloudIndex* index )
{
    CT_PointIterator it(index);
    while(it.hasNext())
    {
        convert_point(it);
    }
}

template <typename PointType>
void SF_Converter_CT_To_PCL<PointType>::compute()
{
    compute_translation_to_origin();
    convert();
}

template <typename PointType>
void SF_Converter_CT_To_PCL<PointType>::convert()
{
    const CT_AbstractPointCloudIndex* index =_itemCpy_cloud_in->getPointCloudIndex();
    assert( index->size() > 0);
    iterate_cloud_and_convert(index);
}

#endif // SF_CONVERTER_CT_TO_PCL_HPP
