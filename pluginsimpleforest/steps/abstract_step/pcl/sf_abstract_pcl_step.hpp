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
#ifndef SF_ABSTRACT_PCL_STEP_HPP
#define SF_ABSTRACT_PCL_STEP_HPP

#include "sf_abstract_pcl_step.h"
//template <typename PointTypeSF>
//void SF_Abstract_PCL_Step<PointTypeSF>::convert_cloud_ct_to_pcl( const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in)
//{
//    SF_Converter_CT_To_PCL<PointTypeSF> converter(itemCpy_cloud_in );
//    converter.compute();
//    extract_cloud(converter);
//}

//template <typename PointTypeSF>
//void SF_Abstract_PCL_Step<PointTypeSF>::extract_cloud(SF_Converter_CT_To_PCL<PointTypeSF> converter)
//{
//    if(_translate_to_origin)
//    {
//        _pcl_cloud_in = converter.get_cloud_translated();
//    } else {
//        _pcl_cloud_in = converter.get_cloud_original();
//    }
//}

template <typename PointTypeSF>
SF_Abstract_PCL_Step<PointTypeSF>::SF_Abstract_PCL_Step(CT_StepInitializeData &data_init):SF_Abstract_Step(data_init)
{

}



#endif // SF_ABSTRACT_PCL_STEP_HPP
