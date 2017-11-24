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
#ifndef SF_CONVERTER_CT_TO_PCL_H
#define SF_CONVERTER_CT_TO_PCL_H

#include <pcl/sf_point.h>

#include <converters/sf_abstract_converter.h>
#include "ct_iterator/ct_pointiterator.h"

template <typename PointType>
class SF_Converter_CT_To_PCL: public SF_Abstract_Converter
{
private:

    /**
     * @brief _cloud_translated A templated cloud centered around the origin
     */
    typename pcl::PointCloud<PointType>::Ptr _cloud_translated;

    /**
     * @brief _cloud_original A templated cloud with the original coordinates
     */
    typename pcl::PointCloud<PointType>::Ptr _cloud_original;

    virtual void reset();

    void iterate_cloud_and_convert(const CT_AbstractPointCloudIndex *index);

    void convert_point(CT_PointIterator it);

    void convert();

public:

    virtual void compute();



    SF_Converter_CT_To_PCL(const CT_AbstractItemDrawableWithPointCloud * itemCpy_cloud_in);







    typename
    pcl::PointCloud<PointType>::Ptr get_cloud_translated() const;
    typename
    pcl::PointCloud<PointType>::Ptr get_cloud_original() const;
};



#include "sf_converter_ct_to_pcl.hpp"

#endif // SF_CONVERTER_CT_TO_PCL_H
