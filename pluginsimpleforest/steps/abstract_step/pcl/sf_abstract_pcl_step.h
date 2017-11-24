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
#ifndef SF_ABSTRACT_PCL_STEP_H
#define SF_ABSTRACT_PCL_STEP_H
#include <cloud/sf_point.h>
#include <steps/abstract_step/sf_abstract_step.h>

#include <converters/CT_To_PCL/sf_converter_ct_to_pcl.h>

template <typename PointType>
class SF_Abstract_PCL_Step: public SF_Abstract_Step
{
    typename
    pcl::PointCloud<PointType>::Ptr _pcl_cloud_in;

//    bool _translate_to_origin = true;

//    void extract_cloud(SF_Converter_CT_To_PCL<PointType> converter);

public:

    SF_Abstract_PCL_Step(CT_StepInitializeData & data_init);

//    void convert_cloud_ct_to_pcl(const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in);

    virtual CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &dataInit) = 0;

protected:

    virtual void createInResultModelListProtected() = 0;

    virtual void createOutResultModelListProtected() = 0;

    virtual void compute() = 0;







};

#include <steps/abstract_step/pcl/sf_abstract_pcl_step.hpp>
#endif // SF_ABSTRACT_PCL_STEP_H

