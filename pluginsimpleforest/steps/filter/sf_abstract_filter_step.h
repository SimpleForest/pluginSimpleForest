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
#ifndef SF_ABSTRACT_FILTER_STEP_H
#define SF_ABSTRACT_FILTER_STEP_H
#include <steps/sf_abstract_step.h>

class SF_Abstract_Filter_Step: public SF_Abstract_Step
{
public:

    SF_Abstract_Filter_Step(CT_StepInitializeData & data_init);

protected:
    void add_scene_to_grp(CT_StandardItemGroup* cloud_grp, const QString & out_cloud_complete_name, CT_PointCloudIndexVector *ct_point_cloud_index, CT_ResultGroup* out_result);

    void add_scene_in_subgrp_to_grp(CT_StandardItemGroup* filter_grp, const QString &out_cloud_complete_name, const QString &sub_grp_complete_name, CT_ResultGroup* out_result,
                                    CT_PointCloudIndexVector * ct_point_cloud_index);

};

#endif // SF_ABSTRACT_FILTER_STEP_H
