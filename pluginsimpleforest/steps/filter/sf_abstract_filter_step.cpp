/****************************************************************************

 Copyright (C) 2017-2018 Jan Hackenberg, free software developer
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

#include "sf_abstract_filter_step.h"

SF_Abstract_Filter_Step::SF_Abstract_Filter_Step(CT_StepInitializeData &data_init): SF_Abstract_Step(data_init) {

}

void SF_Abstract_Filter_Step::add_scene_to_grp(CT_StandardItemGroup* cloud_grp, const QString & out_cloud_complete_name, CT_PointCloudIndexVector * ct_point_cloud_index, CT_ResultGroup* out_result) {
    CT_Scene* outScene = new CT_Scene(out_cloud_complete_name , out_result, PS_REPOSITORY->registerPointCloudIndex(ct_point_cloud_index));
    outScene->updateBoundingBox();
    cloud_grp->addItemDrawable(outScene);
}

void SF_Abstract_Filter_Step::add_scene_in_subgrp_to_grp(CT_StandardItemGroup* filter_grp,const QString & out_cloud_complete_name,
                                                                     const QString & sub_grp_complete_name, CT_ResultGroup* out_result,  CT_PointCloudIndexVector * ct_point_cloud_index) {
    CT_StandardItemGroup* cloud_grp = new CT_StandardItemGroup(sub_grp_complete_name, out_result);
    filter_grp->addGroup(cloud_grp);
    add_scene_to_grp(cloud_grp, out_cloud_complete_name, ct_point_cloud_index , out_result);
}


