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
#include "sf_abstract_step.h"


void  SF_Abstract_Step::recursive_remove_if_empty(CT_AbstractItemGroup *parent, CT_AbstractItemGroup *group){
        if(parent != NULL)
        {
            parent->removeGroup(group);
            if(parent->isEmpty())
            {
                recursive_remove_if_empty(parent->parentGroup(), parent);
            }
        }
        else
        {
            ((CT_ResultGroup*)group->result())-> removeGroupSomethingInStructure(group);
        }
}

void SF_Abstract_Step::set_progress_by_future(QFuture<void> &future, float percentage_interval_start, float percentage_interval_size)
{
    float progress_min = future.progressMinimum();
    float progress_size = future.progressMaximum()-progress_min;
    while(!future.isFinished())
    {
        setProgress(percentage_interval_start + (percentage_interval_size*(future.progressValue() - progress_min)/progress_size ));
    }
}

SF_Abstract_Step::SF_Abstract_Step(CT_StepInitializeData &data_init): CT_AbstractStep(data_init)
{

}
void SF_Abstract_Step::identify_corrupted_scenes( CT_ResultGroup* out_result)
{
    _groups_to_be_removed.clear();
    CT_ResultGroupIterator out_res_it(out_result, this, DEF_IN_GRP);
    while(!isStopped() && out_res_it.hasNext())
    {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) out_res_it.next();
        if(group!=NULL)
        {
            const CT_AbstractItemDrawableWithPointCloud* ct_cloud = (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD);
            if(ct_cloud!= NULL)
            {
                if(ct_cloud->getPointCloudIndex()->size() <=  0)
                {
                    _groups_to_be_removed.push_back(group);
                }
            } else {
                _groups_to_be_removed.push_back(group);
            }
        } else {
            _groups_to_be_removed.push_back(group);
        }
    }
}

void SF_Abstract_Step::remove_corrupted_scenes()
{
    while(!_groups_to_be_removed.isEmpty())
    {
        CT_AbstractItemGroup *group = _groups_to_be_removed.takeLast();
        recursive_remove_if_empty(group->parentGroup(), group);
    }
}


std::vector<CT_PointCloudIndexVector *> SF_Abstract_Step::create_output_vectors(size_t number_output)
{
    std::vector<CT_PointCloudIndexVector *> result;
    for(size_t i = 0; i < number_output; i++)
    {
        CT_PointCloudIndexVector * vec = new CT_PointCloudIndexVector();
        result.push_back(vec);
    }
    return result;
}

void SF_Abstract_Step::create_output_indices(std::vector<CT_PointCloudIndexVector *> &index_vectors, const std::vector<int> &indices, const CT_AbstractItemDrawableWithPointCloud * item_cpy_cloud_in)
{
    const CT_AbstractPointCloudIndex * point_cloud_index = item_cpy_cloud_in->getPointCloudIndex();
    CT_PointIterator point_it(point_cloud_index);
    size_t counter = 0;
    while(point_it.hasNext())
    {
        point_it.next();
        size_t index_ct = point_it.currentGlobalIndex();
        size_t index_cloud = indices.at(counter);
        counter++;
        index_vectors[index_cloud]->addIndex(index_ct);
    }
}

