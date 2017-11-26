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
#ifndef SF_ABSTRACT_STEP_H
#define SF_ABSTRACT_STEP_H

#include <steps/abstract_param/sf_abstract_param.h>
#include <ct_step/abstract/ct_abstractstep.h>
#include <ct_itemdrawable/ct_grid3d_sparse.h>
#include <ct_result/model/inModel/ct_inresultmodelgrouptocopy.h>
#include <ct_itemdrawable/ct_scene.h>
#include <ct_view/ct_stepconfigurabledialog.h>
#include <ct_result/ct_resultgroup.h>
#include <ct_iterator/ct_pointiterator.h>
#include <ct_pointcloudindex/ct_pointcloudindexvector.h>
#include <ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h>
#include <QObject>
#include <QStringList>
#include <QFuture>

#define DEF_IN_RESULT "ires"
#define DEF_IN_GRP    "igrp"
#define DEF_IN_CLOUD  "icloud"

class SF_Abstract_Step: public CT_AbstractStep
{
    Q_OBJECT

protected:
    void recursive_remove_if_empty(CT_AbstractItemGroup *parent, CT_AbstractItemGroup *group);

    void set_progress_by_future(QFuture<void> & future, float percentage_interval_start, float percentage_interval_size);

    virtual void createInResultModelListProtected() = 0;

    virtual void createOutResultModelListProtected() = 0;

    virtual void compute() = 0;

    QList<SF_Param_CT> _param_list;

    virtual void create_param_list(CT_ResultGroup *out_result) = 0;

    std::vector<CT_PointCloudIndexVector*> create_output_vectors(size_t number_output);

    void create_output_indices(std::vector<CT_PointCloudIndexVector*> &index_vectors, const std::vector<int> &indices, const CT_AbstractItemDrawableWithPointCloud *item_cpy_cloud_in);

    void identify_and_remove_corrupted_scenes(CT_ResultGroup* out_result);

    void create_output_index(std::vector<CT_PointCloudIndexVector *> &index_vectors, const std::vector<int> &indices, size_t counter, CT_PointIterator &point_it);

public:

    SF_Abstract_Step(CT_StepInitializeData & data_init);

    virtual CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &dataInit) = 0;

private:

    QList<CT_AbstractItemGroup*> _groups_to_be_removed;

    void check_is_empty(CT_StandardItemGroup* group, const CT_AbstractItemDrawableWithPointCloud* ct_cloud);

    void check_is_null_or_empty(const CT_AbstractItemDrawableWithPointCloud* ct_cloud, CT_StandardItemGroup* group);

    void check_grp_and_cloud(CT_StandardItemGroup* group);

    void identify_corrupted_scenes(CT_ResultGroup* out_result, int progress = 4);

    void remove_corrupted_scenes(int progress = 7);







};

#endif // SF_ABSTRACT_STEP_H
