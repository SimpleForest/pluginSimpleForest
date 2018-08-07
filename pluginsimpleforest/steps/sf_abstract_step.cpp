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

void  SF_Abstract_Step::recursive_remove_if_empty(CT_AbstractItemGroup *parent, CT_AbstractItemGroup *group) {
        if(parent != NULL) {
            parent->removeGroup(group);
            if(parent->isEmpty()) {
                recursive_remove_if_empty(parent->parentGroup(), parent);
            }
        } else {
            ((CT_ResultGroup*)group->result())-> removeGroupSomethingInStructure(group);
        }
}

void SF_Abstract_Step::set_progress_by_future(QFuture<void> &future, float percentage_interval_start, float percentage_interval_size) {
    float progress_min = future.progressMinimum();
    float progress_size = future.progressMaximum()-progress_min;
    while(!future.isFinished()) {
        setProgress(percentage_interval_start + (percentage_interval_size*(future.progressValue() - progress_min)/progress_size ));
    }
}

CT_Scene* SF_Abstract_Step::mergeIndices(CT_ResultGroup *out_result, CT_StandardItemGroup* root, const QString defInnGrp, const QString defInCloud) {
    CT_ResultGroupIterator out_res_it(out_result, this, defInnGrp);
    CT_PointCloudIndexVector *mergedClouds = new CT_PointCloudIndexVector();
    mergedClouds->setSortType(CT_AbstractCloudIndex::NotSorted);
    std::vector<size_t> indices;
    while(!isStopped() && out_res_it.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) out_res_it.next();
        const CT_AbstractItemDrawableWithPointCloud* ct_cloud = (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, defInCloud);
        CT_PointIterator iter(ct_cloud->getPointCloudIndex());
        while(iter.hasNext() && ! isStopped()) {
            iter.next();
            size_t index = iter.currentGlobalIndex();
            indices.push_back(index);
        }
    }
    std::sort(indices.begin(), indices.end());
    for(size_t i = 0; i < indices.size(); i++) {
        mergedClouds->addIndex(indices.at(i));
    }
    mergedClouds->setSortType(CT_PointCloudIndexVector::SortedInAscendingOrder);
    CT_Scene* scene = new CT_Scene(defInCloud , out_result, PS_REPOSITORY->registerPointCloudIndex(mergedClouds));
    return scene;
}

void SF_Abstract_Step::createPreConfigurationDialog() {
    CT_StepConfigurableDialog *config_dialog = newStandardPreConfigurationDialog();
    config_dialog->addBool("Uncheck to deactivate parameterization possibilities of this step. Only recommended for beginners","","expert", _is_expert);
}

Eigen::Vector3f SF_Abstract_Step::get_min(const CT_Scene* ct_cloud) {
    Eigen::Vector3f min;
    min(0) = ct_cloud->minX();
    min(1) = ct_cloud->minY();
    min(2) = ct_cloud->minZ();
    return min;
}

Eigen::Vector3f SF_Abstract_Step::get_max(const CT_Scene *ct_cloud) {
    Eigen::Vector3f max;
    max(0) = ct_cloud->maxX();
    max(1) = ct_cloud->maxY();
    max(2) = ct_cloud->maxZ();
    return max;
}

void SF_Abstract_Step::identify_and_remove_corrupted_scenes(CT_ResultGroup* out_result) {
    identify_corrupted_scenes(out_result);
    remove_corrupted_scenes();
}


SF_Abstract_Step::SF_Abstract_Step(CT_StepInitializeData &data_init): CT_AbstractStep(data_init) {

}
void SF_Abstract_Step::check_is_empty(CT_StandardItemGroup* group, const CT_AbstractItemDrawableWithPointCloud* ct_cloud) {
    if(ct_cloud->getPointCloudIndex()->size() <=  0) {
        _groups_to_be_removed.push_back(group);
    }
}

void SF_Abstract_Step::check_is_null_or_empty(const CT_AbstractItemDrawableWithPointCloud* ct_cloud, CT_StandardItemGroup* group) {
    if(ct_cloud!= NULL){
        check_is_empty(group, ct_cloud);
    } else {
        _groups_to_be_removed.push_back(group);
    }
}

void SF_Abstract_Step::check_grp_and_cloud(CT_StandardItemGroup* group) {
    if(group!=NULL) {
        const CT_AbstractItemDrawableWithPointCloud* ct_cloud = (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD);
        check_is_null_or_empty(ct_cloud, group);
    } else {
        _groups_to_be_removed.push_back(group);
    }
}

void SF_Abstract_Step::identify_corrupted_scenes( CT_ResultGroup* out_result, int progress) {
    _groups_to_be_removed.clear();
    CT_ResultGroupIterator out_res_it(out_result, this, DEF_IN_GRP);
    while(!isStopped() && out_res_it.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) out_res_it.next();
        check_grp_and_cloud(group);
    }    
    setProgress(progress);
}

void SF_Abstract_Step::createPostConfigurationDialog() {
    CT_StepConfigurableDialog *config_dialog = newStandardPostConfigurationDialog();
    if(!_is_expert) {
        createPostConfigurationDialogBeginner(config_dialog);
    } else {
        createPostConfigurationDialogExpert(config_dialog);
    }
    createPostConfigurationDialogCitation(config_dialog);
}

void SF_Abstract_Step::createPostConfigurationDialogCitation(CT_StepConfigurableDialog *config_dialog) {
    config_dialog->addEmpty();
    config_dialog->addTitle(QObject::tr("For general usage of the SimpleForest plugin please cite the following:"));
    config_dialog->addEmpty();
    config_dialog->addTitle(QObject::tr("Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P."));
    config_dialog->addTitle(QObject::tr("<em>SimpleTree - An Efficient Open Source Tool to Build Tree Models from TLS Clouds.</em>"));
    config_dialog->addTitle(QObject::tr("Forests <b>2015</b>, 6, 4245-4294."));
}

void SF_Abstract_Step::remove_corrupted_scenes(int progress) {
    while(!_groups_to_be_removed.isEmpty()) {
        CT_AbstractItemGroup *group = _groups_to_be_removed.takeLast();
        recursive_remove_if_empty(group->parentGroup(), group);
    }    
    setProgress(progress);
}


std::vector<CT_PointCloudIndexVector *> SF_Abstract_Step::create_output_vectors(size_t number_output) {
    std::vector<CT_PointCloudIndexVector *> result;
    for(size_t i = 0; i < number_output; i++) {
        CT_PointCloudIndexVector * vec = new CT_PointCloudIndexVector();
        result.push_back(vec);
    }
    return result;
}

void SF_Abstract_Step::create_output_index(std::vector<CT_PointCloudIndexVector *> &index_vectors, const std::vector<int> &indices, size_t counter, CT_PointIterator & point_it) {
    point_it.next();
    size_t index_ct = point_it.currentGlobalIndex();
    if(counter <indices.size()) {
        size_t index_cloud = indices.at(counter);
        if(index_cloud<index_vectors.size()) {
            index_vectors[index_cloud]->addIndex(index_ct);
        } else {
            qDebug() << "TODO void SF_Abstract_Step::create_output_index";
        }
    } else {
        qDebug() << "TODO void SF_Abstract_Step::create_output_index2";
    }
}

void SF_Abstract_Step::create_output_indices(std::vector<CT_PointCloudIndexVector *> &index_vectors, const std::vector<int> &indices, const CT_AbstractItemDrawableWithPointCloud * item_cpy_cloud_in) {
    const CT_AbstractPointCloudIndex * point_cloud_index = item_cpy_cloud_in->getPointCloudIndex();
    CT_PointIterator point_it(point_cloud_index);
    size_t counter = 0;
    while(point_it.hasNext()) {
        create_output_index(index_vectors, indices, counter++, point_it);
    }
}

void SF_Abstract_Step::write_logger() {
    if(!_param_list.empty()) {
        QString str = _param_list[0].to_string();
        PS_LOG->addMessage(LogInterface::info, LogInterface::step, str);
    }
}

