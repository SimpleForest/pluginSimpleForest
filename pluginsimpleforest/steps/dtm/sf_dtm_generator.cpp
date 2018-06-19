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

#include "sf_dtm_generator.h"
#include "converters/CT_To_PCL/sf_converter_ct_to_pcl.h"
#include "pcl/geometry/DTM/sf_dtm.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/features/normal_3d.h>

SF_DTM_Generator::SF_DTM_Generator(CT_StepInitializeData &data_init): SF_Abstract_Step(data_init) {
    _non_expert_level.append(_less);
    _non_expert_level.append(_intermediate);
    _non_expert_level.append(_many);
}

SF_DTM_Generator::~SF_DTM_Generator() {

}

QString SF_DTM_Generator::getStepDescription() const {
    return tr("DTM generation");
}

QString SF_DTM_Generator::getStepDetailledDescription() const {
    return tr("DTM generation - uses a pyramidal MLESAC plane fitting apporach on a robust downscaled input ground cloud.");
}

QString SF_DTM_Generator::getStepURL() const {
    return tr("https://www.youtube.com/watch?v=5i6_Rtv-xEw");
}

CT_VirtualAbstractStep* SF_DTM_Generator::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_DTM_Generator(dataInit);
}

QStringList SF_DTM_Generator::getStepRISCitations() const {
    QStringList _RIS_citation_list;
    _RIS_citation_list.append(QString("TY  - JOUR\n"
                                      "T1  - SimpleTree - an efficient open source tool to build tree models from TLS clouds\n"
                                      "A1  - Hackenberg, Jan\n"
                                      "A1  - Spiecker, Heinrich\n"
                                      "A1  - Calders, Kim\n"
                                      "A1  - Disney, Mathias\n"
                                      "A1  - Raumonen, Pasi\n"
                                      "JO  - Forests\n"
                                      "VL  - 6\n"
                                      "IS  - 11\n"
                                      "SP  - 4245\n"
                                      "EP  - 4294\n"
                                      "Y1  - 2015\n"
                                      "PB  - Multidisciplinary Digital Publishing Institute\n"
                                      "UL  - http://www.simpletree.uni-freiburg.de/\n"
                                      "ER  - \n"));


    _RIS_citation_list.append(QString("TY  - CONF\n"
                                      "T1  - 3d is here: Point cloud library (pcl)\n"
                                      "A1  - Rusu, Radu Bogdan\n"
                                      "A1  - Cousins, Steve\n"
                                      "JO  - Robotics and Automation (ICRA), 2011 IEEE International Conference on\n"
                                      "SP  - 1\n"
                                      "EP  - 4\n"
                                      "SN  - 1612843859\n"
                                      "Y1  - 2011\n"
                                      "PB  - IEEE\n"
                                      "UL  - http://pointclouds.org/documentation/tutorials/statistical_outlier.php\n"
                                      "ER  - \n"));
    return _RIS_citation_list;
}

void SF_DTM_Generator::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *res_model = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
    assert(res_model != NULL);
    res_model->setZeroOrMoreRootGroup();
    res_model->addGroupModel("", DEF_IN_GRP, CT_AbstractItemGroup::staticGetType(), tr("Point Cloud Grp In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    res_model->addItemModel(DEF_IN_GRP, DEF_IN_CLOUD, CT_Scene::staticGetType(), tr("Point Cloud"));
    res_model->addGroupModel("", DEF_IN_SCENE, CT_AbstractItemGroup::staticGetType(), tr("Input Scene Group"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    res_model->addItemModel(DEF_IN_SCENE, DEF_IN_SCENE_CLOUD, CT_Scene::staticGetType(), tr("Input Scene"));
}

void SF_DTM_Generator::createPostConfigurationDialogExpert(CT_StepConfigurableDialog *config_dialog) {
    config_dialog->addDouble("First the cloud is downscaled to a voxel size of  ",   " (m). " , 0.015,0.1,3,_voxel_size );
    config_dialog->addDouble("For each of the downscaled points its normal is computed with a range search of  ", "  (m). " , 0.025,0.5,3,_radius_normal );
    config_dialog->addText("That range search radius is also used as the approximated cell size of the DTM.");
    config_dialog->addDouble("The angle between a plane normal and the parent plane normal has to be smaller than ", " " , 0.5,45,1,_angle );
    config_dialog->addText("degrees.");
}

void SF_DTM_Generator::createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *config_dialog) {
    config_dialog->addStringChoice("Choose the slope of the terrain","",_non_expert_level, _choice);
}

void SF_DTM_Generator::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *res_modelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(res_modelw != NULL) {
        res_modelw->addGroupModel(DEF_IN_SCENE, _outGroundGRP, new CT_StandardItemGroup(), tr ("Terrain") );
        res_modelw->addItemModel(_outGroundGRP, _outDTM, new CT_Image2D<float>(), tr("DTM"));
        res_modelw->addItemModel(_outGroundGRP, _outCloud, new CT_Scene(), tr("Ground points"));
    }
}

void SF_DTM_Generator::adapt_parameters_to_expert_level() {
    _radius_normal = std::max(3*_voxel_size,_radius_normal);
    if(!_is_expert) {
        if(_choice == _less) {
            _angle = 10;
            _radius_normal = 0.15;
            _voxel_size = 0.05;
        } else if(_choice == _intermediate) {
            _angle = 20;
            _radius_normal = 0.15;
            _voxel_size = 0.05;
        } else {
            _angle = 40;
            _radius_normal = 0.15;
            _voxel_size = 0.05;
        }
    }
}

void SF_DTM_Generator::copyCroppedHeights(pcl::PointCloud<pcl::PointXYZINormal>::Ptr groundCloud, std::shared_ptr<CT_Image2D<float> > dtmPtr, CT_Image2D<float>* CTDTM) {
    float minZ = std::numeric_limits<float>::max();
    float maxz = std::numeric_limits<float>::lowest();
    for(size_t i = 0; i < groundCloud->points.size(); i++) {
        pcl::PointXYZINormal p = groundCloud->points[i];
        if(p.z < minZ) minZ = p.z;
        if(p.z > maxz) maxz = p.z;
    }

    for(size_t i = 0; i < CTDTM->xArraySize(); i++) {
        for(size_t j = 0; j < CTDTM->yArraySize(); j++) {
            size_t indexCTDTM;
            CTDTM->index(i,j,indexCTDTM);
            Eigen::Vector2d bot, top, center;
            CTDTM->getCellCoordinates(indexCTDTM, bot,top);
            center[0] = (bot[0] + top[0])/2- _translate(0);
            center[1] = (bot[1] + top[1])/2- _translate(1);
            size_t indexdtmPtr;
            dtmPtr->indexAtCoords(center[0],center[1],indexdtmPtr);

            float height = dtmPtr->valueAtIndex(indexdtmPtr);
            if(height < minZ) height = minZ;
            if(height > maxz) height = maxz;

            CTDTM->setValueAtIndex(indexCTDTM,height+_translate(2));
        }
    }
}

void SF_DTM_Generator::compute() {
    const QList<CT_ResultGroup*> &out_result_list = getOutResultList();
    CT_ResultGroup * out_result = out_result_list.at(0);
    identify_and_remove_corrupted_scenes(out_result);
    CT_ResultGroupIterator iter(out_result,this, DEF_IN_SCENE);
    CT_StandardItemGroup* root = (CT_StandardItemGroup*) iter.next();
    CT_StandardItemGroup* filter_grp = new CT_StandardItemGroup( _outGroundGRP.completeName(), out_result);
    root->addGroup(filter_grp);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr groundCloud = create_ground_cloud(out_result, filter_grp);
    SF_DTM<pcl::PointXYZINormal> sfDTM(groundCloud, _angle, _radius_normal, out_result, _outDTM2);
    std::shared_ptr<CT_Image2D<float> > dtmPtr = sfDTM.DTM();
    CT_Image2D<float> * CTDTM = CT_Image2D<float>::createImage2DFromXYCoords(_outDTM.completeName(),out_result,
                                                                           dtmPtr->minX()+_translate(0), dtmPtr->minY()+_translate(1),
                                                                           dtmPtr->maxX()+_translate(0), dtmPtr->maxY()+_translate(1),
                                                                           dtmPtr->resolution(), _translate(2), 1337,0);

    copyCroppedHeights(groundCloud, dtmPtr, CTDTM);
    CTDTM->computeMinMax();
    filter_grp->addItemDrawable(CTDTM);
}

void SF_DTM_Generator::write_logger() {
}

void SF_DTM_Generator::computeNormals(pcl::PointCloud<pcl::PointXYZINormal>::Ptr downscaledCloud) {
    pcl::NormalEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal> ne;
    ne.setInputCloud (downscaledCloud);
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (_radius_normal);
    ne.compute (*downscaledCloud);
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr SF_DTM_Generator::convertDownScale(CT_Scene* scene) {
    SF_Converter_CT_To_PCL<pcl::PointXYZINormal> converter;
    converter.set_itemCpy_cloud_in(scene);
    converter.compute();
    _translate =  converter.get_center_of_mass();
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud = converter.get_cloud_translated();
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr downscaledCloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::VoxelGrid<pcl::PointXYZINormal> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (_voxel_size, _voxel_size, _voxel_size);
    sor.filter (*downscaledCloud);
    return downscaledCloud;
}

CT_Scene * SF_DTM_Generator::addGroundCloudToResult(CT_PointCloudIndexVector *mergedClouds, CT_StandardItemGroup* root, CT_ResultGroup *out_result) {
    mergedClouds->setSortType(CT_PointCloudIndexVector::SortedInAscendingOrder);
    CT_Scene* scene (new CT_Scene(_outCloud.completeName(), out_result, PS_REPOSITORY->registerPointCloudIndex(mergedClouds)));
    scene->updateBoundingBox();
    root->addItemDrawable(scene);
    return scene;
}

CT_PointCloudIndexVector * SF_DTM_Generator::mergeIndices(CT_ResultGroupIterator out_res_it) {
    CT_PointCloudIndexVector *mergedClouds = new CT_PointCloudIndexVector();
    mergedClouds->setSortType(CT_AbstractCloudIndex::NotSorted);
    std::vector<size_t> indices;
    while(!isStopped() && out_res_it.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) out_res_it.next();
        const CT_AbstractItemDrawableWithPointCloud* ct_cloud = (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD);
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
    return mergedClouds;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr SF_DTM_Generator::create_ground_cloud(CT_ResultGroup *out_result, CT_StandardItemGroup* root) {
    adapt_parameters_to_expert_level();
    CT_ResultGroupIterator out_res_it(out_result, this, DEF_IN_GRP);
    CT_PointCloudIndexVector *mergedClouds = mergeIndices(out_res_it);
    CT_Scene* scene = addGroundCloudToResult(mergedClouds, root, out_result);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr downscaledCloud = convertDownScale(scene);
    computeNormals(downscaledCloud);
    return downscaledCloud;
}
