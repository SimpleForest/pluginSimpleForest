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

#include <pcl/features/normal_3d.h>

#include "sf_dtm_step.h"
#include "converters/CT_To_PCL/sf_converterCTToPCL.h"
#include "converters/CT_To_PCL/sf_converterCTToPCLDTM.h"
#include "pcl/geometry/DTM/sf_dtm.h"
#include "pcl/filters/voxel_grid.h"

SF_StepDTM::SF_StepDTM(CT_StepInitializeData &dataInit):
    SF_AbstractStep(dataInit) {
    _nonExpertLevel.append(_less);
    _nonExpertLevel.append(_intermediate);
    _nonExpertLevel.append(_many);
}

SF_StepDTM::~SF_StepDTM() {

}

QString SF_StepDTM::getStepDescription() const {
    return tr("DTM generation");
}

QString SF_StepDTM::getStepDetailledDescription() const {
    return tr("DTM generation - uses a pyramidal MLESAC plane fitting apporach on a robust downscaled input ground cloud.");
}

QString SF_StepDTM::getStepURL() const {
    return tr("https://www.youtube.com/watch?v=5i6_Rtv-xEw");
}

CT_VirtualAbstractStep* SF_StepDTM::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_StepDTM(dataInit);
}

QStringList SF_StepDTM::getStepRISCitations() const {
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

void SF_StepDTM::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *res_model = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
    assert(res_model != NULL);
    res_model->setZeroOrMoreRootGroup();
    res_model->addGroupModel("",
                             DEF_IN_GRP_CLUSTER,
                             CT_AbstractItemGroup::staticGetType(),
                             tr("Point Cloud Grp In"),
                             "",
                             CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    res_model->addItemModel(DEF_IN_GRP_CLUSTER,
                            DEF_IN_CLOUD_SEED,
                            CT_Scene::staticGetType(),
                            tr("Point Cloud"));
    res_model->addGroupModel("",
                             DEF_IN_SCENE,
                             CT_AbstractItemGroup::staticGetType(),
                             tr("Input Scene Group"),
                             "",
                             CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    res_model->addItemModel(DEF_IN_SCENE,
                            DEF_IN_SCENE_CLOUD,
                            CT_Scene::staticGetType(),
                            tr("Input Scene"));
}

void SF_StepDTM::createPostConfigurationDialogExpert(CT_StepConfigurableDialog *configDialog) {
    configDialog->addDouble("First the cloud is downscaled to a voxel size of  ",
                             " (m).",
                             0.015,
                             0.1,
                             3,
                             _voxelSize);
    configDialog->addDouble("For each of the downscaled points its normal is computed with a range search of  ",
                             " (m). ",
                             0.025,
                             0.5,
                             3,
                             _radiusNormal);
    configDialog->addDouble("The cell size of the DTM is supposed to be  ",
                             " (m).",
                             0.025,
                             0.5,
                             3,
                             _cellSize);
    configDialog->addInt("For IDW interpolation the following number of nearest neighbors is needed " ,
                          ".",
                          1,
                          99,
                          _idwNeighbors);
    configDialog->addInt("For median interpolation the following number of nearest neighbors is needed " ,
                          ".",
                          1,
                          99,
                          _medianNeighbors);
    configDialog->addDouble("The angle between a plane normal and the parent plane normal has to be smaller than ",
                             " (°).",
                             0.5,
                             45,
                             1,
                             _angle );
}

void SF_StepDTM::createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *configDialog) {
    configDialog->addStringChoice("Choose the slope of the terrain","",_nonExpertLevel, _choice);
}

void SF_StepDTM::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *res_modelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(res_modelw != NULL) {
        res_modelw->addGroupModel(DEF_IN_SCENE, _outGroundGRP, new CT_StandardItemGroup(), tr ("Terrain") );
        res_modelw->addItemModel(_outGroundGRP, _outDTM, new CT_Image2D<float>(), tr("DTM"));
        res_modelw->addItemModel(_outGroundGRP, _outCloud, new CT_Scene(), tr("Ground points"));
    }
}

void SF_StepDTM::adaptParametersToExpertLevel() {
    _radiusNormal = std::max(3*_voxelSize,_radiusNormal);
    if(!_isExpert) {
        if(_choice == _less) {
            _angle = 10;
            _radiusNormal = 0.15;
            _voxelSize = 0.05;
        } else if(_choice == _intermediate) {
            _angle = 20;
            _radiusNormal = 0.15;
            _voxelSize = 0.05;
        } else {
            _angle = 40;
            _radiusNormal = 0.15;
            _voxelSize = 0.05;
        }
        _cellSize = 0.2;
        _medianNeighbors = 9;
        _idwNeighbors = 3;
    }
}

void SF_StepDTM::copyCroppedHeights(pcl::PointCloud<pcl::PointXYZINormal>::Ptr groundCloud,
                                    std::shared_ptr<CT_Image2D<float> > dtmPtr,
                                    CT_Image2D<float>* CTDTM) {
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

void SF_StepDTM::compute() {
    adaptParametersToExpertLevel();
    const QList<CT_ResultGroup*> &out_result_list = getOutResultList();
    CT_ResultGroup * out_result = out_result_list.at(0);
    identifyAndRemoveCorruptedScenes(out_result);
    CT_ResultGroupIterator iter(out_result,this, DEF_IN_SCENE);
    CT_StandardItemGroup* root = (CT_StandardItemGroup*) iter.next();
    CT_StandardItemGroup* terrainGrp = new CT_StandardItemGroup( _outGroundGRP.completeName(),
                                                                 out_result);
    root->addGroup(terrainGrp);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr groundCloud = createGroundCloud(out_result,
                                                                               terrainGrp);
    SF_DTM<pcl::PointXYZINormal> sfDTM(groundCloud,
                                       _angle,
                                       _cellSize,
                                       out_result,
                                       _outDTMDummy);
    std::shared_ptr<CT_Image2D<float> > dtmPtr = sfDTM.DTM();
    CT_Image2D<float> * dtm = CT_Image2D<float>::createImage2DFromXYCoords(_outDTMDummy.completeName(),
                                                                           out_result,
                                                                           dtmPtr->minX()+_translate(0),
                                                                           dtmPtr->minY()+_translate(1),
                                                                           dtmPtr->maxX()+_translate(0),
                                                                           dtmPtr->maxY()+_translate(1),
                                                                           dtmPtr->resolution(),
                                                                           _translate(2),
                                                                           1337,
                                                                           0);
    copyCroppedHeights(groundCloud, dtmPtr, dtm);
    SF_ConverterCTToPCLDTM dtmConverter(_translate,dtm);
    std::shared_ptr<SF_ModelDTM> dtmModel = dtmConverter.dtmPCL();

    CT_Image2D<float> * dtmTrueResolution = CT_Image2D<float>::createImage2DFromXYCoords(_outDTMDummy.completeName(),
                                                                                         out_result,
                                                                                         dtmPtr->minX()+_translate(0),
                                                                                         dtmPtr->minY()+_translate(1),
                                                                                         dtmPtr->maxX()+_translate(0),
                                                                                         dtmPtr->maxY()+_translate(1),
                                                                                         _radiusNormal,
                                                                                         _translate(2),
                                                                                         1337,
                                                                                         0);
    SF_ConverterCTToPCLDTM dtmConverterTrueResolution(_translate,
                                                      dtmTrueResolution);
    std::shared_ptr<SF_ModelDTM> dtmModelTrueResolution = dtmConverterTrueResolution.dtmPCL();
    dtmModel->interpolateIDW(_idwNeighbors,
                             dtmModelTrueResolution);
    CT_Image2D<float> * dtmMedianSmoothed = CT_Image2D<float>::createImage2DFromXYCoords(_outDTM.completeName(),
                                                                                         out_result,
                                                                                         dtmPtr->minX()+_translate(0),
                                                                                         dtmPtr->minY()+_translate(1),
                                                                                         dtmPtr->maxX()+_translate(0),
                                                                                         dtmPtr->maxY()+_translate(1),
                                                                                         _radiusNormal,
                                                                                         _translate(2),
                                                                                         1337,
                                                                                         0);
    SF_ConverterCTToPCLDTM dtmConverterMedianSmoothed(_translate,
                                                      dtmMedianSmoothed);
    std::shared_ptr<SF_ModelDTM> dtmModelMedianSmoothed = dtmConverterMedianSmoothed.dtmPCL();
    dtmModelTrueResolution->interpolateMedian(_medianNeighbors,
                                              dtmModelMedianSmoothed);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = dtmModelMedianSmoothed->getCloud3D();
    for(size_t i = 0; i < dtmMedianSmoothed->nCells(); i++) {
        dtmMedianSmoothed->setValueAtIndex(i,
                                           cloud->points[i].z+_translate(2));
    }
    dtmMedianSmoothed->computeMinMax();
    terrainGrp->addItemDrawable(dtmMedianSmoothed);
    delete dtmTrueResolution;
    delete dtm;
    writeLogger();
}

void SF_StepDTM::writeLogger() {
    QString str = "The DTM was modelled with a cell size of ";
    str.append(QString::number(_radiusNormal));
    str.append(" (m).");
    PS_LOG->addMessage(LogInterface::info,
                       LogInterface::step,
                       str);
}

void SF_StepDTM::computeNormals(pcl::PointCloud<pcl::PointXYZINormal>::Ptr downscaledCloud) {
    pcl::NormalEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal> ne;
    ne.setInputCloud (downscaledCloud);
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (_radiusNormal);
    ne.compute (*downscaledCloud);
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr SF_StepDTM::downScale(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr downscaledCloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::VoxelGrid<pcl::PointXYZINormal> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (_voxelSize, _voxelSize, _voxelSize);
    sor.filter (*downscaledCloud);
    return downscaledCloud;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr SF_StepDTM::convert(CT_Scene* scene) {
    Sf_ConverterCTToPCL<pcl::PointXYZINormal> converter;
    converter.setItemCpyCloudIn(scene);
    converter.compute();
    _translate =  converter.getCenterOfMass();
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud = converter.getCloudTranslated();
    return cloud;
}

CT_Scene * SF_StepDTM::addGroundCloudToResult(CT_PointCloudIndexVector *mergedClouds,
                                              CT_StandardItemGroup* root,
                                              CT_ResultGroup *outResult) {
    CT_Scene* scene (new CT_Scene(_outCloud.completeName(), outResult, PS_REPOSITORY->registerPointCloudIndex(mergedClouds)));
    scene->updateBoundingBox();
    root->addItemDrawable(scene);
    return scene;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr SF_StepDTM::createGroundCloud(CT_ResultGroup *outResult,
                                                                         CT_StandardItemGroup* terrainGrp) {
    CT_Scene* scene = mergeIndices(outResult, terrainGrp, DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud = convert(scene);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr downscaledCloud = downScale(cloud);
    computeNormals(downscaledCloud);
    return downscaledCloud;
}
