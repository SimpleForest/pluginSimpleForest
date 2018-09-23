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

#include "sf_stepStemRANSACFilter.h"

#include "sf_stepStemFilterRANSACAdapter.h"
#include <QtConcurrent/QtConcurrent>

SF_StepStemRANSACFilter::SF_StepStemRANSACFilter(CT_StepInitializeData &dataInit):
    SF_AbstractFilterBinaryStep(dataInit) {
}

SF_StepStemRANSACFilter::~SF_StepStemRANSACFilter() {
}

QString SF_StepStemRANSACFilter::getStepDescription() const {
    return tr("Stem Filter by RANSAC fitting");
}

QString SF_StepStemRANSACFilter::getStepDetailledDescription() const {
    return tr("Stem Filter by RANSAC - Fits robust RANSAC cylinders into one meter height horizontal slices. If the cylinder per slice passes z axis check, its inliers"
              "are put into the output cloud. All other points are noise.");
}

QString SF_StepStemRANSACFilter::getStepURL() const {
    return tr("https://www.youtube.com/watch?v=5i6_Rtv-xEw");
}

CT_VirtualAbstractStep* SF_StepStemRANSACFilter::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_StepStemRANSACFilter(dataInit);
}

QStringList SF_StepStemRANSACFilter::getStepRISCitations() const {
    QStringList _risCitationList;
    _risCitationList.append(QString("TY  - JOUR\n"
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


    _risCitationList.append(QString("TY  - CONF\n"
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
    return _risCitationList;
}

void SF_StepStemRANSACFilter::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *resModel = createNewInResultModelForCopy(DEF_IN_RESULT,
                                                                           tr("Point Cloud"));
    assert(resModel != NULL);
    resModel->setZeroOrMoreRootGroup();
    resModel->addGroupModel("",
                             DEF_IN_GRP_CLUSTER,
                             CT_AbstractItemGroup::staticGetType(),
                             tr("Cloud Grp "),
                             "",
                             CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resModel->addItemModel(DEF_IN_GRP_CLUSTER,
                            DEF_IN_CLOUD_SEED,
                            CT_Scene::staticGetType(),
                            tr("Point Cloud"));
}

void SF_StepStemRANSACFilter::createPostConfigurationDialogExpert(CT_StepConfigurableDialog *configDialog) {
    configDialog->addDouble("First the cloud is downscaled to a voxel size of  ",
                             " (m). " ,
                             0.005,
                             0.1,
                             3,
                             _voxelSize );
    configDialog->addDouble("For each of the downscaled points its normal is computed with a range search of  ",
                             "  (m). ",
                             0.025,
                             0.2,
                             3,
                             _radiusNormal);
    configDialog->addDouble("Into 1 meter horizontal slices a RANSAC cylinder is fitted with   ",
                             "  (m). ",
                             0.01,
                             0.5,
                             2,
                             _inlierDistance );
    configDialog->addText(" inlier Distance. All inliers per slice are set to stem if the cylinder passes the following test:");
    configDialog->addDouble("The angle for each point between this cylinder axis and the adjusted z axis is computed and is not allowed to deviate more than ",
                             " ",
                             0.5,
                             180,
                             1,
                             _angle);
    configDialog->addText("degrees.");
    configDialog->addDouble("The x-component of the adjusted z-axis",
                             " ",
                             0.01,
                             1,
                             2,
                             _x);
    configDialog->addDouble("The y-component of the adjusted z-axis",
                             " ",
                             0.01,
                             1,
                             2,
                             _y);
    configDialog->addDouble("The z-component of the adjusted z-axis",
                             " ",
                             0.01,
                             1,
                             2,
                             _z);
}

void SF_StepStemRANSACFilter::createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *configDialog) {
    configDialog->addStringChoice("Choose how many points should be removed",
                                   "",
                                   _nonExpertLevel,
                                   _choice);
    configDialog->addText("For bended trees select a weaker filter level. Also select weaker level for worse clouds.");
}

void SF_StepStemRANSACFilter::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(resModelw != NULL) {
        resModelw->addGroupModel(DEF_IN_GRP_CLUSTER,
                                  _outGrp,
                                  new CT_StandardItemGroup(),
                                  tr ("Stem RANSAC filter"));
        resModelw->addGroupModel(_outGrp,
                                  _outGrpCloud,
                                  new CT_StandardItemGroup(),
                                  tr ("stem"));
        resModelw->addGroupModel(_outGrp,
                                  _outGrpNoise,
                                  new CT_StandardItemGroup(),
                                  tr ("not stem") );
        resModelw->addItemModel(_outGrpCloud,
                                 _outCloud,
                                 new CT_Scene(),
                                 tr("cloud"));
        resModelw->addItemModel(_outGrpNoise,
                                 _outNoise,
                                 new CT_Scene(),
                                 tr("cloud"));
    }
}

void SF_StepStemRANSACFilter::adaptParametersToExpertLevel() {
    if(!_isExpert) {
        if(_choice == _less) {
            _x = 0;
            _y = 0;
            _z = 1;
            _angle = 15;
            _radiusNormal = 0.04;
            _inlierDistance = 0.05;
            _voxelSize = 0.015;
            _sizeOutput = 2;
        } else if(_choice == _intermediate) {
            _x = 0;
            _y = 0;
            _z = 1;
            _angle = 30;
            _radiusNormal = 0.04;
            _inlierDistance = 0.1;
            _voxelSize = 0.015;
            _sizeOutput = 2;
        } else {
            _x = 0;
            _y = 0;
            _z = 1;
            _angle = 45;
            _radiusNormal = 0.04;
            _voxelSize = 0.015;
            _inlierDistance = 0.2;
            _sizeOutput = 2;
        }
    }
}

void SF_StepStemRANSACFilter::writeOutputPerScence(CT_ResultGroup* outResult, size_t i) {
    SF_ParamStemRansacFilter param = _paramList.at(i);
    std::vector<CT_PointCloudIndexVector *> output_index_list = createOutputVectors(param._sizeOutput);
    createOutputIndices(output_index_list,
                        param._outputIndices,
                        param._itemCpyCloudIn);
    CT_StandardItemGroup* filter_grp = new CT_StandardItemGroup(_outGrp.completeName(),
                                                                outResult);
    param._grpCpyGrp->addGroup(filter_grp);
    addSceneInSubgrpToGrp(filter_grp,
                          outResult,
                          output_index_list[0],
                          _outCloud.completeName(),
                          _outGrpCloud.completeName());
    addSceneInSubgrpToGrp(filter_grp,
                          outResult,
                          output_index_list[1],
                          _outNoise.completeName(),
                          _outGrpNoise.completeName());
}

void SF_StepStemRANSACFilter::writeOutput(CT_ResultGroup* outResult) {
    size_t size = _paramList.size();
    for(size_t i = 0; i < size; i ++) {
        writeOutputPerScence(outResult, i);
    }
}

void SF_StepStemRANSACFilter::compute() {
    const QList<CT_ResultGroup*> &out_result_list = getOutResultList();
    CT_ResultGroup * out_result = out_result_list.at(0);
    identifyAndRemoveCorruptedScenes(out_result);
    createParamList(out_result);
    writeLogger();

    QFuture<void> future = QtConcurrent::map(_paramList,SF_StepStemFilterRANSACAdapter() );
    setProgressByFuture(future,10,85);
    writeOutput(out_result);
}

void SF_StepStemRANSACFilter::writeLogger() {
    if(!_paramList.empty()) {
        QString str = _paramList[0].toString();
        PS_LOG->addMessage(LogInterface::info, LogInterface::step, str);
    }
}

void SF_StepStemRANSACFilter::createParamList(CT_ResultGroup * outResult) {
    adaptParametersToExpertLevel();
    CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
    while(!isStopped() && outResIt.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) outResIt.next();
        const CT_AbstractItemDrawableWithPointCloud* ctCloud = (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
        SF_ParamStemRansacFilter param;
        param._log = PS_LOG;
        param._x = _x;
        param._y = _y;
        param._z = _z;
        param._inlierDistance = _inlierDistance;
        param._angle = _angle;
        param._radiusNormal = _radiusNormal;
        param._voxelSize = _voxelSize;
        param._sizeOutput = 2;
        param._itemCpyCloudIn = ctCloud;
        param._grpCpyGrp = group;
        _paramList.append(param);
    }
}
