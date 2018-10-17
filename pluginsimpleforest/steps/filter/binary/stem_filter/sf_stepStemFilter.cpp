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
#include "sf_stepStemFilter.h"

#include "sf_stepStemFilterAdapter.h"
#include <QtConcurrent/QtConcurrent>

SF_StepStemFilter::SF_StepStemFilter(CT_StepInitializeData &dataInit):
    SF_AbstractFilterBinaryStep(dataInit) {
}

SF_StepStemFilter::~SF_StepStemFilter() {

}

QString SF_StepStemFilter::getStepDescription() const {
    return tr("Stem Filter");
}

QString SF_StepStemFilter::getStepDetailledDescription() const {
    return tr("Stem Filter - This Filter estimates for each point the growth direction of the underlying branch segment. The angle between this direction vector and the z axis is computed. "
              "If the angle is small, the point is detected as stem, if large the point is considered non stem.");
}

QString SF_StepStemFilter::getStepURL() const {
    return tr("https://www.youtube.com/watch?v=5i6_Rtv-xEw");
}

CT_VirtualAbstractStep* SF_StepStemFilter::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_StepStemFilter(dataInit);
}

QStringList SF_StepStemFilter::getStepRISCitations() const {
    QStringList _risCitationList;
    _risCitationList.append(getRISCitationSimpleTree());
    _risCitationList.append(getRISCitationPCL());
    _risCitationList.append(getRISCitationRaumonen());
    return _risCitationList;
}

void SF_StepStemFilter::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *res_model = createNewInResultModelForCopy(DEF_IN_RESULT,
                                                                           tr("Point Cloud"));
    assert(res_model != NULL);
    res_model->setZeroOrMoreRootGroup();
    res_model->addGroupModel("",
                             DEF_IN_GRP_CLUSTER,
                             CT_AbstractItemGroup::staticGetType(),
                             tr("Group to be denoised"),
                             "",
                             CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    res_model->addItemModel(DEF_IN_GRP_CLUSTER,
                            DEF_IN_CLOUD_SEED,
                            CT_Scene::staticGetType(),
                            tr("Cloud to be denoised"));
}

void SF_StepStemFilter::createPostConfigurationDialogExpert(CT_StepConfigurableDialog *configDialog) {
    configDialog->addDouble("First the cloud is downscaled to a voxel size of  ",
                             " (m). ",
                             0.015,
                             0.1,
                             3,
                             _voxelSize );
    configDialog->addDouble("For each of the downscaled points its normal is computed with a range search of  ",
                             "  (m). ",
                             0.025,
                             0.2,
                             3,
                             _radiusNormal);
    configDialog->addDouble("Another neighborhood range search is performed on the downscaled points with range  ",
                             " (m).",
                             0.05,
                             0.5,
                             3,
                             _radiusGrowthDirection);
    configDialog->addText("The second range should be larger than the first, which should be larger than the downscale size.");
    configDialog->addText("For each point the Covariance matrix on all neighboring points within the second range is build and a PCA performed.");
    configDialog->addText("The eigenvector representing the direction along the smallest variance is taken as the growth direction of the point.");
    configDialog->addDouble("The angle for each point between this eigenvector and the z axis is computed and is not allowed to deviate more than ",
                             " ",
                             0.5,
                             180,
                             1,
                             _angle);
    configDialog->addText("degrees.");
    configDialog->addText("Please read Raumonen <b>2013</b> (see Citation menu) for more information, this step is based on knowledge gained there.");
}

void SF_StepStemFilter::createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *configDialog) {
    configDialog->addStringChoice("Choose how many points should be removed",
                                  "",
                                  _numberPoints,
                                  _choiceNumberPoints);
    configDialog->addText("For bended trees select a weaker filter level.");
}

void SF_StepStemFilter::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(resModelw != NULL) {
        resModelw->addGroupModel(DEF_IN_GRP_CLUSTER,
                                 _outGrp,
                                 new CT_StandardItemGroup(),
                                 tr ("Stem Point Filter"));
        resModelw->addItemModel(_outGrp,
                                _outCloud,
                                new CT_Scene(),
                                tr("Cloud"));
        resModelw->addItemModel(_outGrp,
                                _outNoise,
                                new CT_Scene(),
                                tr("Noise"));
    }
}

void SF_StepStemFilter::adaptParametersToExpertLevel() {
    if(!_isExpert) {
        if(_choiceNumberPoints == _few) {
            _x = 0;
            _y = 0;
            _z = 1;
            _angle = 15;
            _radiusGrowthDirection = 0.45;
            _radiusNormal = 0.04;
            _voxelSize = 0.015;
            _sizeOutput = 2;
        } else if(_choiceNumberPoints == _intermediate) {
            _x = 0;
            _y = 0;
            _z = 1;
            _angle = 30;
            _radiusGrowthDirection = 0.45;
            _radiusNormal = 0.04;
            _voxelSize = 0.015;
            _sizeOutput = 2;
        } else if(_choiceNumberPoints == _many) {
            _x = 0;
            _y = 0;
            _z = 1;
            _angle = 45;
            _radiusGrowthDirection = 0.45;
            _radiusNormal = 0.04;
            _voxelSize = 0.015;
            _sizeOutput = 2;
        }
    }
}

void SF_StepStemFilter::writeOutputPerScence(CT_ResultGroup* outResult,
                                                  size_t i) {
    SF_ParamStemFilter<SF_PointNormal> param = _paramList.at(i);
    std::vector<CT_PointCloudIndexVector *> outputIndexList = createOutputVectors(param._sizeOutput);
    createOutputIndices(outputIndexList,
                        param._outputIndices,
                        param._itemCpyCloudIn);
    CT_StandardItemGroup *filterGrp = new CT_StandardItemGroup(_outGrp.completeName(),
                                                               outResult);
    param._grpCpyGrp->addGroup(filterGrp);
    addSceneToFilterGrp(filterGrp,
                        outResult,
                        outputIndexList[0],
            _outCloud.completeName());
    addSceneToFilterGrp(filterGrp,
                        outResult,
                        outputIndexList[1],
            _outNoise.completeName());
}

void SF_StepStemFilter::writeOutput(CT_ResultGroup* outResult) {
    size_t size = _paramList.size();
    for(size_t i = 0; i < size; i ++) {
        writeOutputPerScence(outResult,
                             i);
    }
}

void SF_StepStemFilter::compute() {
    const QList<CT_ResultGroup*> &outResultList = getOutResultList();
    CT_ResultGroup * outResult = outResultList.at(0);
    identifyAndRemoveCorruptedScenes(outResult);
    createParamList(outResult);
    QFuture<void> future = QtConcurrent::map(_paramList,
                                             SF_StepStemFilterAdapter() );
    setProgressByFuture(future,
                        10,
                        85);
    writeOutput(outResult);
    writeLogger();
    _paramList.clear();
}

void SF_StepStemFilter::writeLogger() {
    if(!_paramList.empty()) {
        auto strList = _paramList[0].toStringList();
        for(auto &str : strList) {
            PS_LOG->addMessage(LogInterface::info,
                               LogInterface::step,
                               str);
        }
        size_t filtered = 0;
        size_t total = 0;
        for(auto const &param : _paramList) {
            auto vector = param._outputIndices;
            for(auto i : vector) {
                total++;
                filtered += static_cast<size_t> (i);
            }
        }
        auto str2 = _paramList[0].toFilterString(total,
                                                    filtered);
        PS_LOG->addMessage(LogInterface::info,
                           LogInterface::step,
                           str2);
    }
}

void SF_StepStemFilter::createParamList(CT_ResultGroup * outResult) {
    adaptParametersToExpertLevel();
    CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
    while(!isStopped() && outResIt.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) outResIt.next();
        const CT_AbstractItemDrawableWithPointCloud* ct_cloud =
                (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
        SF_ParamStemFilter<SF_PointNormal> param;
        param._log = PS_LOG;
        param._x = _x;
        param._y = _y;
        param._z = _z;
        param._angle = _angle;
        param._radiusGrowthDirection = _radiusGrowthDirection;
        param._radiusNormal = _radiusNormal;
        param._voxelSize = _voxelSize;
        param._sizeOutput = 2;
        param._itemCpyCloudIn = ct_cloud;
        param._grpCpyGrp = group;
        _paramList.append(param);
    }
}
