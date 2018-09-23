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
    _nonExpertLevel.append(_less);
    _nonExpertLevel.append(_intermediate);
    _nonExpertLevel.append(_many);
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


    _RIS_citation_list.append(QString("TY  - EJOU\n"
                                      "T1  - Fast Automatic Precision Tree Models from Terrestrial Laser Scanner Data\n"
                                      "A1  - Raumonen, Pasi\n"
                                      "A1  - Kaasalainen, Mikko\n"
                                      "A1  - Akerblom, Markku\n"
                                      "A1  - Kaasalainen, Sanna\n"
                                      "A1  - Kaartinen, Harri\n"
                                      "A1  - Vastaranta, Mikko\n"
                                      "A1  - Holopainen, Markus\n"
                                      "A1  - Disney, Mathias\n"
                                      "A1  - Lewis, Philip\n"
                                      "JO  - Remote Sensing\n"
                                      "SP  - 5\n"
                                      "EP  - 2\n"
                                      "SN  - 2072-4292\n"
                                      "Y1  - 2013\n"
                                      "PB  - MDPI\n"
                                      "UL  - http://www.mdpi.com/2072-4292/5/2/491\n"
                                      "ER  - \n"));
    return _RIS_citation_list;
}

void SF_StepStemFilter::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *res_model = createNewInResultModelForCopy(DEF_IN_RESULT,
                                                                           tr("Point Cloud"));
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
    configDialog->addDouble("The angle for each point between this eigenvector and the adjusted z axis is computed and is not allowed to deviate more than ",
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
    configDialog->addDouble("The y-component of the adjusted z-axis", " ",
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
    configDialog->addText("Please read Raumonen <b>2013</b> (see Citation menu) for more information, this step is based on knowledge gained there.");
}

void SF_StepStemFilter::createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *configDialog) {
    configDialog->addStringChoice("Choose how many points should be removed",
                                   "",
                                   _nonExpertLevel,
                                   _choice);
    configDialog->addText("For bended trees select a weaker filter level.");
}

void SF_StepStemFilter::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(resModelw != NULL) {
        resModelw->addGroupModel(DEF_IN_GRP_CLUSTER, _outGrp, new CT_StandardItemGroup(), tr ("stem filtered removal") );
        resModelw->addGroupModel(_outGrp, _outGrpCloud, new CT_StandardItemGroup(), tr ("stem") );
        resModelw->addGroupModel(_outGrp, _outGrpNoise, new CT_StandardItemGroup(), tr ("not stem") );
        resModelw->addItemModel(_outGrpCloud, _outCloud, new CT_Scene(), tr("cloud"));
        resModelw->addItemModel(_outGrpNoise, _outNoise, new CT_Scene(), tr("cloud"));
    }
}

void SF_StepStemFilter::adaptParametersToExpertLevel() {
    if(!_isExpert) {
        if(_choice == _less) {
            _x = 0;
            _y = 0;
            _z = 1;
            _angle = 15;
            _radiusGrowthDirection = 0.45;
            _radiusNormal = 0.04;
            _voxelSize = 0.015;
            _sizeOutput = 2;
        } else if(_choice == _intermediate) {
            _x = 0;
            _y = 0;
            _z = 1;
            _angle = 30;
            _radiusGrowthDirection = 0.45;
            _radiusNormal = 0.04;
            _voxelSize = 0.015;
            _sizeOutput = 2;
        } else {
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

void SF_StepStemFilter::writeOutputPerScence(CT_ResultGroup* out_result,
                                                  size_t i) {
    SF_ParamStemFilter<SF_PointNormal> param = _paramList.at(i);
    std::vector<CT_PointCloudIndexVector *> output_index_list = createOutputVectors(param._sizeOutput);
    createOutputIndices(output_index_list,
                        param._outputIndices,
                        param._itemCpyCloudIn);
    CT_StandardItemGroup* filter_grp = new CT_StandardItemGroup(_outGrp.completeName(),
                                                                out_result);
    param._grpCpyGrp->addGroup(filter_grp);
    addSceneInSubgrpToGrp(filter_grp,
                          out_result,
                          output_index_list[0],
                         _outCloud.completeName(),
                         _outGrpCloud.completeName());
    addSceneInSubgrpToGrp(filter_grp,
                          out_result,
                          output_index_list[1],
                          _outNoise.completeName(),
                          _outGrpNoise.completeName());
}

void SF_StepStemFilter::writeOutput(CT_ResultGroup* out_result) {
    size_t size = _paramList.size();
    for(size_t i = 0; i < size; i ++) {
        writeOutputPerScence(out_result, i);
    }
}

void SF_StepStemFilter::compute() {
    const QList<CT_ResultGroup*> &outResultList = getOutResultList();
    CT_ResultGroup * out_result = outResultList.at(0);
    identifyAndRemoveCorruptedScenes(out_result);
    createParamList(out_result);
    writeLogger();

    QFuture<void> future = QtConcurrent::map(_paramList,SF_StepStemFilterAdapter() );
    setProgressByFuture(future,10,85);
    writeOutput(out_result);
}

void SF_StepStemFilter::writeLogger() {
    if(!_paramList.empty()) {
        QString str = _paramList[0].toString();
        PS_LOG->addMessage(LogInterface::info, LogInterface::step, str);
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
