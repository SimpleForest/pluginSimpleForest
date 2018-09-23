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
#include "sf_radiusOutlierFilterStep.h"
#include "steps/filter/binary/radius_outlier_filter/sf_radiusOutlierFilterAdapter.h"
#include <QtConcurrent/QtConcurrent>

SF_RadiusOutlierFilterStep::SF_RadiusOutlierFilterStep(CT_StepInitializeData &data_init): SF_AbstractFilterBinaryStep(data_init) {
    _nonExpertLevel.append(_less);
    _nonExpertLevel.append(_intermediate);
    _nonExpertLevel.append(_many);
    _nonExpertLevel.append(_clearSky);
}

SF_RadiusOutlierFilterStep::~SF_RadiusOutlierFilterStep() {

}

QString SF_RadiusOutlierFilterStep::getStepDescription() const {
    return tr("Radius Outlier Removal");
}

QString SF_RadiusOutlierFilterStep::getStepDetailledDescription() const {
    return tr("Radius Outlier Removal - Frontend to PCL filter. Points with not enough neighbors in a supported range are eliminated.");
}

QString SF_RadiusOutlierFilterStep::getStepURL() const {
    return tr("http://pointclouds.org/documentation/tutorials/remove_outliers.php");
}

CT_VirtualAbstractStep* SF_RadiusOutlierFilterStep::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_RadiusOutlierFilterStep(dataInit);
}

QStringList SF_RadiusOutlierFilterStep::getStepRISCitations() const {
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

void SF_RadiusOutlierFilterStep::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *res_model = createNewInResultModelForCopy(DEF_IN_RESULT,
                                                                           tr("Point Cloud"));
    assert(res_model != NULL);    
    res_model->setZeroOrMoreRootGroup();
    res_model->addGroupModel("", DEF_IN_GRP_CLUSTER,
                             CT_AbstractItemGroup::staticGetType(),
                             tr("Point Cloud Grp In"),
                             "",
                             CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    res_model->addItemModel(DEF_IN_GRP_CLUSTER,
                            DEF_IN_CLOUD_SEED,
                            CT_Scene::staticGetType(),
                            tr("Point Cloud"));
}

void SF_RadiusOutlierFilterStep::createPostConfigurationDialogExpert(CT_StepConfigurableDialog *config_dialog) {
    config_dialog->addDouble("Looks for each point at its numbers of neighbors in range ",
                             "",
                             0.01,
                             4.1,
                             3,
                             _radius );
    config_dialog->addInt("A point is eliminated if it contains less than.",
                          " Points",
                          2,
                          99999,
                          _minPts);
}

void SF_RadiusOutlierFilterStep::createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *config_dialog) {
    config_dialog->addStringChoice("Choose how many points should be removed",
                                   "",
                                   _nonExpertLevel,
                                   _choice);
    config_dialog->addText("Low resulted clouds are affected more.");
}

void SF_RadiusOutlierFilterStep::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *res_modelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(res_modelw != NULL) {
        res_modelw->addGroupModel(DEF_IN_GRP_CLUSTER,
                                  _outGrp,
                                  new CT_StandardItemGroup(),
                                  tr ("radius outlier removal") );
        res_modelw->addGroupModel(_outGrp,
                                  _outGrpCloud,
                                  new CT_StandardItemGroup(),
                                  tr ("filtered") );
        res_modelw->addGroupModel(_outGrp,
                                  _outGrpNoise,
                                  new CT_StandardItemGroup(),
                                  tr ("noise") );
        res_modelw->addItemModel(_outGrpCloud,
                                 _outCloud,
                                 new CT_Scene(),
                                 tr("cloud"));
        res_modelw->addItemModel(_outGrpNoise,
                                 _outNoise,
                                 new CT_Scene(),
                                 tr("cloud"));
    }
}

void SF_RadiusOutlierFilterStep::writeOutputPerScence(CT_ResultGroup* outResult, size_t i) {
    SF_ParamRadiusOutlierFilter<SF_PointNormal> param = _paramList.at(i);
    std::vector<CT_PointCloudIndexVector *> outputIndexList = createOutputVectors(param._sizeOutput);
    createOutputIndices(outputIndexList,
                        param._outputIndices,
                        param._itemCpyCloudIn);
    CT_StandardItemGroup* filterGrp = new CT_StandardItemGroup(_outGrp.completeName(),
                                                                outResult);
    param._grpCpyGrp->addGroup(filterGrp);
    addSceneInSubgrpToGrp(filterGrp,
                          outResult,
                          outputIndexList[0],
                          _outCloud.completeName(),
                          _outGrpCloud.completeName());
    addSceneInSubgrpToGrp(filterGrp,
                          outResult,
                          outputIndexList[1],
                          _outNoise.completeName(),
                          _outGrpNoise.completeName());
}

void SF_RadiusOutlierFilterStep::writeOutput(CT_ResultGroup* outResult) {
    size_t size = _paramList.size();
    for(size_t i = 0; i < size; i ++) {
        writeOutputPerScence(outResult, i);
    }
}

void SF_RadiusOutlierFilterStep::compute() {
    const QList<CT_ResultGroup*> &outResultList = getOutResultList();
    CT_ResultGroup * outResult = outResultList.at(0);
    identifyAndRemoveCorruptedScenes(outResult);
    createParamList(outResult);
    writeLogger();
    QFuture<void> future = QtConcurrent::map(_paramList,
                                             SF_RadiusOutlierFilterAdapter() );
    setProgressByFuture(future,10,85);
    writeOutput(outResult);
}

void SF_RadiusOutlierFilterStep::writeLogger() {
    if(!_paramList.empty()) {
        QString str = _paramList[0].toString();
        PS_LOG->addMessage(LogInterface::info,
                           LogInterface::step,
                           str);
    }
}

void SF_RadiusOutlierFilterStep::adaptParametersToExpertLevel() {
    if(!_isExpert) {
        _radius = 0.03;
        if(_choice == _less) {
            _minPts = 5;
        } else if(_choice == _intermediate) {
            _minPts= 18;
        } else if(_choice == _clearSky) {
            _radius = 1.5;
            _minPts = 1000;
        } else {
            _minPts = 40;
        }
    }
}

void SF_RadiusOutlierFilterStep::createParamList(CT_ResultGroup * outResult) {
    adaptParametersToExpertLevel();
    CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
    while(!isStopped() && outResIt.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) outResIt.next();
        const CT_AbstractItemDrawableWithPointCloud* ctCloud = (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
        SF_ParamRadiusOutlierFilter<SF_PointNormal> param;
        param._log = PS_LOG;
        param._radius = _radius;
        param._minPts = _minPts;
        param._sizeOutput = 2;
        param._itemCpyCloudIn = ctCloud;
        param._grpCpyGrp = group;
        _paramList.append(param);
    }
}
