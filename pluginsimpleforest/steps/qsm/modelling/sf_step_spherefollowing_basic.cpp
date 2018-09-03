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

#include "sf_step_spherefollowing_basic.h"
#include "sf_step_spherefollowing_basic_adapter.h"
#include <QtConcurrent/QtConcurrent>

SF_Step_Spherefollowing_Basic::SF_Step_Spherefollowing_Basic(CT_StepInitializeData &data_init): SF_Segmentation_Step(data_init) {
    _non_expert_level.append(_low);
    _non_expert_level.append(_medium);
    _non_expert_level.append(_high);
}

SF_Step_Spherefollowing_Basic::~SF_Step_Spherefollowing_Basic() {
}

QString SF_Step_Spherefollowing_Basic::getStepDescription() const {
    return tr("SphereFollowing Basic");
}

QString SF_Step_Spherefollowing_Basic::getStepDetailledDescription() const {
    return tr("This implementation of the SphereFollowing method utilizes an unsegmented tree cloud. Only one set of parameters will be optimized."
              "Results in a fast QSM estimation with less accuracy. From the QSM a segmentation then is performed on the tree cloud.");
}

QString SF_Step_Spherefollowing_Basic::getStepURL() const {
    return tr("");
}

CT_VirtualAbstractStep* SF_Step_Spherefollowing_Basic::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_Step_Spherefollowing_Basic(dataInit);
}

QStringList SF_Step_Spherefollowing_Basic::getStepRISCitations() const {
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

    _RIS_citation_list.append(QString("TY  - JOUR\n"
                                      "T1  - Highly Accurate Tree Models Derived from Terrestrial Laser Scan Data: A Method Description\n"
                                      "A1  - Hackenberg, Jan\n"
                                      "A1  - Sheppard, Jonathan\n"
                                      "A1  - Spiecker, Heinrich\n"
                                      "A1  - Disney, Mathias\n"
                                      "JO  - Forests\n"
                                      "VL  - 5\n"
                                      "IS  - 5\n"
                                      "SP  - 1069\n"
                                      "EP  - 1105\n"
                                      "Y1  - 2014\n"
                                      "PB  - Multidisciplinary Digital Publishing Institute\n"
                                      "UL  - http://www.mdpi.com/1999-4907/5/5/1069\n"
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


void SF_Step_Spherefollowing_Basic::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *res_model = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
    res_model->setZeroOrMoreRootGroup();
    res_model->addGroupModel("", DEF_IN_GRP_CLUSTER, CT_AbstractItemGroup::staticGetType(), tr("Tree Group"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    res_model->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, CT_Scene::staticGetType(), tr("Tree Cloud"));
}

void SF_Step_Spherefollowing_Basic::createPostConfigurationDialogExpert(CT_StepConfigurableDialog *config_dialog) {
    config_dialog->addDouble("The cloud is downscaled first to  ", " (m). " , 0.005,0.03,4,_voxelSize );
    config_dialog->addDouble("The algorithm is initialized on a close to ground slice with height ", " " , 0.05,0.3,2,_heightStartSphere );
    config_dialog->addInt("At minimum  ", " points are needed to fit a circle",3,20,_minPtsCircle );
    config_dialog->addDouble("The sphere surface has a thickness of  ", " " , 0.01,0.1,2, _sphereEpsilon );
    config_dialog->addDouble("Each sphere has at minimum a radius of   ", " " , 0.01,0.1,2, _minRadius );
    config_dialog->addDouble("To generate a sphere each fitted circle is multiplied with    ", " " , 1.4 ,4,2, _sphereRadiusMultiplier );
    config_dialog->addDouble("Point on sphere surface are clustered with threshold    ", " " , 0.02 ,0.1 , 2, _euclideanDistance );
}

void SF_Step_Spherefollowing_Basic::createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *config_dialog) {
    config_dialog->addStringChoice("Choose the quality of the point cloud","",_non_expert_level, _choice);
}

void SF_Step_Spherefollowing_Basic::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *res_modelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(res_modelw != NULL) {
        res_modelw->addItemModel(DEF_IN_SCENE, _out_cloud_cluster, new CT_Scene(), tr("Dijsktra Segmented"));
    }
}

void SF_Step_Spherefollowing_Basic::adapt_parameters_to_expert_level() {
    if(!_is_expert) {
        _euclideanDistance = 0.02;
        _sphereRadiusMultiplier = 2;
        _minRadius = 0.07;
        _sphereEpsilon = 0.035;
        _ransacCircleInlierDistance = 0.03;
        _minPtsCircle = 3;
        _heightStartSphere = 0.1;
        _voxelSize = 0.01;
        if(_choice == _low) {
            _euclideanDistance = 0.1;
            _minRadius = 2;
            _sphereEpsilon = 0.05;
        } else if(_choice == _medium) {
            _euclideanDistance = 0.04;
            _minRadius = 0.1;
            _sphereEpsilon = 0.5;
        } else {
            _euclideanDistance = 0.02;
            _minRadius = 0.07;
            _sphereEpsilon = 0.035;
        }
    }
}

void SF_Step_Spherefollowing_Basic::compute() {
    const QList<CT_ResultGroup*> &out_result_list = getOutResultList();
    CT_ResultGroup * out_result = out_result_list.at(0);
    identify_and_remove_corrupted_scenes(out_result);
    createParamList(out_result);
    write_logger();
    QFuture<void> future = QtConcurrent::map(_param_list,SF_Spherefollowing_Basic_Adapter() );
//    set_progress_by_future(future,10,85);
//    write_output(out_result);
}

void SF_Step_Spherefollowing_Basic::createParamList(CT_ResultGroup * out_result) {
    adapt_parameters_to_expert_level();
    CT_ResultGroupIterator out_res_it(out_result, this, DEF_IN_GRP_CLUSTER);
    while(!isStopped() && out_res_it.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) out_res_it.next();
        const CT_AbstractItemDrawableWithPointCloud* ct_cloud = (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
        SF_Param_Spherefollowing_Basic<SF_Point> param;
        param._log = PS_LOG;
        param._itemCpy_cloud_in = ct_cloud;
        param._grpCpy_grp = group;
        param._euclideanDistance = _euclideanDistance;
        param._sphereRadiusMultiplier = _sphereRadiusMultiplier;
        param._minRadius = _minRadius;
        param._sphereEpsilon = _sphereEpsilon;
        param._ransacCircleInlierDistance = _ransacCircleInlierDistance;
        param._minPtsCircle = _minPtsCircle;
        param._heightStartSphere = _heightStartSphere;
        _param_list.append(param);
    }
}
