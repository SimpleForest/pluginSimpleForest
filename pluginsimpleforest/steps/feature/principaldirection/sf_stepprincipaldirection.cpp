/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_stepprincipaldirection.cpp is part of SimpleForest - a plugin for the
    Computree platform.

    SimpleForest is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SimpleForest is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with
    SimpleForest. If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/

#include "sf_stepprincipaldirection.h"

#include "ct_colorcloud/ct_colorcloudstdvector.h"
#include "ct_itemdrawable/ct_pointsattributescolor.h"
#include "ct_itemdrawable/ct_scene.h"
#include "ct_result/ct_resultgroup.h"
#include "ct_result/model/inModel/ct_inresultmodelgroup.h"
#include "ct_result/model/outModel/ct_outresultmodelgroup.h"

#include <QtConcurrent/QtConcurrent>

#define DEF_outColorResult "outColorResult";
#define DEF_outColorGrp    "outColorGrp";
#define DEF_outColorItem   "outColorItem";

SF_StepPrincipalDirection::SF_StepPrincipalDirection(CT_StepInitializeData &dataInit):
    SF_AbstractStepFeature(dataInit)
{
}

SF_StepPrincipalDirection::~SF_StepPrincipalDirection()
{
}

QString SF_StepPrincipalDirection::getStepDescription() const
{
    return tr("SphereFollowing Basic");
}

QString SF_StepPrincipalDirection::getStepDetailledDescription() const
{
    return tr("This implementation of the SphereFollowing method utilizes an unsegmented tree cloud. Only one set of parameters will be optimized."
              "Results in a fast QSM estimation with less accuracy.");
}

QString SF_StepPrincipalDirection::getStepURL() const
{
    return tr("");
}

CT_VirtualAbstractStep* SF_StepPrincipalDirection::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new SF_AbstractStepFeature(dataInit);
}

QStringList SF_StepPrincipalDirection::getStepRISCitations() const
{
    QStringList _risCitationList;
    _risCitationList.append(getRISCitationSimpleTree());
    _risCitationList.append(getRISCitationPCL());
    _risCitationList.append(getRISCitationRaumonen());
    return _risCitationList;
}


void SF_StepPrincipalDirection::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *resModel = createNewInResultModelForCopy(DEF_IN_RESULT,
                                                                           tr("Point Cloud"),
                                                                           "",
                                                                           false);
    resModel->setZeroOrMoreRootGroup();
    resModel->addGroupModel("",
                             DEF_IN_GRP_CLUSTER,
                             CT_AbstractItemGroup::staticGetType(),
                             tr("Point Group"),
                             "",
                             CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resModel->addItemModel(DEF_IN_GRP_CLUSTER,
                            DEF_IN_CLOUD_SEED,
                            CT_Scene::staticGetType(),
                            tr("Point Cloud"));
}

void SF_StepPrincipalDirection::createOutResultModelListProtected()
{
    CT_OutResultModelGroup *resOutModel = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(resOutModel != NULL)
    {
        resOutModel->addItemModel(DEF_IN_GRP_CLUSTER,
                                  m_outCloudItem,
                                  CT_PointsAttributesColor(),
                                  tr("Principal Component"));
    }
}

void SF_StepPrincipalDirection::createPostConfigurationDialog(CT_StepConfigurableDialog *configDialog) {
    configDialog->addText("First to enable multithreading, the point cloud is clustered with voxelization.");
    configDialog->addInt("A voxelization with [<em><b>voxel size</b></em>]:",
                            " (m) sized voxels divides the cloud in n clusters.",
                            1,
                            20,
                            &m_param.m_parameterVoxelization.m_voxelSize,
                            "");
    configDialog->addText("This step computes the <b>principal direction</b>, e.g. the second normal derivate, for a point cloud.");
    configDialog->addDouble("The [<em><b>normal radius</b></em>]:",
                            " (m)",
                            0.01,
                            0.5,
                            2,
                            &m_param.m_normalRadius,
                            1,
                            "Used for the normal computation.");
    configDialog->addDouble("The [<em><b>principal direction radius</b></em>]:",
                            " (m)",
                            0.01,
                            0.5,
                            2,
                            &m_param.m_pdRadius,
                            1,
                            "Used for the principal curvature computation.");
}

void SF_StepPrincipalDirection::createPostConfigurationDialogCitationSecond(CT_StepConfigurableDialog *configDialog) {
    configDialog->addText(QObject::tr("For this step please cite in addition:"),
                          "Pasi Raumonen, Mikko Kaasalainen, Markku Åkerblom, Sanna Kaasalainen, Harri Kaartinen, Mikko Vastaranta, Markus Holopainen, Mathias Disney and Philip Lewis");
    configDialog->addText("",
                          "<em>Fast Automatic Precision Tree Models from Terrestrial Laser Scanner Data.</em>");
    configDialog->addText("",
                          "Remote Sensing<b>2013</b>, 5, 491-520.");
}

void SF_StepPrincipalDirection::compute() {
    const QList<CT_ResultGroup*> &out_result_list = getOutResultList();
    CT_ResultGroup * outResult = out_result_list.at(0);
    identifyAndRemoveCorruptedScenes(outResult);
    createParamList(outResult);
    writeLogger();
    QFuture<void> future = QtConcurrent::map(_paramList, SF_SpherefollowingRootAdapter());
    setProgressByFuture(future,
                        10,
                        85);
}

void SF_StepPrincipalDirection::createParamList(CT_ResultGroup * outResult)
{
    adaptParametersToExpertLevel();
    CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
    while(!isStopped() && outResIt.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) outResIt.next();
        const CT_AbstractItemDrawableWithPointCloud*  ctCloud =
       (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
        SF_ParamSpherefollowingBasic<SF_PointNormal> param;
        m_param.

        param._distanceParams = distanceParams;
        param._sphereFollowingParams = sphereFollowingParams;
        param._voxelSize = _PP_voxelSize;
        param._clusteringDistance = _PP_euclideanClusteringDistance;
        param._maxError  = -1337;
        param._minError  = 1337;
        param._fittedGeometries = 0;
        param._log = PS_LOG;
        param._itemCpyCloudIn = ctCloud;
        param._grpCpyGrp = group;
        _paramList.append(param);
    }
}
