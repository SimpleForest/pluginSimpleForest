/****************************************************************************

 Copyright (C) 2017-2019 Dr. Jan Hackenberg, free software developer
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

#include "sf_stepSpherefollowingAdvanced.h"

#include "steps/item/sf_spherefollowing_parameters_item.h"
#include "steps/qsm/modelling/sf_stepSpherefollowingAdvancedAdapter.h"

#include <QtConcurrent/QtConcurrent>

#include <ct_itemdrawable/ct_cylinder.h>

SF_StepSphereFollowingAdvanced::SF_StepSphereFollowingAdvanced(CT_StepInitializeData& dataInit) : SF_AbstractStepQSM(dataInit) {}

SF_StepSphereFollowingAdvanced::~SF_StepSphereFollowingAdvanced() {}

QString
SF_StepSphereFollowingAdvanced::getStepDescription() const
{
  return tr("SphereFollowing Advanced");
}

QString
SF_StepSphereFollowingAdvanced::getStepDetailledDescription() const
{
  return tr("This implementation of the SphereFollowing method utilizes an "
            "segmented tree cloud and preestimated parameters."
            "  We initialize our routine with the cluster containing the root of the tree."
            " On that cluster we perform a Nelder and Mead parameter search. Then we"
            " extend the cluster by adding its closest neighboring cluster, e.g. most likely"
            " at this stage we will add major branching structure to the previously optimized stem."
            " A new parameter search with the extended cloud is performed and we proceed after each fit"
            " to add cluster by cluster until the whole cloud is processed.");
}

QString
SF_StepSphereFollowingAdvanced::getStepURL() const
{
  return tr("");
}

CT_VirtualAbstractStep*
SF_StepSphereFollowingAdvanced::createNewInstance(CT_StepInitializeData& dataInit)
{
  return new SF_StepSphereFollowingAdvanced(dataInit);
}

QStringList
SF_StepSphereFollowingAdvanced::getStepRISCitations() const
{
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  _risCitationList.append(getRISCitationSphereFollowing());
  _risCitationList.append(getRISCitationPCL());
  return _risCitationList;
}

void
SF_StepSphereFollowingAdvanced::configDialogAddSphereFollowingNelderMead(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addText("<b>Downhill Simplex</b>:");
  configDialog->addDouble("The downhill simplex search reaches it convergence criteria"
                          " with[<em><b>min size</b></em>] ",
                          " 1e-3(m). ",
                          0.001,
                          0.999,
                          3,
                          _NM_minSize);
  configDialog->addInt("We set for the simplex search a maximal"
                       "[<em><b> number of iterations</b></em>]  ",
                       " .",
                       1,
                       1000,
                       _NM_iterations);
}

void
SF_StepSphereFollowingAdvanced::configDialogGuruAddPreProcessing(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addText("<b>Pre Processing and Spherefollowing</b>:");
  configDialog->addText(
    "Spherefollowing parameters as well as preprocessing parameters are imported from basic spherefollowing step.");
  configDialog->addEmpty();
}

void
SF_StepSphereFollowingAdvanced::configDialogGuruAddGridSearchCloudToModelDistance(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addText("<b>Cloud To Model Distance</b>:");
  configDialog->addText("For the grid search evaluation the parameter set with "
                        "the smallest qsm to cloud distance is chosen.");
  configDialog->addStringChoice("For the cloud to model distance we choose "
                                "[<em><b>distance method</b></em>] ",
                                " ",
                                _CMD_methodList,
                                _CMD_methodChoice);
  configDialog->addInt("Test for each point its nearest [<em><b>k</b></em>] "
                       "cylinders to get best nearest neighbor",
                       "",
                       3,
                       9,
                       _CMD_k);
  configDialog->addDouble("For MSAC and inlier methods the distance is cropped "
                          "at [<em><b>crop distance</b></em>] ",
                          " (m).",
                          0.05,
                          0.5,
                          2,
                          _CMD_cropDistance);
  configDialog->addDouble("The "
                          "at [<em><b>inlier distance</b></em>] ",
                          " (m).",
                          0.01,
                          0.1,
                          2,
                          _CMD_inlierDistance);
  configDialog->addEmpty();
}

void
SF_StepSphereFollowingAdvanced::createPreConfigurationDialog()
{
  CT_StepConfigurableDialog* configDialog = newStandardPreConfigurationDialog();
  configDialogGuruAddPreProcessing(configDialog);
  createPostConfigurationDialogCitation(configDialog);
  addCitationPCL(configDialog);
}

void
SF_StepSphereFollowingAdvanced::createPostConfigurationDialog()
{
  CT_StepConfigurableDialog* configDialog = newStandardPostConfigurationDialog();
  configDialogGuruAddGridSearchCloudToModelDistance(configDialog);
  configDialogAddSphereFollowingNelderMead(configDialog);
}

void
SF_StepSphereFollowingAdvanced::createInResultModelListProtected()
{
  CT_InResultModelGroupToCopy* resModel = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
  resModel->setZeroOrMoreRootGroup();
  resModel->addGroupModel("",
                          DEF_IN_GRP_CLUSTER,
                          CT_AbstractItemGroup::staticGetType(),
                          tr("Tree Group"),
                          "",
                          CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
  resModel->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, CT_Scene::staticGetType(), tr("Tree Cloud"));
  resModel->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_ID, CT_PointsAttributesScalarTemplated<int>::staticGetType(), tr("Cluster ID"));
  resModel->addItemModel(
    DEF_IN_GRP_CLUSTER, DEF_IN_PARAMS, SF_SphereFollowing_Parameters_Item::staticGetType(), tr("SphereFollowing parameters"));
}

void
SF_StepSphereFollowingAdvanced::createOutResultModelListProtected()
{
  CT_OutResultModelGroupToCopyPossibilities* resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
  if (resModelw != NULL) {
    resModelw->addItemModel(DEF_IN_GRP_CLUSTER, m_outCloudItem, new CT_PointsAttributesColor(), tr("Spherefollowing Fit Quality"));
    addQSMToOutResult(resModelw, QString("QSM SphereFollowing clustered"), QString::fromUtf8(DEF_IN_GRP_CLUSTER));
    resModelw->addItemModel(DEF_IN_GRP_CLUSTER, _outSFQSM, new SF_QSM_Item(), tr("QSM cylinders clustered spherefollowing"));
    resModelw->addItemModel(
      DEF_IN_GRP_CLUSTER, _outParams, new SF_SphereFollowing_Parameters_Item(), tr("SphereFollowing parameters"));
  }
}

void
SF_StepSphereFollowingAdvanced::adaptParametersToExpertLevel()
{}

void
SF_StepSphereFollowingAdvanced::createPostConfigurationDialogCitationSecond(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addText(QObject::tr("For this step please cite in addition:"),
                        "Hackenberg, J.; Morhart, C.; Sheppard, J.; Spiecker, H.; Disney, M.");
  configDialog->addText("",
                        "<em>Highly Accurate Tree Models Derived from Terrestrial Laser Scan "
                        "Data: A Method Description.</em>");
  configDialog->addText("", "Forests <b>2014</b>, 5, 1069-1105.");
  configDialog->addEmpty();
}

void
SF_StepSphereFollowingAdvanced::compute()
{
  const QList<CT_ResultGroup*>& out_result_list = getOutResultList();
  CT_ResultGroup* outResult = out_result_list.at(0);
  identifyAndRemoveCorruptedScenes(outResult);
  createParamList(outResult);
  writeLogger();
  QFuture<void> future = QtConcurrent::map(_paramList, SF_SpherefollowingAdvancedAdapter());
  while (!future.isFinished()) {
    setProgressByCounter(10.0f, 85.0f);
  }
  addColors(outResult, paramList(), DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, m_outCloudItem.completeName());

  SF_AbstractStepQSM::addQSM<SF_ParamSpherefollowingAdvanced<SF_PointNormal>>(
    outResult, _paramList, QString::fromUtf8(DEF_IN_GRP_CLUSTER), _outSFQSM.completeName(), _outParams.completeName());
  _paramList.clear();
}

int
SF_StepSphereFollowingAdvanced::getNumberOfClusters(CT_PointsAttributesScalarTemplated<int>* ctID)
{
  int numberClusters = 0;
  for (size_t i = 0; i < ctID->attributesSize(); i++) {
    int ID = ctID->valueAt(i);
    if (numberClusters < ID + 1) {
      numberClusters = ID + 1;
    }
  }
  return numberClusters;
}

QList<SF_ParamQSM<SF_PointNormal>>
SF_StepSphereFollowingAdvanced::paramList()
{
  QList<SF_ParamQSM<SF_PointNormal>> paramList;
  std::for_each(_paramList.begin(), _paramList.end(), [&paramList](SF_ParamSpherefollowingBasic<SF_PointNormal>& params) {
    SF_ParamQSM<SF_PointNormal> param;
    param._qsm = params._qsm;
    param._translation = params._translation;
    param._colors = params._colors;
    paramList.push_back(param);
  });
  return paramList;
}

int
SF_StepSphereFollowingAdvanced::toStringSFMethod()
{
  int type = -1;
  if (_SF_methodChoice == _RANSAC)
    type = pcl::SAC_RANSAC;
  if (_SF_methodChoice == _LMEDS)
    type = pcl::SAC_LMEDS;
  if (_SF_methodChoice == _MSAC)
    type = pcl::SAC_MSAC;
  if (_SF_methodChoice == _RRANSAC)
    type = pcl::SAC_RRANSAC;
  if (_SF_methodChoice == _RMSAC)
    type = pcl::SAC_RMSAC;
  if (_SF_methodChoice == _MLESAC)
    type = pcl::SAC_MLESAC;
  if (_SF_methodChoice == _PROSAC)
    type = pcl::SAC_PROSAC;
  return type;
}

SF_CLoudToModelDistanceMethod
SF_StepSphereFollowingAdvanced::toStringCMDMethod()
{
  if (_CMD_methodChoice == _ZEROMOMENTUMORDER)
    return SF_CLoudToModelDistanceMethod::ZEROMOMENTUMORDER;
  if (_CMD_methodChoice == _FIRSTMOMENTUMORDERMSAC)
    return SF_CLoudToModelDistanceMethod::FIRSTMOMENTUMORDERMSAC;
  if (_CMD_methodChoice == _FIRSTMOMENTUMORDER)
    return SF_CLoudToModelDistanceMethod::FIRSTMOMENTUMORDER;
  if (_CMD_methodChoice == _SECONDMOMENTUMORDERMSAC)
    return SF_CLoudToModelDistanceMethod::SECONDMOMENTUMORDERMSAC;
  if (_CMD_methodChoice == _SECONDMOMENTUMORDER)
    return SF_CLoudToModelDistanceMethod::SECONDMOMENTUMORDER;
  return SF_CLoudToModelDistanceMethod::ZEROMOMENTUMORDER;
}

std::vector<double>
SF_StepSphereFollowingAdvanced::paramsStringToNumber(const QString& UISelection)
{
  std::vector<double> paramVec;
  return paramVec;
}

void
SF_StepSphereFollowingAdvanced::createParamList(CT_ResultGroup* outResult)
{
  SF_CloudToModelDistanceParameters distanceParams;
  distanceParams._method = toStringCMDMethod();
  distanceParams._k = _CMD_k;
  distanceParams._cropDistance = _CMD_cropDistance;
  distanceParams._inlierDistance = _CMD_inlierDistance;

  CT_ResultGroupIterator outResItCloud(outResult, this, DEF_IN_GRP_CLUSTER);
  int numberComputationsPerCloud = 300;
  while (!isStopped() && outResItCloud.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResItCloud.next();
    const CT_AbstractItemDrawableWithPointCloud* ctCloud = (const CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(
      this, DEF_IN_CLOUD_SEED);
    CT_PointsAttributesScalarTemplated<int>* ctID = (CT_PointsAttributesScalarTemplated<int>*)group->firstItemByINModelName(this,
                                                                                                                            DEF_IN_ID);
    const SF_SphereFollowing_Parameters_Item* ctParameters = (const SF_SphereFollowing_Parameters_Item*)group->firstItemByINModelName(
      this, DEF_IN_PARAMS);
    SF_ParamSpherefollowingBasic<SF_PointNormal> param = ctParameters->getParams();
    SF_ParamSpherefollowingAdvanced<SF_PointNormal> paramAdvanced;
    paramAdvanced._sphereFollowingParams = param._sphereFollowingParams;
    paramAdvanced.m_numClstrs = getNumberOfClusters(ctID);
    paramAdvanced._sphereFollowingParams.m_optimizationParams.clear();
    for (size_t m = 0; m < paramAdvanced.m_numClstrs; m++) {
      paramAdvanced._sphereFollowingParams.m_optimizationParams.push_back(param._sphereFollowingParams.m_optimizationParams[0]);
    }
    paramAdvanced._stepProgress = _stepProgress;
    paramAdvanced._distanceParams = distanceParams;
    paramAdvanced._voxelSize = param._voxelSize;
    paramAdvanced._clusteringDistance = param._clusteringDistance;
    paramAdvanced._modelCloudError = std::numeric_limits<float>::max();
    paramAdvanced._fittedGeometries = 0;
    paramAdvanced._log = PS_LOG;
    paramAdvanced._itemCpyCloudIn = ctCloud;
    paramAdvanced._ctID = ctID;
    paramAdvanced._grpCpyGrp = group;
    paramAdvanced._iterations = _NM_iterations;
    numberComputationsPerCloud = (_NM_iterations + 27) * (paramAdvanced.m_numClstrs + 1);
    paramAdvanced._fitQuality = _NM_minSize;
    _paramList.append(paramAdvanced);
  }
  int numberClouds = _paramList.size();
  m_computationsTotal = numberClouds * numberComputationsPerCloud;
}
