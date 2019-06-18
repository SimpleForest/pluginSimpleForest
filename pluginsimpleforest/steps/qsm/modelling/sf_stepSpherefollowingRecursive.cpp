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

#include "sf_stepSpherefollowingRecursive.h"

#include "sf_stepSpherefollowingRecursiveAdapter.h"
#include "steps/item/sf_spherefollowing_parameters_item.h"

#include <QtConcurrent/QtConcurrent>

#include <ct_itemdrawable/ct_cylinder.h>

SF_StepSphereFollowingRecursive::SF_StepSphereFollowingRecursive(CT_StepInitializeData& dataInit)
  : SF_StepSpherefollowingBasic(dataInit)
{}

SF_StepSphereFollowingRecursive::~SF_StepSphereFollowingRecursive() {}

QString
SF_StepSphereFollowingRecursive::getStepDescription() const
{
  return tr("SphereFollowing Recursive");
}

QString
SF_StepSphereFollowingRecursive::getStepDetailledDescription() const
{
  return tr("First from an input cloud and an input qsm unfitted points are estimated."
            " Those points are clustered. Each cluster large enough gets processed with the"
            " Spherefollowing routine. This produces a sub qsm representing a before not fitted"
            " branch. This subqsm is attached to the input qsm.");
}

QString
SF_StepSphereFollowingRecursive::getStepURL() const
{
  return tr("http://simpleforest.org/");
}

CT_VirtualAbstractStep*
SF_StepSphereFollowingRecursive::createNewInstance(CT_StepInitializeData& dataInit)
{
  return new SF_StepSphereFollowingRecursive(dataInit);
}

QStringList
SF_StepSphereFollowingRecursive::getStepRISCitations() const
{
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  _risCitationList.append(getRISCitationSphereFollowing());
  _risCitationList.append(getRISCitationPCL());
  return _risCitationList;
}

void
SF_StepSphereFollowingRecursive::createPreConfigurationDialog()
{
  CT_StepConfigurableDialog* configDialog = newStandardPreConfigurationDialog();
  configDialog->addBool("Uncheck to deactivate parameterization possibilities "
                        "of this step. Only recommended for beginners",
                        "",
                        "expert",
                        _isExpert);
  configDialog->addStringChoice("In beginner mode select point cloud quality", "", _pointDensities, _choicePointDensity);
  configDialog->addBool(
    "You can let Simple Forest choose method parameters", " select if you want auto parameters", "", _SF_parameterAutoSearch);
  createPostConfigurationDialogCitation(configDialog);
  addCitationPCL(configDialog);
}

void
SF_StepSphereFollowingRecursive::configRecursion(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addText("<b>Recusion, see (Hackenberg et al <b>2014</b> - section 4.5. Imperfect Point Clouds):");
  configDialog->addDouble("All points of the input cloud with a [<em><b>point to model distance</b></em>] larger ",
                          " (m) are extracted.",
                          0.02,
                          0.9,
                          2,
                          m_unfittedDistance);
  configDialog->addDouble("Those unfitted points are clustered with a  "
                          "[<em><b>clustering range</b></em>]  ",
                          " (m). ",
                          0.03,
                          0.1,
                          2,
                          m_clusteringDistance);
  configDialog->addDouble("Each cluster larger than a "
                          "[<em><b>percentage</b></em>]  ",
                          " (%) of the input cloud is further processed. ",
                          0.001,
                          1.0,
                          3,
                          m_minPercentage);
  configDialog->addEmpty();
}

void
SF_StepSphereFollowingRecursive::createPostConfigurationDialog()
{
  CT_StepConfigurableDialog* configDialog = newStandardPostConfigurationDialog();
  addOutputFormat(configDialog);
  configRecursion(configDialog);
  configDialogGuruAddPreProcessing(configDialog);
  configDialogAddSphereFollowingOptimizableParameters(configDialog);
  configDialogGuruAddGridSearchCloudToModelDistance(configDialog);
  configDialogAddSphereFollowingHyperParameters(configDialog);
}

void
SF_StepSphereFollowingRecursive::createOutResultModelListProtected()
{
  CT_OutResultModelGroupToCopyPossibilities* resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT2);
  if (resModelw != NULL) {
    addQSMToOutResult(resModelw, QString("QSM SphereFollowing recursion"), QString::fromUtf8(DEF_IN_GRP_CLUSTER2));
    resModelw->addItemModel(_QSMGrp, _outSFQSM, new SF_QSM_Item(), tr("Internal QSM SphereFollowing"));
    resModelw->addItemModel(
      _QSMGrp, _outParams, new SF_SphereFollowing_Parameters_Item(), tr(" Internal parameters QSM SphereFollowing"));
  }
}

void
SF_StepSphereFollowingRecursive::createInResultModelListProtected()
{
  CT_InResultModelGroupToCopy* resModel2 = createNewInResultModelForCopy(DEF_IN_RESULT2, tr("Second group result"));
  resModel2->setZeroOrMoreRootGroup();
  resModel2->addGroupModel("",
                           DEF_IN_GRP_CLUSTER2,
                           CT_AbstractItemGroup::staticGetType(),
                           tr("QSM Group"),
                           "",
                           CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
  resModel2->addItemModel(DEF_IN_GRP_CLUSTER2, DEF_IN_QSM, SF_QSM_Item::staticGetType(), tr("internal QSM"));

  CT_InResultModelGroup* resModel = createNewInResultModel(DEF_IN_RESULT, tr("Input Result QSM SphereFollowing"));
  resModel->setZeroOrMoreRootGroup();
  resModel->addGroupModel("",
                          DEF_IN_GRP_CLUSTER,
                          CT_AbstractItemGroup::staticGetType(),
                          tr("Tree Group"),
                          "",
                          CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
  resModel->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, CT_Scene::staticGetType(), tr("Input Cloud QSM SphereFollowing"));
}

void
SF_StepSphereFollowingRecursive::createPostConfigurationDialogCitationSecond(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addText(QObject::tr("For this step please cite in addition (section 4.5. Imperfect Point Clouds):"),
                        "Hackenberg, J.; Morhart, C.; Sheppard, J.; Spiecker, H.; Disney, M.");
  configDialog->addText("",
                        "<em>Highly Accurate Tree Models Derived from Terrestrial Laser Scan "
                        "Data: A Method Description.</em>");
  configDialog->addText("", "Forests <b>2014</b>, 5, 1069-1105.");
  configDialog->addEmpty();
}

void
SF_StepSphereFollowingRecursive::compute()
{
  CT_ResultGroup* outResult = getInputResults().at(1);
  identifyAndRemoveCorruptedScenes(outResult);
  createParamList(outResult);
  writeLogger();
  QFuture<void> future = QtConcurrent::map(_paramList, SF_SpherefollowingRecursiveAdapter());
  while (!future.isFinished()) {
    setProgressByCounter(10.0f, 85.0f);
  }
  SF_AbstractStepQSM::addQSM<SF_ParamSpherefollowingRecursive<SF_PointNormal>>(
    getOutResultList().front(), _paramList, QString::fromUtf8(DEF_IN_GRP_CLUSTER2), _outSFQSM.completeName(), "");
  _paramList.clear();
}

QList<SF_ParamQSM<SF_PointNormal>>
SF_StepSphereFollowingRecursive::paramList()
{
  QList<SF_ParamQSM<SF_PointNormal>> paramList;
  std::for_each(_paramList.begin(), _paramList.end(), [&paramList](SF_ParamSpherefollowingRecursive<SF_PointNormal>& params) {
    SF_ParamQSM<SF_PointNormal> param;
    param._qsm = params._qsm;
    param._translation = params._translation;
    param._colors = params._colors;
    paramList.push_back(param);
  });
  return paramList;
}

void
SF_StepSphereFollowingRecursive::createParamList(CT_ResultGroup* outResult)
{
  SF_SphereFollowingParameters sphereFollowingParams;
  SF_SphereFollowingOptimizationParameters sfOptimizationParameters;
  sfOptimizationParameters._epsilonSphere = _SF_OPT_sphereEpsilon;
  sfOptimizationParameters._epsilonSphereMultiplier = paramsStringToNumber(_PARAMETERS_CHOICE_SPHERE_EPSILON);
  sfOptimizationParameters._euclideanClusteringDistance = _SF_OPT_euclideanClusteringDistance;
  sfOptimizationParameters._euclideanClusteringDistanceMultiplier = paramsStringToNumber(
    _PARAMETERS_CHOICE_EUCLIDEAN_CLUSTERING_DISTANCE);
  sfOptimizationParameters._sphereRadiusMultiplier = _SF_OPT_sphereRadiusMultiplier;
  sfOptimizationParameters._sphereRadiusMultiplierMultiplier = paramsStringToNumber(_PARAMETERS_CHOICE_SPHERE_RADIUS_MULTIPLIER);
  std::vector<SF_SphereFollowingOptimizationParameters> optimizationParametersVector;
  optimizationParametersVector.push_back(sfOptimizationParameters);
  sphereFollowingParams.m_optimizationParams = optimizationParametersVector;
  sphereFollowingParams._minPtsGeometry = _SF_minPtsGeometry;
  sphereFollowingParams._inlierDistance = _SF_inlierDistance;
  sphereFollowingParams._RANSACIterations = _SF_RANSACIiterations;
  sphereFollowingParams._heightInitializationSlice = _SF_heightInitializationSlice;
  sphereFollowingParams._minGlobalRadius = _SF_minRadiusGlobal;
  sphereFollowingParams._fittingMethod = toStringSFMethod();

  SF_CloudToModelDistanceParameters distanceParams;
  distanceParams._method = toStringCMDMethod();
  distanceParams._k = _CMD_k;
  distanceParams._cropDistance = _CMD_cropDistance;
  distanceParams._inlierDistance = _CMD_inlierDistance;

  adaptParametersToExpertLevel();
  CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
  while (!isStopped() && outResIt.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt.next();
    const CT_AbstractItemDrawableWithPointCloud* ctCloud = (const CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(
      this, DEF_IN_CLOUD_SEED);
    SF_ParamSpherefollowingRecursive<SF_PointNormal> param;
    param.m_clusteringDistance = m_clusteringDistance;
    param.m_minPercentage = m_minPercentage;
    param.m_unfittedDistance = m_unfittedDistance;
    param._stepProgress = _stepProgress;
    param._distanceParams = distanceParams;
    param._sphereFollowingParams = sphereFollowingParams;
    param._voxelSize = _PP_voxelSize;
    param._clusteringDistance = _PP_euclideanClusteringDistance;
    param.m_numClstrs = _CMD_numClstrs;
    param._modelCloudError = 1337;
    param._fittedGeometries = 0;
    param._log = PS_LOG;
    param._itemCpyCloudIn = ctCloud;
    param._grpCpyGrp = group;
    _paramList.append(param);
  }
  if (paramList().empty()) {
    std::cout << "SF_StepSphereFollowingRecursive No cloud received" << std::endl;
    return;
  }
  CT_ResultGroup* outResult2 = getInputResults().at(0);
  CT_ResultGroupIterator outResIt2(outResult2, this, DEF_IN_GRP_CLUSTER2);
  size_t index = 0;
  while (!isStopped() && outResIt2.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt2.next();
    if (index > static_cast<size_t>(_paramList.size())) {
      std::cout << "SF_StepSphereFollowingRecursive More trees than clouds" << std::endl;
      return;
    }
    index = std::min(index, static_cast<size_t>(_paramList.size()));
    SF_ParamSpherefollowingRecursive<SF_PointNormal>& param = _paramList[index];
    index++;
    const SF_QSM_Item* qmsItem = (const SF_QSM_Item*)group->firstItemByINModelName(this, DEF_IN_QSM);
    auto qsm = qmsItem->getQsm();
    param._qsm = qsm;
  }
  m_computationsTotal = _paramList.size();
}
