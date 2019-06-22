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

#include "sf_stepDijkstraLightRecursive.h"

#include "sf_stepDijkstraLightRecursiveAdapter.h"
#include "steps/item/sf_spherefollowing_parameters_item.h"

#include <QtConcurrent/QtConcurrent>

#include <ct_itemdrawable/ct_cylinder.h>

SF_StepDijkstraLightRecursive::SF_StepDijkstraLightRecursive(CT_StepInitializeData& dataInit) : SF_StepSpherefollowingBasic(dataInit)
{}

SF_StepDijkstraLightRecursive::~SF_StepDijkstraLightRecursive() {}

QString
SF_StepDijkstraLightRecursive::getStepDescription() const
{
  return tr("SphereFollowing Dijkstra light Recursive");
}

QString
SF_StepDijkstraLightRecursive::getStepDetailledDescription() const
{
  return tr("First from an input cloud and an input qsm unfitted points are estimated."
            " Those points are clustered. Each cluster large enough gets processed with the"
            " dijkstra algorthim routine after being downscaled. The Dijkstra pathes are"
            " used as tree skeleton with an hardcoded diameter.");
}

QString
SF_StepDijkstraLightRecursive::getStepURL() const
{
  return tr("http://simpleforest.org/");
}

CT_VirtualAbstractStep*
SF_StepDijkstraLightRecursive::createNewInstance(CT_StepInitializeData& dataInit)
{
  return new SF_StepDijkstraLightRecursive(dataInit);
}

QStringList
SF_StepDijkstraLightRecursive::getStepRISCitations() const
{
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  _risCitationList.append(getRISCitationSphereFollowing());
  _risCitationList.append(getRISCitationPCL());
  return _risCitationList;
}

void
SF_StepDijkstraLightRecursive::createPreConfigurationDialog()
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
}

void
SF_StepDijkstraLightRecursive::configRecursion(CT_StepConfigurableDialog* configDialog)
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
                          "[<em><b>min percentage</b></em>]  ",
                          " (%) of the input cloud is further processed. ",
                          0.001,
                          1.0,
                          3,
                          m_minPercentage);
  configDialog->addDouble("Each cluster smaller than a "
                          "[<em><b>max percentage</b></em>]  ",
                          " (%) of the input cloud is further processed. ",
                          0.001,
                          1.0,
                          3,
                          m_maxPercentage);
  configDialog->addDouble("Each cluster is then downscaled with a "
                          "[<em><b>cell width</b></em>]  ",
                          " (m). ",
                          0.01,
                          0.2,
                          2,
                          m_clusterDownScale);
  configDialog->addEmpty();
}

void
SF_StepDijkstraLightRecursive::createPostConfigurationDialog()
{
  CT_StepConfigurableDialog* configDialog = newStandardPostConfigurationDialog();
  addOutputFormat(configDialog);
  configRecursion(configDialog);
  configDialogGuruAddPreProcessing(configDialog);
}

void
SF_StepDijkstraLightRecursive::createOutResultModelListProtected()
{
  CT_OutResultModelGroupToCopyPossibilities* resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT2);
  if (resModelw != NULL) {
    QString name = tr("QSM dijkstra light recursion ");
    QString sfCylinders = name;
    sfCylinders.append(tr("SF QSM plugin internal"));
    addQSMToOutResult(resModelw, name, QString::fromUtf8(DEF_IN_GRP_CLUSTER2));
    resModelw->addItemModel(_QSMGrp, _outSFQSM, new SF_QSM_Item(), sfCylinders);
  }
}

void
SF_StepDijkstraLightRecursive::createInResultModelListProtected()
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

  CT_InResultModelGroup* resModel = createNewInResultModel(DEF_IN_RESULT, tr("Input Result QSM"));
  resModel->setZeroOrMoreRootGroup();
  resModel->addGroupModel("",
                          DEF_IN_GRP_CLUSTER,
                          CT_AbstractItemGroup::staticGetType(),
                          tr("Tree Group"),
                          "",
                          CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
  resModel->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, CT_Scene::staticGetType(), tr("Input Cloud QSM"));
}

void
SF_StepDijkstraLightRecursive::createPostConfigurationDialogCitationSecond(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addEmpty();
  configDialog->addText(
    QObject::tr("For this step please cite in addition the early method using Dijkstra for predicting tree skeleton:"),
    "Xu, H.; Gossett, N.; Chen B.");
  configDialog->addText("(section 5.1 Tree Allometry)", "<em>Knowledge and heuristic-based modeling of laser-scanned trees.</em>");
  configDialog->addText("", "ACM Transactions on Graphics <b>2007</b>, 19-es.");
  configDialog->addEmpty();
  configDialog->addEmpty();
}

void
SF_StepDijkstraLightRecursive::compute()
{
  CT_ResultGroup* outResult = getInputResults().at(1);
  createParamList(outResult);
  writeLogger();
  QFuture<void> future = QtConcurrent::map(_paramList, SF_SpherefollowingDijkstraLightRecursiveAdapter());
  while (!future.isFinished()) {
    setProgressByCounter(10.0f, 85.0f);
  }
  SF_AbstractStepQSM::addQSM<SF_ParamSpherefollowingRecursive<SF_PointNormal>>(
    getOutResultList().front(), _paramList, QString::fromUtf8(DEF_IN_GRP_CLUSTER2), _outSFQSM.completeName(), "");
  _paramList.clear();
}

void
SF_StepDijkstraLightRecursive::createParamList(CT_ResultGroup* outResult)
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
    param.m_maxPercentage = m_maxPercentage;
    param.m_clusterDownScale = m_clusterDownScale;
    param.m_clusteringDistance = m_clusteringDistance;
    param.m_minPercentage = m_minPercentage;
    param.m_unfittedDistance = m_unfittedDistance;
    param._stepProgress = _stepProgress;
    param._distanceParams = distanceParams;
    param._sphereFollowingParams = sphereFollowingParams;
    param._voxelSize = m_clusteringDistance;
    param._clusteringDistance = _PP_euclideanClusteringDistance;
    param.m_numClstrs = _CMD_numClstrs;
    param._modelCloudError = 1337;
    param._fittedGeometries = 0;
    param._log = PS_LOG;
    param._itemCpyCloudIn = ctCloud;
    param._grpCpyGrp = group;
    _paramList.append(param);
  }
  CT_ResultGroup* outResult2 = getInputResults().at(0);
  CT_ResultGroupIterator outResIt2(outResult2, this, DEF_IN_GRP_CLUSTER2);
  size_t index = 0;
  while (!isStopped() && outResIt2.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt2.next();
    if (index > static_cast<size_t>(_paramList.size())) {
      std::cout << "SF_StepDijkstraLightRecursive More trees than clouds" << std::endl;
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
