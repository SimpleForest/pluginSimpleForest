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

#include "sf_stepSpherefollowingBasic.h"

#include "sf_stepSpherefollowingBasicAdapter.h"
#include "steps/item/sf_spherefollowing_parameters_item.h"

#include <QtConcurrent/QtConcurrent>

#include <ct_itemdrawable/ct_cylinder.h>

SF_StepSpherefollowingBasic::SF_StepSpherefollowingBasic(CT_StepInitializeData& dataInit) : SF_AbstractStepQSM(dataInit)
{
  _pointDensities.append(_lowDensity);
  _pointDensities.append(_mediumDensity);
  _pointDensities.append(_highDensity);

  _PARAMETERS_LIST_SPHERE_RADIUS_MULTIPLIER.push_back(_PARAMETERS_1);
  _PARAMETERS_LIST_SPHERE_RADIUS_MULTIPLIER.push_back(_PARAMETERS_3);
  _PARAMETERS_LIST_SPHERE_RADIUS_MULTIPLIER.push_back(_PARAMETERS_5);
  _PARAMETERS_LIST_SPHERE_RADIUS_MULTIPLIER.push_back(_PARAMETERS_7);
  _PARAMETERS_LIST_SPHERE_RADIUS_MULTIPLIER.push_back(_PARAMETERS_9);
  _PARAMETERS_LIST_SPHERE_RADIUS_MULTIPLIER.push_back(_PARAMETERS_11);

  _PARAMETERS_LIST_SPHERE_EPSILON.push_back(_PARAMETERS_1);
  _PARAMETERS_LIST_SPHERE_EPSILON.push_back(_PARAMETERS_3);
  _PARAMETERS_LIST_SPHERE_EPSILON.push_back(_PARAMETERS_5);
  _PARAMETERS_LIST_SPHERE_EPSILON.push_back(_PARAMETERS_7);
  _PARAMETERS_LIST_SPHERE_EPSILON.push_back(_PARAMETERS_9);
  _PARAMETERS_LIST_SPHERE_EPSILON.push_back(_PARAMETERS_11);

  _PARAMETERS_LIST_EUCLIDEAN_CLUSTERING_DISTANCE.push_back(_PARAMETERS_1);
  _PARAMETERS_LIST_EUCLIDEAN_CLUSTERING_DISTANCE.push_back(_PARAMETERS_3);
  _PARAMETERS_LIST_EUCLIDEAN_CLUSTERING_DISTANCE.push_back(_PARAMETERS_5);
  _PARAMETERS_LIST_EUCLIDEAN_CLUSTERING_DISTANCE.push_back(_PARAMETERS_7);
  _PARAMETERS_LIST_EUCLIDEAN_CLUSTERING_DISTANCE.push_back(_PARAMETERS_9);
  _PARAMETERS_LIST_EUCLIDEAN_CLUSTERING_DISTANCE.push_back(_PARAMETERS_11);
}

SF_StepSpherefollowingBasic::~SF_StepSpherefollowingBasic() {}

QString
SF_StepSpherefollowingBasic::getStepDescription() const
{
  return tr("SphereFollowing Basic");
}

QString
SF_StepSpherefollowingBasic::getStepDetailledDescription() const
{
  return tr("This implementation of the SphereFollowing method utilizes an "
            "unsegmented tree cloud. On the sphere following parameters a raster search if performed.");
}

QString
SF_StepSpherefollowingBasic::getStepURL() const
{
  return tr("http://simpleforest.org/");
}

CT_VirtualAbstractStep*
SF_StepSpherefollowingBasic::createNewInstance(CT_StepInitializeData& dataInit)
{
  return new SF_StepSpherefollowingBasic(dataInit);
}

QStringList
SF_StepSpherefollowingBasic::getStepRISCitations() const
{
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  _risCitationList.append(getRISCitationSphereFollowing());
  _risCitationList.append(getRISCitationPCL());
  return _risCitationList;
}

void
SF_StepSpherefollowingBasic::configDialogAddSphereFollowingHyperParameters(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addText("<b>SphereFollowing Method Hyper Parameters</b>:");
  configDialog->addText("First Hyper Parameters are set. Those parameters will "
                        "never change during optimization.");
  configDialog->addStringChoice("[<em><b>SAC Model Consensus method</b></em>] ", " ", _SF_methodList, _SF_methodChoice);
  configDialog->addDouble("The [<em><b>inlier distance</b></em>] for the"
                          " selected <em>SAC Model Consensus method</em> is ",
                          " (m). ",
                          0.005,
                          0.05,
                          3,
                          _SF_inlierDistance);
  configDialog->addInt("For fitting a geometry with the selected <em>SAC Model Consensus "
                       "method</em> at minimum [<em><b>minPts</b></em>]",
                       " points are needed.",
                       3,
                       100,
                       _SF_minPtsGeometry);
  configDialog->addInt("For the selected <em>SAC Model Consensus method</em> "
                       "[<em><b>iterations</b></em>]",
                       " are performed.",
                       3,
                       100,
                       _SF_RANSACIiterations);
  configDialog->addDouble("The radius of the search sphere during "
                          "sphereFollowing is never smaller than"
                          " [<em><b>min global radius</b></em>] ",
                          " (m). ",
                          0.01,
                          0.2,
                          3,
                          _SF_minRadiusGlobal);
  configDialog->addDouble("The algorithm is initialized on a close to ground slice with height "
                          "[<em><b>initialization height</b></em>]",
                          " (m).",
                          0.1,
                          0.5,
                          2,
                          _SF_heightInitializationSlice);
  configDialog->addEmpty();
}

void
SF_StepSpherefollowingBasic::configDialogAddSphereFollowingOptimizableParameters(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addText("<b>SphereFollowing Method Optimizationable Parameters</b>:");
  configDialog->addText("You can pre select also optimizable sphere following parameters:");
  configDialog->addDouble("To generate a sphere each fitted circle is multiplied with the "
                          "[<em><b>sphere multiplier</b></em>]",
                          ".",
                          1.4,
                          4,
                          2,
                          _SF_OPT_sphereRadiusMultiplier);
  configDialog->addStringChoice("We search for <em>sphere multiplier</em> with the following potential parameterization:",
                                ".",
                                _PARAMETERS_LIST_SPHERE_RADIUS_MULTIPLIER,
                                _PARAMETERS_CHOICE_SPHERE_RADIUS_MULTIPLIER);
  configDialog->addDouble(
    "The sphere surface has a thickness of [<em><b>sphere epsilon</b></em>] ", " (m).", 0.01, 0.1, 2, _SF_OPT_sphereEpsilon);
  configDialog->addStringChoice("We search for <em>sphere epsilon</em> with the following potential parameterization:",
                                ".",
                                _PARAMETERS_LIST_SPHERE_EPSILON,
                                _PARAMETERS_CHOICE_SPHERE_EPSILON);
  configDialog->addDouble("Point on sphere surface are clustered with threshold  [<em><b>euclidean "
                          "clustering distance</b></em>]  ",
                          " (m) to build new circle clusters.",
                          0.02,
                          0.1,
                          2,
                          _SF_OPT_euclideanClusteringDistance);
  configDialog->addStringChoice("We search for <em>euclidean clustering distance</em> with the following potential parameterization:",
                                ".",
                                _PARAMETERS_LIST_EUCLIDEAN_CLUSTERING_DISTANCE,
                                _PARAMETERS_CHOICE_EUCLIDEAN_CLUSTERING_DISTANCE);
  configDialog->addEmpty();
}

void
SF_StepSpherefollowingBasic::configDialogGuruAddPreProcessing(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addText("<b>Pre Processing</b>:");
  configDialog->addDouble("To even out the distribution and speed things up the cloud is "
                          "downscaled first to [<em><b>voxel size</b></em>] ",
                          " (m). ",
                          0.005,
                          0.03,
                          3,
                          _PP_voxelSize);
  configDialog->addDouble("Only the largest cluster will be processed with "
                          "[<em><b>clustering range</b></em>]  ",
                          " (m). ",
                          0.03,
                          0.9,
                          2,
                          _PP_euclideanClusteringDistance);
  configDialog->addEmpty();
}

void
SF_StepSpherefollowingBasic::configDialogGuruAddGridSearchCloudToModelDistance(CT_StepConfigurableDialog* configDialog)
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
SF_StepSpherefollowingBasic::createPreConfigurationDialog()
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
  configDialog->addText(QObject::tr("For this step please cite in addition the paper presenting the spherefollowing routine:"),
                        "Hackenberg, J.; Morhart, C.; Sheppard, J.; Spiecker, H.; Disney, M.");
  configDialog->addText("(section 4.3. Cylinder Model Creation)",
                        "<em>Highly Accurate Tree Models Derived from Terrestrial Laser Scan "
                        "Data: A Method Description.</em>");
  configDialog->addText("", "Forests <b>2014</b>, 5, 1069-1105.");
  configDialog->addText(QObject::tr("And this inventing the automatic parameter search for QSM modeling:"),
                        "Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P.");
  configDialog->addText("(section 2.2. Tree Modeling - Parameter Optimization)",
                        "<em>SimpleTree - An Efficient Open Source Tool to "
                        "Build Tree Models from TLS Clouds.</em>");
  configDialog->addText("", "Forests <b>2015</b>, 6, 4245-4294.");
  configDialog->addText("", "Forests <b>2014</b>, 5, 1069-1105.");
  createPostConfigurationDialogCitation(configDialog);
}

void
SF_StepSpherefollowingBasic::createPostConfigurationDialog()
{
  CT_StepConfigurableDialog* configDialog = newStandardPostConfigurationDialog();
  configDialogGuruAddPreProcessing(configDialog);
  configDialogAddSphereFollowingOptimizableParameters(configDialog);
  configDialogGuruAddGridSearchCloudToModelDistance(configDialog);
  configDialogAddSphereFollowingHyperParameters(configDialog);
  addOutputFormat(configDialog);
}

void
SF_StepSpherefollowingBasic::createOutResultModelListProtected()
{
  CT_OutResultModelGroupToCopyPossibilities* resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
  if (resModelw != NULL) {
    addQSMToOutResult(resModelw, QString("QSM SphereFollowing"), QString::fromUtf8(DEF_IN_GRP_CLUSTER));
    resModelw->addItemModel(_QSMGrp, _outSFQSM, new SF_QSM_Item(), tr("Internal QSM SphereFollowing"));
    resModelw->addItemModel(
      _QSMGrp, _outParams, new SF_SphereFollowing_Parameters_Item(), tr(" Internal parameters QSM SphereFollowing"));
  }
}

void
SF_StepSpherefollowingBasic::createInResultModelListProtected()
{
  CT_InResultModelGroupToCopy* resModel = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Input Result QSM SphereFollowing"));
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
SF_StepSpherefollowingBasic::adaptParametersToExpertLevel()
{
  if (!_isExpert) {
    _SF_OPT_euclideanClusteringDistance = 0.02;
    _SF_OPT_sphereRadiusMultiplier = 2;
    _SF_OPT_sphereEpsilon = 0.035;
    _SF_inlierDistance = 0.03;
    _SF_minPtsGeometry = 3;
    _SF_heightInitializationSlice = 0.1;
    _PARAMETERS_CHOICE_SPHERE_EPSILON = _PARAMETERS_7;
    _PARAMETERS_CHOICE_EUCLIDEAN_CLUSTERING_DISTANCE = _PARAMETERS_7;
    _PARAMETERS_CHOICE_SPHERE_RADIUS_MULTIPLIER = _PARAMETERS_7;
    _PP_voxelSize = 0.01;
    if (_choicePointDensity == _lowDensity) {
      _SF_OPT_sphereRadiusMultiplier = 3;
      _SF_OPT_euclideanClusteringDistance = 0.1;
      _SF_OPT_sphereEpsilon = 0.05;
    } else if (_choicePointDensity == _mediumDensity) {
      _SF_OPT_sphereRadiusMultiplier = 2.5;
      _SF_OPT_euclideanClusteringDistance = 0.04;
      _SF_OPT_sphereEpsilon = 0.5;
    } else {
      _SF_OPT_euclideanClusteringDistance = 0.02;
      _SF_OPT_sphereEpsilon = 0.035;
    }
  }
}

void
SF_StepSpherefollowingBasic::createPostConfigurationDialogCitationSecond(CT_StepConfigurableDialog* configDialog)
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
SF_StepSpherefollowingBasic::compute()
{
  const QList<CT_ResultGroup*>& out_result_list = getOutResultList();
  CT_ResultGroup* outResult = out_result_list.at(0);
  identifyAndRemoveCorruptedScenes(outResult);
  createParamList(outResult);
  writeLogger();
  QFuture<void> future = QtConcurrent::map(_paramList, SF_SpherefollowingRootAdapter());
  while (!future.isFinished()) {
    setProgressByCounter(10.0f, 85.0f);
  }
  SF_AbstractStepQSM::addQSM<SF_ParamSpherefollowingBasic<SF_PointNormal>>(outResult,
                                                                           _paramList,
                                                                           QString::fromUtf8(DEF_IN_GRP_CLUSTER),
                                                                           _outSFQSM.completeName(),
                                                                           _outParams.completeName(),
                                                                           QString::fromUtf8(DEF_IN_CLOUD_SEED));
  _paramList.clear();
}

QList<SF_ParamQSM<SF_PointNormal>>
SF_StepSpherefollowingBasic::paramList()
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
SF_StepSpherefollowingBasic::toStringSFMethod()
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
SF_StepSpherefollowingBasic::toStringCMDMethod()
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
SF_StepSpherefollowingBasic::paramsStringToNumber(const QString& UISelection)
{
  std::vector<double> paramVec;
  if (UISelection == _PARAMETERS_1) {
    paramVec.push_back(1.0);
  } else if (UISelection == _PARAMETERS_3) {
    paramVec.push_back(0.75);
    paramVec.push_back(1.0);
    paramVec.push_back(1.5);
  } else if (UISelection == _PARAMETERS_5) {
    paramVec.push_back(0.75);
    paramVec.push_back(1.0);
    paramVec.push_back(1.50);
    paramVec.push_back(2.0);
    paramVec.push_back(2.5);
  } else if (UISelection == _PARAMETERS_7) {
    paramVec.push_back(0.75);
    paramVec.push_back(0.9);
    paramVec.push_back(1.0);
    paramVec.push_back(1.4);
    paramVec.push_back(1.9);
    paramVec.push_back(2.5);
    paramVec.push_back(3.0);
  } else if (UISelection == _PARAMETERS_9) {
    paramVec.push_back(0.75);
    paramVec.push_back(0.9);
    paramVec.push_back(1.0);
    paramVec.push_back(1.4);
    paramVec.push_back(1.8);
    paramVec.push_back(2.4);
    paramVec.push_back(3.0);
    paramVec.push_back(3.7);
    paramVec.push_back(4.5);
  } else if (UISelection == _PARAMETERS_11) {
    paramVec.push_back(0.75);
    paramVec.push_back(0.9);
    paramVec.push_back(1.0);
    paramVec.push_back(1.4);
    paramVec.push_back(1.9);
    paramVec.push_back(2.4);
    paramVec.push_back(3.0);
    paramVec.push_back(4.0);
    paramVec.push_back(5.0);
    paramVec.push_back(6.0);
    paramVec.push_back(7.0);
  } else {
    throw("SF_StepSpherefollowingRoot: Invalid grid parameter selected.");
  }
  return paramVec;
}

void
SF_StepSpherefollowingBasic::createParamList(CT_ResultGroup* outResult)
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
    SF_ParamSpherefollowingBasic<SF_PointNormal> param;
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
  int numberClouds = _paramList.size();
  int numberComputationsPerCloud = sfOptimizationParameters._epsilonSphereMultiplier.size() *
                                   sfOptimizationParameters._euclideanClusteringDistanceMultiplier.size() *
                                   sfOptimizationParameters._sphereRadiusMultiplierMultiplier.size();
  m_computationsTotal = numberClouds * numberComputationsPerCloud;
}
