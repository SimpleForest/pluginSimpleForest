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

#include "sf_stepSpherefollowingRoot.h"
#include "sf_stepSpherefollowingBasicAdapter.h"
#include <QtConcurrent/QtConcurrent>

#include <ct_itemdrawable/ct_cylinder.h>

SF_StepSpherefollowingRoot::SF_StepSpherefollowingRoot(CT_StepInitializeData& dataInit) : SF_AbstractStepSegmentation(dataInit)
{
  _pointDensities.append(_lowDensity);
  _pointDensities.append(_mediumDensity);
  _pointDensities.append(_highDensity);
}

SF_StepSpherefollowingRoot::~SF_StepSpherefollowingRoot() {}

QString
SF_StepSpherefollowingRoot::getStepDescription() const
{
  return tr("SphereFollowing Basic");
}

QString
SF_StepSpherefollowingRoot::getStepDetailledDescription() const
{
  return tr("This implementation of the SphereFollowing method utilizes an "
            "unsegmented tree cloud. Only one set of parameters will be optimized."
            "Results in a fast QSM estimation with less accuracy.");
}

QString
SF_StepSpherefollowingRoot::getStepURL() const
{
  return tr("");
}

CT_VirtualAbstractStep*
SF_StepSpherefollowingRoot::createNewInstance(CT_StepInitializeData& dataInit)
{
  return new SF_StepSpherefollowingRoot(dataInit);
}

QStringList
SF_StepSpherefollowingRoot::getStepRISCitations() const
{
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  _risCitationList.append(getRISCitationSphereFollowing());
  _risCitationList.append(getRISCitationPCL());
  return _risCitationList;
}

void
SF_StepSpherefollowingRoot::createInResultModelListProtected()
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
}

void
SF_StepSpherefollowingRoot::configDialogAddSphereFollowingHyperParameters(CT_StepConfigurableDialog* configDialog)
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
  configDialog->addBool(
    "Or you can let Simple Forest choose method parameters", " select if you want auto parameters", "", _SF_parameterAutoSearch);
  configDialog->addEmpty();
}

void
SF_StepSpherefollowingRoot::configDialogAddSphereFollowingOptimizableParameters(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addText("<b>SphereFollowing Method Optimizationable Parameters</b>:");
  configDialog->addText("You can pre select also optimizable sphere following parameters:");
  configDialog->addDouble("To generate a sphere each fitted circle is multiplied with the "
                          "[<em><b>sphere multiplier</b></em>]",
                          " ",
                          1.4,
                          4,
                          2,
                          _SF_OPT_sphereRadiusMultiplier);
  configDialog->addDouble(
    "The sphere surface has a thickness of [<em><b>sphere epsilon</b></em>] ", " (m).", 0.01, 0.1, 2, _SF_OPT_sphereEpsilon);
  configDialog->addDouble("Point on sphere surface are clustered with threshold  [<em><b>euclidean "
                          "clustering distance</b></em>]  ",
                          " (m) to build new circle clusters.",
                          0.02,
                          0.1,
                          2,
                          _SF_OPT_euclideanClusteringDistance);
  configDialog->addEmpty();
}

void
SF_StepSpherefollowingRoot::configDialogGuruAddSphereFollowing(CT_StepConfigurableDialog* configDialog)
{
  configDialogAddSphereFollowingHyperParameters(configDialog);
  // configDialogAddSphereFollowingOptimizableParameters(configDialog);
}

void
SF_StepSpherefollowingRoot::configDialogGuruAddPreProcessing(CT_StepConfigurableDialog* configDialog)
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
SF_StepSpherefollowingRoot::configDialogGuruAddGridSearchCloudToModelDistance(CT_StepConfigurableDialog* configDialog)
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
  configDialog->addInt("For each point the distance to the model is computed. "
                       "Only a  [<em><b>robust percentage</b></em>] ",
                       " of smallest distances is used.",
                       50,
                       100,
                       _CMD_robustPercentage);
  configDialog->addDouble("For MSAC and inlier methods the distance is cropped "
                          "at [<em><b>crop distance</b></em>] ",
                          "",
                          0.01,
                          0.3,
                          2,
                          _CMD_inlierDistance);
  configDialog->addInt("Sorting by growth length the cloud is subdivided in  "
                       "[<em><b>numClstrs</b></em>] ",
                       " clusters.",
                       1,
                       10,
                       _CMD_numClstrs);
  configDialog->addEmpty();
}

void
SF_StepSpherefollowingRoot::createPostConfigurationDialogExpert(CT_StepConfigurableDialog* configDialog)
{
  configDialogGuruAddPreProcessing(configDialog);
  configDialogGuruAddSphereFollowing(configDialog);
  configDialogGuruAddGridSearchCloudToModelDistance(configDialog);
}

void
SF_StepSpherefollowingRoot::createPostConfigurationDialogBeginner(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addStringChoice("Choose the quality of the point cloud", "", _pointDensities, _choicePointDensity);
}

void
SF_StepSpherefollowingRoot::createOutResultModelListProtected()
{
  CT_OutResultModelGroupToCopyPossibilities* resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
  if (resModelw != NULL) {
    resModelw->addGroupModel(DEF_IN_GRP_CLUSTER, _outCylinderGroup, new CT_StandardItemGroup(), tr("QSM Group"));
    resModelw->addItemModel(DEF_IN_GRP_CLUSTER, m_outCloudItem, new CT_PointsAttributesColor(), tr("Spherefollowing Group Color"));
    resModelw->addItemModel(_outCylinderGroup, _outCylinders, new CT_Cylinder(), tr("QSM"));
  }
}

void
SF_StepSpherefollowingRoot::adaptParametersToExpertLevel()
{
  if (!_isExpert) {
    _SF_OPT_euclideanClusteringDistance = 0.02;
    _SF_OPT_sphereRadiusMultiplier = 2;
    _SF_OPT_sphereEpsilon = 0.035;
    _SF_inlierDistance = 0.03;
    _SF_minPtsGeometry = 3;
    _SF_heightInitializationSlice = 0.1;
    _PP_voxelSize = 0.01;
    if (_choicePointDensity == _lowDensity) {
      _SF_OPT_euclideanClusteringDistance = 0.1;
      _SF_OPT_sphereEpsilon = 0.05;
    } else if (_choicePointDensity == _mediumDensity) {
      _SF_OPT_euclideanClusteringDistance = 0.04;
      _SF_OPT_sphereEpsilon = 0.5;
    } else {
      _SF_OPT_euclideanClusteringDistance = 0.02;
      _SF_OPT_sphereEpsilon = 0.035;
    }
  }
}

void
SF_StepSpherefollowingRoot::createPostConfigurationDialogCitationSecond(CT_StepConfigurableDialog* configDialog)
{
  configDialog->addText(QObject::tr("For this step please cite in addition:"),
                        "Hackenberg, J.; Morhart, C.; Sheppard, J.; Spiecker, H.; Disney, M.");
  configDialog->addText("",
                        "<em>Highly Accurate Tree Models Derived from Terrestrial Laser Scan "
                        "Data: A Method Description.</em>");
  configDialog->addText("", "Forests <b>2014</b>, 5, 1069-1105.");
}

void
SF_StepSpherefollowingRoot::compute()
{
  const QList<CT_ResultGroup*>& out_result_list = getOutResultList();
  CT_ResultGroup* outResult = out_result_list.at(0);
  identifyAndRemoveCorruptedScenes(outResult);
  createParamList(outResult);
  writeLogger();
  QFuture<void> future = QtConcurrent::map(_paramList, SF_SpherefollowingRootAdapter());
  setProgressByFuture(future, 10, 85);
  addQSM(outResult, paramList(), QString::fromUtf8(DEF_IN_GRP_CLUSTER), _outCylinders.completeName(), _outCylinderGroup.completeName());
  addColors(outResult, paramList(), DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, m_outCloudItem.completeName());
}

QList<SF_ParamQSM<SF_PointNormal> > SF_StepSpherefollowingRoot::paramList()
{
    QList<SF_ParamQSM<SF_PointNormal> > paramList;
    std::for_each(
      _paramList.begin(), _paramList.end(), [&paramList](SF_ParamSpherefollowingBasic<SF_PointNormal>& params) {
        SF_ParamQSM<SF_PointNormal> param;
        param._tree = params._tree;
        param._translation = params._translation;
        param._colors = params._colors;
        paramList.push_back(param);
    });
    return paramList;
}

int
SF_StepSpherefollowingRoot::toStringSFMethod()
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

int
SF_StepSpherefollowingRoot::toStringCMDMethod()
{
  int type = -1;
  if (_CMD_methodChoice == _ZEROMOMENTUMORDER)
    type = SF_CLoudToModelDistanceMethod::ZEROMOMENTUMORDER;
  if (_CMD_methodChoice == _FIRSTMOMENTUMORDERMSAC)
    type = SF_CLoudToModelDistanceMethod::FIRSTMOMENTUMORDERMSAC;
  if (_CMD_methodChoice == _FIRSTMOMENTUMORDER)
    type = SF_CLoudToModelDistanceMethod::FIRSTMOMENTUMORDER;
  if (_CMD_methodChoice == _SECONDMOMENTUMORDERMSAC)
    type = SF_CLoudToModelDistanceMethod::SECONDMOMENTUMORDERMSAC;
  if (_CMD_methodChoice == _SECONDMOMENTUMORDER)
    type = SF_CLoudToModelDistanceMethod::SECONDMOMENTUMORDER;
  return type;
}

void
SF_StepSpherefollowingRoot::createParamList(CT_ResultGroup* outResult)
{
  SF_SphereFollowingParameters sphereFollowingParams;
  SF_SphereFollowingOptimizationParameters sfOptimizationParameters;
  sfOptimizationParameters._epsilonSphere = _SF_OPT_sphereEpsilon;
  sfOptimizationParameters._euclideanClusteringDistance = _SF_OPT_euclideanClusteringDistance;
  sfOptimizationParameters._sphereRadiusMultiplier = _SF_OPT_sphereRadiusMultiplier;
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
  distanceParams._inlierDistance = _CMD_inlierDistance;
  distanceParams._robustPercentage = _CMD_robustPercentage;

  adaptParametersToExpertLevel();
  CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
  while (!isStopped() && outResIt.hasNext()) {
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt.next();
    const CT_AbstractItemDrawableWithPointCloud* ctCloud = (const CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(
      this, DEF_IN_CLOUD_SEED);
    SF_ParamSpherefollowingBasic<SF_PointNormal> param;
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
}
