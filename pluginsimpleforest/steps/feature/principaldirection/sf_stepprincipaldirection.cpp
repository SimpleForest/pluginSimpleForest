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

#include "steps/feature/principaldirection/sf_adapterprincipaldirection.h"

#include "ct_colorcloud/ct_colorcloudstdvector.h"
#include "ct_itemdrawable/ct_pointsattributescolor.h"
#include "ct_itemdrawable/ct_scene.h"
#include "ct_result/ct_resultgroup.h"
#include "ct_result/model/inModel/ct_inresultmodelgroup.h"
#include "ct_result/model/outModel/ct_outresultmodelgroup.h"

#include <QtConcurrent/QtConcurrent>

#define DEF_outColorResult "outColorResult";
#define DEF_outColorGrp "outColorGrp";
#define DEF_outColorItem "outColorItem";

SF_StepPrincipalDirection::SF_StepPrincipalDirection(
    CT_StepInitializeData &dataInit)
    : SF_AbstractStepFeature(dataInit) {}

SF_StepPrincipalDirection::~SF_StepPrincipalDirection() {}

QString SF_StepPrincipalDirection::getStepDescription() const {
  return tr("PrincipalDirection");
}

QString SF_StepPrincipalDirection::getStepDetailledDescription() const {
  return tr(
      "This implementation of the SphereFollowing method utilizes an "
      "unsegmented tree cloud. Only one set of parameters will be optimized."
      "Results in a fast QSM estimation with less accuracy.");
}

QString SF_StepPrincipalDirection::getStepURL() const { return tr(""); }

CT_VirtualAbstractStep *
SF_StepPrincipalDirection::createNewInstance(CT_StepInitializeData &dataInit) {
  return new SF_StepPrincipalDirection(dataInit);
}

QStringList SF_StepPrincipalDirection::getStepRISCitations() const {
  QStringList _risCitationList;
  _risCitationList.append(getRISCitationSimpleTree());
  _risCitationList.append(getRISCitationPCL());
  _risCitationList.append(getRISCitationRaumonen());
  return _risCitationList;
}

void SF_StepPrincipalDirection::createInResultModelListProtected() {
  CT_InResultModelGroupToCopy *resModel = createNewInResultModelForCopy(
      DEF_IN_RESULT, tr("Point Cloud"), "", false);
  resModel->setZeroOrMoreRootGroup();
  resModel->addGroupModel(
      "", DEF_IN_GRP_CLUSTER, CT_AbstractItemGroup::staticGetType(),
      tr("Point Group"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
  resModel->addItemModel(DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED,
                         CT_Scene::staticGetType(), tr("Point Cloud"));
}

void SF_StepPrincipalDirection::createOutResultModelListProtected() {
  CT_OutResultModelGroupToCopyPossibilities *resOutModel =
      createNewOutResultModelToCopy(DEF_IN_RESULT);
  if (resOutModel != NULL) {
    resOutModel->addItemModel(DEF_IN_GRP_CLUSTER, m_outCloudItem,
                              new CT_PointsAttributesColor(),
                              tr("Principal Component"));
  }
}

void SF_StepPrincipalDirection::createPostConfigurationDialogCitationSecond(
    CT_StepConfigurableDialog *configDialog) {
  addCitationRaumonen(configDialog);
}

void SF_StepPrincipalDirection::compute() {
  const QList<CT_ResultGroup *> &out_result_list = getOutResultList();
  CT_ResultGroup *outResult = out_result_list.at(0);
  identifyAndRemoveCorruptedScenes(outResult);
  adaptParametersToExpertLevel();
  processInput(outResult);
  writeLogger();
  // QFuture<void> future = QtConcurrent::map(_paramList,
  // SF_SpherefollowingRootAdapter());
  //    setProgressByFuture(future,
  //                      10,
  //                    85);
}

void SF_StepPrincipalDirection::createPostConfigurationDialogBeginner(
    CT_StepConfigurableDialog *configDialog) {}

void SF_StepPrincipalDirection::createPostConfigurationDialogExpert(
    CT_StepConfigurableDialog *configDialog) {
  configDialog->addText("First to enable multithreading, the point cloud is "
                        "clustered with voxelization.");
  configDialog->addDouble("A voxelization with [<em><b>voxel size</b></em>]:",
                          " (m) sized voxels divides the cloud in n clusters.",
                          0.5, 10, 2, m_voxelSizeCluster);
  configDialog->addText(".");
  configDialog->addDouble("Then the cloud is downscaled with "
                          "[<em><b>voxelGridCluster cell size</b></em>]:",
                          " (m).", 0.01, 0.5, 3, m_voxelSizeDownscaling);
  configDialog->addText(
      "Then this step computes the <b>principal direction</b>, e.g. the second "
      "normal derivate, for a point cloud.");
  configDialog->addDouble("The [<em><b>normal radius</b></em>]:", " (m)", 0.01,
                          0.5, 2, m_normalRadius, 1,
                          "Used for the normal computation.");
  configDialog->addDouble(
      "The [<em><b>principal direction radius</b></em>]:", " (m)", 0.01, 0.5, 2,
      m_pdRadius, 1, "Used for the principal curvature computation.");
  configDialog->addText(
      "The output is stored color coded on the backscaled cloud.");
}

std::vector<
    std::pair<pcl::PointCloud<SF_PointNormal>::Ptr, std::vector<size_t>>>
SF_StepPrincipalDirection::voxelizeInput(
    SF_ConverterCTCloudToPCLCloud<SF_PointNormal> &converter) {
  m_parameterVoxelization.m_voxelSize = m_voxelSizeCluster;
  m_parameterVoxelization.m_cloud = converter.cloudOut();
  SF_VoxelClustering<SF_PointNormal> voxelCluster;
  voxelCluster.setParam(m_parameterVoxelization);
  voxelCluster.compute();
  auto strList = m_parameterVoxelization.paramsToString();
  for (auto &str : strList) {
    PS_LOG->addMessage(LogInterface::info, LogInterface::step, str);
  }
  return voxelCluster.clusterOut();
}

void SF_StepPrincipalDirection::processInput(CT_ResultGroup *outResult) {
  CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
  while (!isStopped() && outResIt.hasNext()) {
    CT_StandardItemGroup *group = (CT_StandardItemGroup *)outResIt.next();
    CT_AbstractItemDrawableWithPointCloud *ctCloud =
        (CT_AbstractItemDrawableWithPointCloud *)group->firstItemByINModelName(
            this, DEF_IN_CLOUD_SEED);
    SF_ConverterCTCloudToPCLCloud<SF_PointNormal> converter(ctCloud);
    converter.compute();

    SF_ParameterSetPrincipalDirection<SF_PointNormal> param;
    param.m_paramVoxelGridDownscaling.m_voxelSize = m_voxelSizeDownscaling;
    param.m_paramVoxelGridDownscaling.m_cloud = converter.cloudOut();
    param.m_normalRadius = m_normalRadius;
    param.m_pdRadius = m_pdRadius;

    std::vector<
        std::pair<pcl::PointCloud<SF_PointNormal>::Ptr, std::vector<size_t>>>
        clusters = voxelizeInput(converter);
    QList<SF_ParameterSetPrincipalDirection<SF_PointNormal>> paramList;
    std::for_each(
        clusters.begin(), clusters.end(),
        [&paramList,
         &param](const std::pair<pcl::PointCloud<SF_PointNormal>::Ptr,
                                 std::vector<size_t>> &cluster) {
          SF_ParameterSetPrincipalDirection<SF_PointNormal> paramCluster =
              param;
          paramCluster.m_paramVoxelGridDownscaling.m_cloud = cluster;
          paramList.append(paramCluster);
        });
    pcl::PointCloud<SF_PointNormal>::Ptr cloud(
        new pcl::PointCloud<SF_PointNormal>);
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr featureCloud(
        new pcl::PointCloud<pcl::PrincipalCurvatures>);
    QFuture<void> future =
        QtConcurrent::map(paramList, SF_AdapterPrincipalDirection());
    setProgressByFuture(future, 10, 85);
    std::for_each(
        paramList.begin(), paramList.end(),
        [&cloud, &featureCloud](
            SF_ParameterSetPrincipalDirection<SF_PointNormal> &param) {
          *cloud += *(param.m_cloud.first);
          *featureCloud += *(param.m_principalCurvatures);
        });
    pcl::KdTreeFLANN<SF_PointNormal> kdtree;
    kdtree.setInputCloud(cloud);
    pcl::PointCloud<SF_PointNormal>::Ptr cloudIn = converter.cloudOut().first;
    CT_ColorCloudStdVector *colors =
        new CT_ColorCloudStdVector(cloudIn->points.size());
    size_t index = 0;
    std::for_each(
        cloudIn->points.begin(), cloudIn->points.end(),
        [&kdtree, &featureCloud, &index, colors](SF_PointNormal &point) {
          std::vector<int> indices(1);
          std::vector<float> distances(1);
          CT_Color &col = colors->colorAt(index++);
          if (kdtree.nearestKSearch(point, 1, indices, distances)) {
            pcl::PrincipalCurvatures feature = featureCloud->points[indices[0]];
            Eigen::Vector3f vec(feature.principal_curvature_x,
                                feature.principal_curvature_y,
                                feature.principal_curvature_z);
            if (feature.principal_curvature_z < 0) {
              vec = Eigen::Vector3f(-feature.principal_curvature_x,
                                    -feature.principal_curvature_y,
                                    -feature.principal_curvature_z);
            }
            Eigen::Vector3f vecNorm = vec.normalized();
            col.r() = (std::abs((vecNorm[0] * 126) + 127));
            col.g() = (std::abs((vecNorm[1] * 126) + 127));
            col.b() = (std::abs((vecNorm[2] * 250)));

            //                col.r() = (std::sqrt((vecNorm[0])*(vecNorm[0])
            //                +(vecNorm[1])*(vecNorm[1]))*126); col.g() =
            //                (std::sqrt((vecNorm[0])*(vecNorm[0])
            //                +(vecNorm[1])*(vecNorm[1]))*126); col.b() =
            //                (std::abs((vecNorm[2]*250)));
          }
        });
    CT_PointsAttributesColor *colorAttribute = new CT_PointsAttributesColor(
        m_outCloudItem.completeName(), outResult,
        ctCloud->getPointCloudIndexRegistered(), colors);
    group->addItemDrawable(colorAttribute);
  }
}
