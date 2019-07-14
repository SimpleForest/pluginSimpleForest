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

#include "sf_filter3dGridSubCloud.h"

#include <algorithm>

SF_Filter3dGridSubCloud::SF_Filter3dGridSubCloud(CT_StepInitializeData& dataInit) : SF_AbstractFilterMultipleStep(dataInit)
{
  _voxelSize = 3.0;
}

SF_Filter3dGridSubCloud::~SF_Filter3dGridSubCloud() {}

QString
SF_Filter3dGridSubCloud::getStepDescription() const
{
  return tr("Voxelises the cloud into a 3d Grid.");
}

QString
SF_Filter3dGridSubCloud::getStepDetailledDescription() const
{
  return tr("Voxelises the cloud into a 3d Grid. Improves the runtime of "
            "various filters by enabling multithreading.");
}

QString
SF_Filter3dGridSubCloud::getStepURL() const
{
  return tr("");
}

CT_VirtualAbstractStep*
SF_Filter3dGridSubCloud::createNewInstance(CT_StepInitializeData& dataInit)
{
  return new SF_Filter3dGridSubCloud(dataInit);
}

QStringList
SF_Filter3dGridSubCloud::getStepRISCitations() const
{
  QStringList _risCitationList;
  _risCitationList.append(SF_AbstractStep::getRISCitationSimpleTree());
  return _risCitationList;
}

void
SF_Filter3dGridSubCloud::createInResultModelListProtected()
{
  CT_InResultModelGroupToCopy* resModel = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
  resModel->setZeroOrMoreRootGroup();
  resModel->addGroupModel("", DEF_IN_GRP_CLUSTER);
  resModel->addItemModel(
    DEF_IN_GRP_CLUSTER, DEF_IN_CLOUD_SEED, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Point Cloud"));
}

void
SF_Filter3dGridSubCloud::createPostConfigurationDialog()
{
  CT_StepConfigurableDialog* configDialog = newStandardPostConfigurationDialog();
  configDialog->addDouble("The x,y,z voxelsize size of the output clouds ", " ", 0.1, 10, 4, _voxelSize);
  createPostConfigurationDialogCitation(configDialog);
}

void
SF_Filter3dGridSubCloud::createOutResultModelListProtected()
{
  CT_OutResultModelGroupToCopyPossibilities* resModelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
  if (resModelw != NULL) {
    resModelw->addGroupModel(DEF_IN_GRP_CLUSTER, _outGrpCluster, new CT_StandardItemGroup(), tr("Grid"));
    resModelw->addItemModel(_outGrpCluster, _outCloudCluster, new CT_Scene(), tr("Sub Cloud Grid Cell"));
  }
}

CT_Grid3D_Sparse<int>*
SF_Filter3dGridSubCloud::createGrid3dFromScene(const CT_Scene* ctCloud, double voxelSize)
{
  Eigen::Vector3f min = getMin(ctCloud);
  Eigen::Vector3f max = getMax(ctCloud);
  CT_Grid3D_Sparse<int>* hitGrid = CT_Grid3D_Sparse<int>::createGrid3DFromXYZCoords(
    NULL, NULL, min(0), min(1), min(2), max(0), max(1), max(2), voxelSize, -2, -1);
  return hitGrid;
}

void
SF_Filter3dGridSubCloud::createGridCluster(int& val,
                                           std::vector<CT_PointCloudIndexVector*>& clusters,
                                           const CT_Point& point,
                                           CT_Grid3D_Sparse<int>* hitGrid)
{
  val = static_cast<int>(clusters.size());
  hitGrid->setValueAtXYZ(point(0), point(1), point(2), val);
  CT_PointCloudIndexVector* cluster = new CT_PointCloudIndexVector();
  clusters.push_back(cluster);
}

void
SF_Filter3dGridSubCloud::createGridClusterIfNeeded(int& val,
                                                   std::vector<CT_PointCloudIndexVector*>& clusters,
                                                   const CT_Point& point,
                                                   CT_Grid3D_Sparse<int>* hitGrid)
{
  if (val < 0) {
    createGridCluster(val, clusters, point, hitGrid);
  }
}

void
SF_Filter3dGridSubCloud::addPointToGridCluster(CT_Grid3D_Sparse<int>* hitGrid,
                                               std::vector<CT_PointCloudIndexVector*>& clusters,
                                               CT_PointIterator& it)
{
  const CT_Point& point = it.next().currentPoint();
  int val = hitGrid->valueAtXYZ(point(0), point(1), point(2));
  createGridClusterIfNeeded(val, clusters, point, hitGrid);
  size_t indexCt = it.currentGlobalIndex();
  clusters.at(val)->addIndex(indexCt);
}

void
SF_Filter3dGridSubCloud::addCloudToGridCluster(const CT_Scene* ctCloud, std::vector<CT_PointCloudIndexVector*>& clusters)
{
  CT_Grid3D_Sparse<int>* hitGrid = createGrid3dFromScene(ctCloud, _voxelSize);
  const CT_AbstractPointCloudIndex* pointIndex = ctCloud->getPointCloudIndex();
  CT_PointIterator it(pointIndex);
  while (it.hasNext()) {
    addPointToGridCluster(hitGrid, clusters, it);
  }
}

void
SF_Filter3dGridSubCloud::compute()
{
  const QList<CT_ResultGroup*>& outResultList = getOutResultList();
  CT_ResultGroup* outResult = outResultList.at(0);
  identifyAndRemoveCorruptedScenes(outResult);
  CT_ResultGroupIterator outResIt(outResult, this, DEF_IN_GRP_CLUSTER);
  while (!isStopped() && outResIt.hasNext()) {
    std::vector<CT_PointCloudIndexVector*> clusters;
    CT_StandardItemGroup* group = (CT_StandardItemGroup*)outResIt.next();
    const CT_Scene* ctCloud = (const CT_Scene*)group->firstItemByINModelName(this, DEF_IN_CLOUD_SEED);
    addCloudToGridCluster(ctCloud, clusters);
    std::sort(clusters.begin(), clusters.end(), sfCompareCTCloudsBySize);
    writeOutput(outResult, clusters, group);
  }
}
