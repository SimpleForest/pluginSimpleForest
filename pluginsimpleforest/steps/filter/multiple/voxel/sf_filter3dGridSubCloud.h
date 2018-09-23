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
#ifndef SF_FILTER_3D_GRID_SUB_CLOUD_H
#define SF_FILTER_3D_GRID_SUB_CLOUD_H

#include <QObject>

#include "steps/filter/multiple/sf_abstractFilterMultipleStep.h"

class SF_Filter3dGridSubCloud: public SF_AbstractFilterMultipleStep
{
    Q_OBJECT

public:
    SF_Filter3dGridSubCloud(CT_StepInitializeData &data_init);
    ~SF_Filter3dGridSubCloud();
    QString getStepDescription() const;
    QString getStepDetailledDescription() const;
    QString getStepURL() const;
    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &data_init);
    QStringList getStepRISCitations() const;

protected:
    void createInResultModelListProtected();
    void createPostConfigurationDialog();
    void createOutResultModelListProtected();
    void createPreConfigurationDialog(){}
    void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *config_dialog){}
    void createPostConfigurationDialogExpert(CT_StepConfigurableDialog *config_dialog){}
    void adaptParametersToExpertLevel(){}
    void compute();

private:
    double _voxel_size = 3.0;
    CT_Grid3D_Sparse<int> *createGrid3dFromScene(const CT_Scene *ctCloud,
                                                 double voxelSize);
    void addPointToGridCluster(CT_Grid3D_Sparse<int>* hit_grid,
                               std::vector<CT_PointCloudIndexVector *> &clusters, CT_PointIterator & it);
    void createGridCluster(int &val, std::vector<CT_PointCloudIndexVector *> &clusters, const CT_Point &point, CT_Grid3D_Sparse<int>* hit_grid);
    void createGridClusterIfNeeded(int &val, std::vector<CT_PointCloudIndexVector *> &clusters, const CT_Point &point, CT_Grid3D_Sparse<int>* hit_grid);
    void addCloudToGridCluster(const CT_Scene* ct_cloud, std::vector<CT_PointCloudIndexVector *> &clusters);
};

#endif // SF_FILTER_3D_GRID_SUB_CLOUD_H
