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

#ifndef SF_DTM_STEP_H
#define SF_DTM_STEP_H

#include "steps/param/sf_paramAllSteps.h"
#include "steps/filter/binary/sf_abstractFilterBinaryStep.h"
#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_itemdrawable/ct_image2d.h"

class SF_StepDTM:  public SF_AbstractStep {
    Q_OBJECT

public:
    SF_StepDTM(CT_StepInitializeData &dataInit);
    ~SF_StepDTM();
    QString getStepDescription() const;
    QString getStepDetailledDescription() const;
    QString getStepURL() const;
    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &dataInit);
    QStringList getStepRISCitations() const;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr convert(CT_Scene* scene);

protected:
    void createInResultModelListProtected();
    void createOutResultModelListProtected();
    void adaptParametersToExpertLevel();
    void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *configDialog);
    void createPostConfigurationDialogExpert(CT_StepConfigurableDialog *configDialog);
    void compute();
    virtual void writeLogger();

private:
    QString _less         = "no slope";
    QString _intermediate = "intermediate slope";
    QString _many         = "hard slope";
    QString _choicePointDensity       = _intermediate;

    double _angle = 20;
    double _radiusNormal = 0.2;
    double _cellSize = 0.2;
    int _medianNeighbors = 9;
    int _idwNeighbors = 3;
    double _voxelSize = 0.05;

    CT_AutoRenameModels     _outDTM;
    CT_AutoRenameModels     _outDTMDummy;
    CT_AutoRenameModels     _outCloud;
    CT_AutoRenameModels     _outGroundGRP;
    Eigen::Vector3d _translate;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr createGroundCloud(CT_ResultGroup *outResult,
                                                                 CT_StandardItemGroup *terrainGrp);
    void computeNormals(pcl::PointCloud<pcl::PointXYZINormal>::Ptr downscaledCloud);
    CT_Scene * addGroundCloudToResult(CT_PointCloudIndexVector *mergedClouds,
                                      CT_StandardItemGroup* root,
                                      CT_ResultGroup *outResult);
    void copyCroppedHeights(pcl::PointCloud<pcl::PointXYZINormal>::Ptr groundCloud,
                            std::shared_ptr<CT_Image2D<float> > dtmPtr,
                            CT_Image2D<float>* CTDTM);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr downScale(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud);
};

#endif // SF_DTM_STEP_H
