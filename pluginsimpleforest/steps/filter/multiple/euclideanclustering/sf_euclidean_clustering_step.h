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

#ifndef SF_EUCLIDEAN_CLUSTERING_STEP_H
#define SF_EUCLIDEAN_CLUSTERING_STEP_H

#include "steps/filter/multiple/sf_abstract_filter_multiple_step.h"

class SF_Euclidean_Clustering_Step: public SF_Abstract_Filter_Multiple_Step
{
    Q_OBJECT

public:
    SF_Euclidean_Clustering_Step(CT_StepInitializeData &data_init);
    ~SF_Euclidean_Clustering_Step();
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
    void adapt_parameters_to_expert_level(){}
    void createParamList(CT_ResultGroup * out_result);
    void compute();

    void mergeClustersToPCLCloud(std::vector<size_t> indices, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCL, CT_ResultGroupIterator out_res_it);
    
private:
    double _voxelSize = 0.04;
    double _euclideanDistance = 0.1;
    int    _minPts = 1;
};

#endif // SF_EUCLIDEAN_CLUSTERING_STEP_H