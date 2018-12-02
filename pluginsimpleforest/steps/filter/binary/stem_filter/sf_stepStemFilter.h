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
#ifndef SF_STEP_STEM_FILTER_H
#define SF_STEP_STEM_FILTER_H

#include "steps/param/sf_paramAllSteps.h"
#include "steps/filter/binary/sf_abstractFilterBinaryStep.h"
#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"

class SF_StepStemFilter:
        public SF_AbstractFilterBinaryStep {
    Q_OBJECT

public:
    SF_StepStemFilter(CT_StepInitializeData &dataInit);
    ~SF_StepStemFilter();
    QString getStepDescription() const;
    QString getStepDetailledDescription() const;
    QString getStepURL() const;
    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &data_init);
    QStringList getStepRISCitations() const;

protected:
    QList<SF_ParamStemFilter<SF_PointNormal> > _paramList;
    void createInResultModelListProtected();
    void createOutResultModelListProtected();
    void adaptParametersToExpertLevel();
    void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *configDialog);
    void createPostConfigurationDialogExpert(CT_StepConfigurableDialog *configDialog);
    void compute();
    virtual void writeLogger();

private:
    double _x = 0;
    double _y = 0;
    double _z = 1;
    double _angle = 25;
    double _radiusGrowthDirection = 0.4;
    double _radiusNormal = 0.2;
    double _voxelSize = 0.04;
    double _sizeOutput = 2;
    CT_AutoRenameModels m_outCloudItem;
    void writeOutputPerScence(CT_ResultGroup* outResult, size_t i);
    void writeOutput(CT_ResultGroup* outResult);
    void createParamList(CT_ResultGroup *out_result);
};

#endif // SF_STEP_STEM_FILTER_H
