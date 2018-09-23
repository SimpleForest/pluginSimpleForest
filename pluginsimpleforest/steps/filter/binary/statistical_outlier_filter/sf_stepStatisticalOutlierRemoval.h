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
#ifndef SF_STEP_STATISTICAL_OUTLIER_REMOVAL_H
#define SF_STEP_STATISTICAL_OUTLIER_REMOVAL_H

#include "steps/filter/binary/sf_abstractFilterBinaryStep.h"

#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include <steps/param/sf_paramAllSteps.h>

class SF_StepStatisticalOutlierRemoval:
        public SF_AbstractFilterBinaryStep {
    Q_OBJECT

public:
    SF_StepStatisticalOutlierRemoval(CT_StepInitializeData &dataInit);
    ~SF_StepStatisticalOutlierRemoval();
    QString getStepDescription() const;
    QString getStepDetailledDescription() const;
    QString getStepURL() const;
    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &dataInit);
    QStringList getStepRISCitations() const;

protected:
    void createInResultModelListProtected();
    void createOutResultModelListProtected();
    void adaptParametersToExpertLevel();
    void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *configDialog);
    void createPostConfigurationDialogExpert(CT_StepConfigurableDialog *configDialog);
    void compute();
    QList<SF_ParamStatisticalOutlierFilter<SF_Point> > _paramList;
    virtual void writeLogger();

private:
    QString _less = "less";
    QString _intermediate = "intermediate";
    QString _many         = "many";
    QString _choicePointDensity       = _intermediate;
    double _std_mult = 3.0;
    int _iterations = 5;
    int _k = 2;
    void writeOutputPerScence(CT_ResultGroup* outResult,
                                 size_t i);
    void writeOutput(CT_ResultGroup* outResult);
    void createParamList(CT_ResultGroup *outResult);
};

#endif // SF_STEP_STATISTICAL_OUTLIER_REMOVAL_H
