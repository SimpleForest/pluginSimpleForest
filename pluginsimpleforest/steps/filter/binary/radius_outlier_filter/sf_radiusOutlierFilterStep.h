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
#ifndef SF_RADIUS_OUTLIER_FILTER_STEP_H
#define SF_RADIUS_OUTLIER_FILTER_STEP_H

#include "steps/filter/binary/sf_abstractFilterBinaryStep.h"
#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"

class SF_RadiusOutlierFilterStep:
        public SF_AbstractFilterBinaryStep {
    Q_OBJECT

public:
    SF_RadiusOutlierFilterStep(CT_StepInitializeData &dataInit);
    ~SF_RadiusOutlierFilterStep();
    QString getStepDescription() const;
    QString getStepDetailledDescription() const;
    QString getStepURL() const;
    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &dataInit);
    QStringList getStepRISCitations() const;

protected:
    QList<SF_ParamRadiusOutlierFilter<SF_PointNormal> > _paramList;
    void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *configDialog);
    void createPostConfigurationDialogExpert(CT_StepConfigurableDialog *configDialog);
    void createInResultModelListProtected();
    void createOutResultModelListProtected();
    void compute();
    virtual void writeLogger();
    void adaptParametersToExpertLevel();

private:
    QString _clearSky    = "clear sky";
    QString _choicePointDensity       = _intermediate;
    double _radius = 0.03;
    int _minPts = 5;
    void writeOutputPerScence(CT_ResultGroup* outResult, size_t i);
    void writeOutput(CT_ResultGroup* outResult);
    void createParamList(CT_ResultGroup *outResult);
};
#endif // SF_RADIUS_OUTLIER_FILTER_STEP_H
