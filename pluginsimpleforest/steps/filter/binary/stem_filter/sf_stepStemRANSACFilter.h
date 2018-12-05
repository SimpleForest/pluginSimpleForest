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
#ifndef SF_STEP_STEM_RANSAC_FILTER_H
#define SF_STEP_STEM_RANSAC_FILTER_H

#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_view/ct_stepconfigurabledialog.h"
#include "steps/filter/binary/sf_abstractFilterBinaryStep.h"
#include "steps/param/sf_paramAllSteps.h"

class SF_StepStemRANSACFilter : public SF_AbstractFilterBinaryStep {
  Q_OBJECT

public:
  SF_StepStemRANSACFilter(CT_StepInitializeData &dataInit);
  ~SF_StepStemRANSACFilter();
  QString getStepDescription() const;
  QString getStepDetailledDescription() const;
  QString getStepURL() const;
  CT_VirtualAbstractStep *createNewInstance(CT_StepInitializeData &dataInit);
  QStringList getStepRISCitations() const;

protected:
  QList<SF_ParamStemRansacFilter> _paramList;
  void createInResultModelListProtected();
  void createOutResultModelListProtected();
  void adaptParametersToExpertLevel();
  void createPostConfigurationDialogBeginner(
      CT_StepConfigurableDialog *configDialog);
  void
  createPostConfigurationDialogExpert(CT_StepConfigurableDialog *configDialog);
  void compute();
  virtual void writeLogger();

private:
  double _x = 0;
  double _y = 0;
  double _z = 1;
  double _angle = 25;
  double _radiusNormal = 0.03;
  double _voxelSize = 0.01;
  double _sizeOutput = 2;
  double _inlierDistance = 0.1;
  void writeOutputPerScence(CT_ResultGroup *outResult, size_t i);
  void writeOutput(CT_ResultGroup *outResult);
  void createParamList(CT_ResultGroup *outResult);
};
#endif
