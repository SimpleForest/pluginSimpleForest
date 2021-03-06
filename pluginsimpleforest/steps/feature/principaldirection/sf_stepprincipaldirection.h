/****************************************************************************

    Copyright (C) 2017-2018 , Dr. Jan Hackenberg

    sf_stepprincipaldirection.h is part of SimpleForest - a plugin for the
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

#ifndef SF_STEPPRINCIPALDIRECTION_H
#define SF_STEPPRINCIPALDIRECTION_H

#include <parameter/sf_parameterSetPrincipalDirection.h>

#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_view/ct_stepconfigurabledialog.h"
#include "steps/feature/sf_abstractstepfeature.h"

class SF_StepPrincipalDirection : public SF_AbstractStepFeature
{
  Q_OBJECT

public:
  SF_StepPrincipalDirection(CT_StepInitializeData& dataInit);
  ~SF_StepPrincipalDirection();
  QString getStepDescription() const;
  QString getStepDetailledDescription() const;
  QString getStepURL() const;
  CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData& data_init);
  QStringList getStepRISCitations() const;

protected:
  QList<SF_ParameterSetPrincipalDirection<pcl::PointXYZINormal>> _paramList;
  void createInResultModelListProtected();
  void createPostConfigurationDialogCitationSecond(CT_StepConfigurableDialog* configDialog);
  void createOutResultModelListProtected();
  void adaptParametersToExpertLevel() {}
  void compute();
  virtual void writeLogger() {}
  virtual void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog* configDialog);
  virtual void createPostConfigurationDialogExpert(CT_StepConfigurableDialog* configDialog);

  std::vector<std::pair<pcl::PointCloud<SF_PointNormal>::Ptr, std::vector<size_t>>> voxelizeInput(
    SF_ConverterCTCloudToPCLCloud<SF_PointNormal>& converter);

private:
  SF_ParameterSetVoxelization<SF_PointNormal> m_parameterVoxelization;
  double m_voxelSizeCluster = 3.00;
  double m_voxelSizeDownscaling = 0.02;
  double m_normalRadius = 0.075;
  double m_pdRadius = 0.15;
  SF_ParameterSetPrincipalDirection<pcl::PointXYZINormal> m_param;
  CT_AutoRenameModels m_outCloudItem;
  void writeOutputPerScence(CT_ResultGroup* outResult, size_t i);
  void writeOutput(CT_ResultGroup* outResult);
  void processInput(CT_ResultGroup* out_result);
};

#endif // SF_STEPPRINCIPALDIRECTION_H
