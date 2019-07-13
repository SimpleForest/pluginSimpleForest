#ifndef SF_STEPREVERSEPIPEMODELCORRECTION_H
#define SF_STEPREVERSEPIPEMODELCORRECTION_H

#include "steps/qsm/sf_abstractStepQSM.h"

#include <QString>

class SF_StepReversePipeModelCorrection : public SF_AbstractStepQSM
{
  Q_OBJECT

public:
  SF_StepReversePipeModelCorrection(CT_StepInitializeData& dataInit);
  ~SF_StepReversePipeModelCorrection();
  QString getStepDescription() const;
  QString getStepDetailledDescription() const;
  QString getStepURL() const;
  CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData& dataInit);
  QStringList getStepRISCitations() const;

protected:
  void createInResultModelListProtected();
  void createOutResultModelListProtected();
  void adaptParametersToExpertLevel() {}
  void createParamList(CT_ResultGroup* out_result);
  virtual void createPreConfigurationDialog() {}
  virtual void createPostConfigurationDialog();
  void compute();
  QList<SF_ParamReversePipeModelCorrection> _paramList;
  void createPostConfigurationDialogCitationSecond(CT_StepConfigurableDialog* configDialog);

private:
  CT_AutoRenameModels m_outCloudItem;
  CT_AutoRenameModels _outCylinderGroup;
  CT_AutoRenameModels _outCylinders;
  CT_AutoRenameModels _outSFQSM;
  int toStringSFMethod();
  SF_CLoudToModelDistanceMethod toStringCMDMethod();

  double _minRadius = 0.005;
  int m_minPts = 5;
  double m_inlierDistance = 0.015;
  int m_ransacIterations = 1000;

  virtual void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
  virtual void createPostConfigurationDialogExpert(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
};

#endif // SF_STEPREVERSEPIPEMODELCORRECTION_H
