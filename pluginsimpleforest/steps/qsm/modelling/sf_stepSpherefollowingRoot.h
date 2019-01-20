#ifndef SF_STEP_SPHEREFOLLOWING_BASIC_H
#define SF_STEP_SPHEREFOLLOWING_BASIC_H

#include "steps/segmentation/sf_AbstractStepSegmentation.h"

class SF_StepSpherefollowingRoot : public SF_AbstractStepSegmentation
{
  Q_OBJECT

public:
  SF_StepSpherefollowingRoot(CT_StepInitializeData& dataInit);
  ~SF_StepSpherefollowingRoot();
  QString getStepDescription() const;
  QString getStepDetailledDescription() const;
  QString getStepURL() const;
  CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData& dataInit);
  QStringList getStepRISCitations() const;

protected:
  void createInResultModelListProtected();
  void createOutResultModelListProtected();
  void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog* configDialog);
  void createPostConfigurationDialogExpert(CT_StepConfigurableDialog* configDialog);
  void adaptParametersToExpertLevel();
  void createPostConfigurationDialogCitationSecond(CT_StepConfigurableDialog* configDialog);
  void createParamList(CT_ResultGroup* out_result);
  void compute();
  QList<SF_ParamSpherefollowingBasic<SF_PointNormal>> _paramList;

private:
  QList<SF_ParamQSM<SF_PointNormal>> paramList();
  CT_AutoRenameModels m_outCloudItem;
  CT_AutoRenameModels _outCylinderGroup;
  CT_AutoRenameModels _outCylinders;
  int toStringSFMethod();
  int toStringCMDMethod();
  double _PP_voxelSize = 0.01;
  double _PP_euclideanClusteringDistance = 0.2;
  double _SF_OPT_euclideanClusteringDistance = 0.03;
  double _SF_OPT_sphereRadiusMultiplier = 2;
  double _SF_OPT_sphereEpsilon = 0.035;
  bool _SF_parameterAutoSearch = true;
  int _SF_RANSACIiterations = 100;
  double _SF_minRadiusGlobal = 0.04;
  double _SF_inlierDistance = 0.03;
  int _SF_minPtsGeometry = 3;
  double _SF_heightInitializationSlice = 0.05;
  int _CMD_robustPercentage = 100;
  int _CMD_fittingMethod = toStringCMDMethod();
  int _CMD_k = 5;
  int _CMD_numClstrs = 1;
  double _CMD_inlierDistance = 0.05;

  void configDialogGuruAddSphereFollowing(CT_StepConfigurableDialog* configDialog);
  void configDialogAddSphereFollowingHyperParameters(CT_StepConfigurableDialog* configDialog);
  void configDialogAddSphereFollowingOptimizableParameters(CT_StepConfigurableDialog* configDialog);
  void configDialogGuruAddPreProcessing(CT_StepConfigurableDialog* configDialog);
  void configDialogGuruAddGridSearchCloudToModelDistance(CT_StepConfigurableDialog* configDialog);
};

#endif // SF_STEP_SPHEREFOLLOWING_BASIC_H
