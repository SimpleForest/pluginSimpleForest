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
  void adaptParametersToExpertLevel();
  void createPostConfigurationDialogCitationSecond(CT_StepConfigurableDialog* configDialog);
  void createParamList(CT_ResultGroup* out_result);
  virtual void createPreConfigurationDialog();
  virtual void createPostConfigurationDialog();

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

  QString _PARAMETERS_1 = "100%";
  QString _PARAMETERS_3 = "75%; 100%; 150%";
  QString _PARAMETERS_5 = "50%; 75%; 100%; 150%; 200%";
  QString _PARAMETERS_7 = "50%; 75%; 90%; 100%; 140%; 190%; 250%";
  QString _PARAMETERS_9 = "45%; 60%; 75%; 90%; 100%; 140%; 180%; 240%; 300%";
  QString _PARAMETERS_11 = "40%; 50%; 65%; 80%; 90%; 100%; 140%; 190%; 240%; 300%; 400%";
  QString _PARAMETERS_CHOICE_SPHERE_RADIUS_MULTIPLIER = _PARAMETERS_5;
  QString _PARAMETERS_CHOICE_SPHERE_EPSILON = _PARAMETERS_5;
  QString _PARAMETERS_CHOICE_EUCLIDEAN_CLUSTERING_DISTANCE = _PARAMETERS_5;
  QStringList _PARAMETERS_LIST_SPHERE_RADIUS_MULTIPLIER;
  QStringList _PARAMETERS_LIST_SPHERE_EPSILON;
  QStringList _PARAMETERS_LIST_EUCLIDEAN_CLUSTERING_DISTANCE;

  std::vector<double> paramsStringToNumber(const QString& UISelection);

  void configDialogAddSphereFollowingHyperParameters(CT_StepConfigurableDialog* configDialog);
  void configDialogAddSphereFollowingOptimizableParameters(CT_StepConfigurableDialog* configDialog);
  void configDialogGuruAddPreProcessing(CT_StepConfigurableDialog* configDialog);
  void configDialogGuruAddGridSearchCloudToModelDistance(CT_StepConfigurableDialog* configDialog);

  virtual void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
  virtual void createPostConfigurationDialogExpert(CT_StepConfigurableDialog* configDialog) { configDialog = nullptr; }
};

#endif // SF_STEP_SPHEREFOLLOWING_BASIC_H
