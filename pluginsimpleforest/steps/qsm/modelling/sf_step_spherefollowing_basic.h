#ifndef SF_STEP_SPHEREFOLLOWING_BASIC_H
#define SF_STEP_SPHEREFOLLOWING_BASIC_H

#include "steps/segmentation/sf_segmentation_step.h"

class SF_StepSpherefollowingRoot: public SF_SegmentationStep {
    Q_OBJECT

public:
    SF_StepSpherefollowingRoot(CT_StepInitializeData &dataInit);
    ~SF_StepSpherefollowingRoot();
    QString getStepDescription() const;
    QString getStepDetailledDescription() const;
    QString getStepURL() const;
    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &dataInit);
    QStringList getStepRISCitations() const;

    void configDialogGuruAddSphereFollowingGridSearch(CT_StepConfigurableDialog *configDialog);

protected:
    void createInResultModelListProtected();
    void createOutResultModelListProtected();
    void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *configDialog);
    void createPostConfigurationDialogExpert(CT_StepConfigurableDialog *configDialog);
    void adaptParametersToExpertLevel();
    void createPostConfigurationDialogCitationSecond(CT_StepConfigurableDialog *configDialog);
    void createParamList(CT_ResultGroup * out_result);
    void compute();
    QList<SF_ParamSpherefollowingBasic<SF_PointNormal> > _paramList;

private:
    int toStringSFMethod();
    int toStringCMDMethod();
    double _PP_voxelSize                       = 0.01;
    double _PP_euclideanClusteringDistance     = 0.2;
    double _SF_OPT_euclideanClusteringDistance = 0.03;
    double _SF_OPT_sphereRadiusMultiplier      = 2;
    double _SF_OPT_minRadius                   = 0.07;
    double _SF_OPT_sphereEpsilon               = 0.035;
    bool   _SF_parameterAutoSearch             = true;
    int    _SF_RANSACIiterations                = 100;
    double _SF_minRadiusGlobal                 = 0.04;
    double _SF_inlierDistance                  = 0.03;
    int    _SF_minPtsGeometry                  = 3;
    double _SF_heightInitializationSlice       = 0.05;
    int    _CMD_robustPercentage               = 100;
    int    _CMD_fittingMethod                  = toStringCMDMethod();
    int    _CMD_k                              = 5;
    double _CMD_inlierDistance                 = 0.05;
    bool   _GS_doGridSearch                    = true;
    int    _GS_nDimensions                     = 4;
    int    _GS_resolution                      = 3;
    int    _GS_maximizeSearchSpace             = 81;


    void configDialogGuruAddSphereFollowing(CT_StepConfigurableDialog *configDialog);
    void configDialogAddSphereFollowingHyperParameters(CT_StepConfigurableDialog *configDialog);
    void configDialogAddSphereFollowingOptimizableParameters(CT_StepConfigurableDialog *configDialog);
    void configDialogGuruAddPreProcessing(CT_StepConfigurableDialog *configDialog);
    void configDialogGuruAddGridSearchCloudToModelDistance(CT_StepConfigurableDialog *configDialog);
};

#endif // SF_STEP_SPHEREFOLLOWING_BASIC_H
