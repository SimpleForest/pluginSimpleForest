#ifndef SF_STEP_SPHEREFOLLOWING_BASIC_H
#define SF_STEP_SPHEREFOLLOWING_BASIC_H

#include "steps/segmentation/sf_segmentation_step.h"

class SF_Step_Spherefollowing_Basic: public SF_Segmentation_Step {
    Q_OBJECT

public:
    SF_Step_Spherefollowing_Basic(CT_StepInitializeData &data_init);
    ~SF_Step_Spherefollowing_Basic();
    QString getStepDescription() const;
    QString getStepDetailledDescription() const;
    QString getStepURL() const;
    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &data_init);
    QStringList getStepRISCitations() const;

protected:
    void createInResultModelListProtected();
    void createOutResultModelListProtected();
    void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *config_dialog);
    void createPostConfigurationDialogExpert(CT_StepConfigurableDialog *config_dialog);
    void adapt_parameters_to_expert_level();
    void createParamList(CT_ResultGroup * out_result);
    void compute();
    QList<SF_Param_Spherefollowing_Basic<SF_Point> > _param_list;

private:
    double _euclideanDistance = 0.03;
    double _sphereRadiusMultiplier = 2;
    double _minRadius = 0.07;
    double _sphereEpsilon = 0.035;
    double _ransacCircleInlierDistance = 0.03;
    int _minPtsCircle = 3;
    double _heightStartSphere = 0.05;
    double _voxelSize = 0.01;

    QString _low    = "low";
    QString _medium = "medium";
    QString _high   = "high";
    QString _choice = _medium;
};

#endif // SF_STEP_SPHEREFOLLOWING_BASIC_H
