#ifndef SF_STEP_CUT_CLOUD_ABOVE_DTM_H
#define SF_STEP_CUT_CLOUD_ABOVE_DTM_H

#include "steps/param/sf_abstract_param.h"
#include "steps/filter/binary/sf_abstractFilterBinaryStep.h"
#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"

class SF_StepCutCloudAboveDTM:
        public SF_AbstractFilterBinaryStep {
    Q_OBJECT

public:
    SF_StepCutCloudAboveDTM(CT_StepInitializeData &data_init);
    ~SF_StepCutCloudAboveDTM();
    QString getStepDescription() const;
    QString getStepDetailledDescription() const;
    QString getStepURL() const;
    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &data_init);
    QStringList getStepRISCitations() const;

protected:
    QList<SF_ParamDTMHeight<pcl::PointXYZ> > _paramList;
    void createInResultModelListProtected();
    void createOutResultModelListProtected();
    virtual void createPreConfigurationDialog();
    void adaptParametersToExpertLevel();
    void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *config_dialog);
    void createPostConfigurationDialogExpert(CT_StepConfigurableDialog *config_dialog);
    void compute();
    virtual void writeLogger();

private:
    double _cutHeight = 0.03f;
    void writeOutputPerScence(CT_ResultGroup* outResult, size_t i);
    void writeOutput(CT_ResultGroup* outResult);
    void createParamList(CT_ResultGroup *outResult);
};


#endif // SF_STEP_CUT_CLOUD_ABOVE_DTM_H
