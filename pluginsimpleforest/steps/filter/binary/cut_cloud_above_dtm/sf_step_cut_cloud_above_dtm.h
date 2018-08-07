#ifndef SF_STEP_CUT_CLOUD_ABOVE_DTM_H
#define SF_STEP_CUT_CLOUD_ABOVE_DTM_H

#include "steps/param/sf_abstract_param.h"
#include "steps/filter/binary/sf_abstract_filter_binary_step.h"
#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"

class SF_Step_Cut_Cloud_Above_DTM:  public SF_Abstract_Filter_Binary_Step {
    Q_OBJECT

public:
    SF_Step_Cut_Cloud_Above_DTM(CT_StepInitializeData &data_init);
    ~SF_Step_Cut_Cloud_Above_DTM();
    QString getStepDescription() const;
    QString getStepDetailledDescription() const;
    QString getStepURL() const;
    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &data_init);
    QStringList getStepRISCitations() const;

protected:
    QList<SF_Param_DTM_Height<pcl::PointXYZ> > _param_list;
    void createInResultModelListProtected();
    void createOutResultModelListProtected();
    virtual void createPreConfigurationDialog();
    void adapt_parameters_to_expert_level();
    void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *config_dialog);
    void createPostConfigurationDialogExpert(CT_StepConfigurableDialog *config_dialog);
    void compute();
    virtual void write_logger();

private:
    double _cutHeight = 0.03f;
    void write_output_per_scence(CT_ResultGroup* out_result, size_t i);
    void write_output(CT_ResultGroup* out_result);
    void create_param_list(CT_ResultGroup *out_result);
};


#endif // SF_STEP_CUT_CLOUD_ABOVE_DTM_H
