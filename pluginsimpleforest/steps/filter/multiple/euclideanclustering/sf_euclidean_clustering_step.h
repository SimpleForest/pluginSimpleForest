#ifndef SF_EUCLIDEAN_CLUSTERING_STEP_H
#define SF_EUCLIDEAN_CLUSTERING_STEP_H

#include "steps/filter/multiple/sf_abstract_filter_multiple_step.h"

class SF_Euclidean_Clustering_Step: public SF_Abstract_Filter_Multiple_Step
{
    Q_OBJECT

public:
    SF_Euclidean_Clustering_Step(CT_StepInitializeData &data_init);
    ~SF_Euclidean_Clustering_Step();
    QString getStepDescription() const;
    QString getStepDetailledDescription() const;
    QString getStepURL() const;
    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &data_init);
    QStringList getStepRISCitations() const;

protected:

    void createInResultModelListProtected();
    void createPostConfigurationDialog();
    void createOutResultModelListProtected();
    void createPreConfigurationDialog(){}
    void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *config_dialog){}
    void createPostConfigurationDialogExpert(CT_StepConfigurableDialog *config_dialog){}
    void adapt_parameters_to_expert_level(){}
    void createParamList(CT_ResultGroup * out_result);
    void compute();

    void mergeClustersToPCLCloud(std::vector<size_t> indices, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCL, CT_ResultGroupIterator out_res_it);
    
private:
    double _voxelSize = 0.04;
    double _euclideanDistance = 0.1;
    int    _minPts = 1;
};

#endif // SF_EUCLIDEAN_CLUSTERING_STEP_H
