/****************************************************************************

 Copyright (C) 2017-2017 Jan Hackenberg, free software developer
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
#ifndef SF_STEP_STATISTICAL_OUTLIER_REMOVAL_H
#define SF_STEP_STATISTICAL_OUTLIER_REMOVAL_H

#include "steps/filter/binary/sf_abstract_filter_binary_step.h"

#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include <steps/param/sf_abstract_param.h>

//void compute_statistical_outlier_removal(SF_Param_Statistical_Outlier_Filter<SF_Point> & params)
//{
//    SF_Converter_CT_To_PCL<SF_Point> converter( params._itemCpy_cloud_in);
//    converter.compute();
//    params._cloud_in = converter.get_cloud_translated();
//    SF_Statistical_Outlier_Filter<SF_Point> filter (params._cloud_in);
//    filter.compute(params);
//    params._output_indices = filter.get_indices();
//}
//std::function<void(SF_Param_Statistical_Outlier_Filter<SF_Point>& )> compute_statistical_outlier_removal =
//        []( SF_Param_Statistical_Outlier_Filter<SF_Point> & params)
//   {
//    SF_Converter_CT_To_PCL<SF_Point> converter( params._itemCpy_cloud_in);
//    converter.compute();
//    params._cloud_in = converter.get_cloud_translated();
//    SF_Statistical_Outlier_Filter<SF_Point> filter (params._cloud_in);
//    filter.compute(params);
//    params._output_indices = filter.get_indices();
//   };



class SF_Step_Statistical_Outlier_Removal:  public SF_Abstract_Filter_Binary_Step
{
    Q_OBJECT

public:

    SF_Step_Statistical_Outlier_Removal(CT_StepInitializeData &data_init);

    ~SF_Step_Statistical_Outlier_Removal();

    QString getStepDescription() const;

    QString getStepDetailledDescription() const;

    QString getStepURL() const;

    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &data_init);

    QStringList getStepRISCitations() const;

protected:

    void createInResultModelListProtected();

    void createPostConfigurationDialog();

    void createOutResultModelListProtected();

    void compute();

    QList<SF_Param_Statistical_Outlier_Filter<SF_Point> > _param_list;

private:

    double _std_mult = 3.0;

    int _iterations = 5;

    int _k = 2;

    void write_output_per_scence(CT_ResultGroup* out_result, size_t i);

    void write_output(CT_ResultGroup* out_result);

    void create_param_list(CT_ResultGroup *out_result);

};

#endif // SF_STEP_STATISTICAL_OUTLIER_REMOVAL_H
