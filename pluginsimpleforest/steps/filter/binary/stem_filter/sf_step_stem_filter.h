/****************************************************************************

 Copyright (C) 2017-2018 Jan Hackenberg, free software developer
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
#ifndef SF_STEP_STEM_FILTER_H
#define SF_STEP_STEM_FILTER_H

#include "steps/filter/binary/sf_abstract_filter_binary_step.h"

#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include <steps/param/sf_abstract_param.h>

class SF_Step_Stem_Filter:  public SF_Abstract_Filter_Binary_Step {
    Q_OBJECT

public:

    SF_Step_Stem_Filter(CT_StepInitializeData &data_init);

    ~SF_Step_Stem_Filter();

    QString getStepDescription() const;

    QString getStepDetailledDescription() const;

    QString getStepURL() const;

    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &data_init);

    QStringList getStepRISCitations() const;


protected:

    void createInResultModelListProtected();

    void createOutResultModelListProtected();

    void adapt_parameters_to_expert_level();

    void createPostConfigurationDialogBeginner(CT_StepConfigurableDialog *config_dialog);

    void createPostConfigurationDialogExpert(CT_StepConfigurableDialog *config_dialog);

    void compute();

    QList<SF_Param_Stem_Filter<SF_Point_N> > _param_list;

    virtual void write_logger();

private:

    QString _less = "less";

    QString _intermediate = "intermediate";

    QString _many         = "many";

    QString _choice       = _intermediate;


    double _x = 0;
    double _y = 0;
    double _z = 1;
    double _angle = 25;
    double _radius_growth_direction = 0.45;
    double _radius_normal = 0.04;
    double _voxel_size = 0.015;
    double _size_output = 2;

    void write_output_per_scence(CT_ResultGroup* out_result, size_t i);

    void write_output(CT_ResultGroup* out_result);

    void create_param_list(CT_ResultGroup *out_result);
};

#endif // SF_STEP_STEM_FILTER_H
