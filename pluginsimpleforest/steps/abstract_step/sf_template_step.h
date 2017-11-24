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
#ifndef SF_TEMPLATE_STEP_H
#define SF_TEMPLATE_STEP_H


#include <steps/abstract_param/sf_abstract_param.h>
#include "ct_step/abstract/ct_abstractstep.h"

#include <QObject>
#include <QStringList>

//class SF_Template_Step: public CT_AbstractStep
//{
//public:
//    Q_OBJECT

//    SF_Template_Step(CT_StepInitializeData &data_ini);

//    ~SF_Template_Step();

//    virtual QString getStepDescription() const;

//    virtual QString getStepDetailledDescription() const;

//    virtual QString getStepURL() const;

//    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &data_init);

//    QStringList getPluginRISCitationList() const;

//protected:

//    void createInResultModelListProtected();

//    void createPostConfigurationDialog();

//    void createOutResultModelListProtected();

//    void compute();
//};

#endif // SF_TEMPLATE_STEP_H
