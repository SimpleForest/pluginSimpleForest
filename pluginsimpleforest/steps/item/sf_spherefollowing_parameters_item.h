/****************************************************************************

 Copyright (C) 2017-2019 Dr. Jan Hackenberg, free software developer
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

#ifndef SF_SPHEREFOLLOWING_PARAMETERS_ITEM_H
#define SF_SPHEREFOLLOWING_PARAMETERS_ITEM_H

#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithoutpointcloud.h"
#include "ct_itemdrawable/tools/drawmanager/ct_standardreferencepointdrawmanager.h"
#include "ct_tools/model/ct_autorenamemodels.h"

#include "steps/param/sf_paramAllSteps.h"

class SF_SphereFollowing_Parameters_Item : public CT_AbstractItemDrawableWithoutPointCloud
{
  Q_OBJECT
  CT_TYPE_IMPL_MACRO(SF_SphereFollowing_Parameters_Item, CT_AbstractItemDrawableWithoutPointCloud, SF_SphereFollowing_Parameters)
public:
  SF_SphereFollowing_Parameters_Item();
  SF_SphereFollowing_Parameters_Item(const CT_OutAbstractSingularItemModel* model,
                                     const CT_AbstractResult* result,
                                     SF_ParamSpherefollowingBasic<pcl::PointXYZINormal> params);
  SF_SphereFollowing_Parameters_Item(const QString& modelName,
                                     const CT_AbstractResult* result,
                                     SF_ParamSpherefollowingBasic<pcl::PointXYZINormal> params);
  virtual CT_AbstractItemDrawable* copy(const CT_OutAbstractItemModel* model,
                                        const CT_AbstractResult* result,
                                        CT_ResultCopyModeList copyModeList);
  virtual CT_AbstractItemDrawable* copy(const QString& modelName, const CT_AbstractResult* result, CT_ResultCopyModeList copyModeList);
  QString getParameterID() const;
  SF_ParamSpherefollowingBasic<pcl::PointXYZINormal> getParams() const;

private:
  CT_AutoRenameModels _id;
  SF_ParamSpherefollowingBasic<pcl::PointXYZINormal> _params;

  CT_DEFAULT_IA_BEGIN(SF_SphereFollowing_Parameters_Item)
  CT_DEFAULT_IA_V3(SF_SphereFollowing_Parameters_Item,
                   CT_AbstractCategory::staticInitDataId(),
                   &SF_SphereFollowing_Parameters_Item::getParameterID,
                   QObject::tr("ID"),
                   "id")
  CT_DEFAULT_IA_V3(SF_SphereFollowing_Parameters_Item,
                   CT_AbstractCategory::staticInitDataId(),
                   &SF_SphereFollowing_Parameters_Item::getParameterID,
                   QObject::tr("ID"),
                   "id")
  CT_DEFAULT_IA_END(SF_SphereFollowing_Parameters_Item)
};

#endif // SF_SPHEREFOLLOWING_PARAMETERS_ITEM_H
