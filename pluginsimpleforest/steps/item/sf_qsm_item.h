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

#ifndef SF_QSM_ITEM_H
#define SF_QSM_ITEM_H

#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithoutpointcloud.h"
#include "ct_itemdrawable/tools/drawmanager/ct_standardreferencepointdrawmanager.h"
#include "ct_tools/model/ct_autorenamemodels.h"

#include "qsm/model/sf_modelQSM.h"

class SF_QSM_Item : public CT_AbstractItemDrawableWithoutPointCloud
{
  Q_OBJECT
  CT_TYPE_IMPL_MACRO(SF_QSM_Item, CT_AbstractItemDrawableWithoutPointCloud, SF_QSM)
public:
  SF_QSM_Item();
  SF_QSM_Item(const CT_OutAbstractSingularItemModel* model, const CT_AbstractResult* result, std::shared_ptr<SF_ModelQSM> qsm);
  SF_QSM_Item(const QString& modelName, const CT_AbstractResult* result, std::shared_ptr<SF_ModelQSM> qsm);
  virtual CT_AbstractItemDrawable* copy(const CT_OutAbstractItemModel* model,
                                        const CT_AbstractResult* result,
                                        CT_ResultCopyModeList copyModeList);
  virtual CT_AbstractItemDrawable* copy(const QString& modelName, const CT_AbstractResult* result, CT_ResultCopyModeList copyModeList);
  QString getTreeID() const;
  std::shared_ptr<SF_ModelQSM> getQsm() const;

private:
  CT_AutoRenameModels _id;
  std::shared_ptr<SF_ModelQSM> _qsm;

  CT_DEFAULT_IA_BEGIN(SF_QSM_Item)
  CT_DEFAULT_IA_V3(SF_QSM_Item, CT_AbstractCategory::staticInitDataId(), &SF_QSM_Item::getTreeID, QObject::tr("ID"), "id")
  CT_DEFAULT_IA_V3(SF_QSM_Item, CT_AbstractCategory::staticInitDataId(), &SF_QSM_Item::getTreeID, QObject::tr("ID"), "id")
  CT_DEFAULT_IA_END(SF_QSM_Item)
};

#endif // SF_QSM_ITEM_H
