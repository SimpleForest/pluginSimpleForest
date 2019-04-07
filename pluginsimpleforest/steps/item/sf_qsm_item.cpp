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

#include "sf_qsm_item.h"

SF_QSM_Item::SF_QSM_Item() {}

SF_QSM_Item::SF_QSM_Item(const CT_OutAbstractSingularItemModel* model,
                         const CT_AbstractResult* result,
                         std::shared_ptr<SF_ModelQSM> qsm)
  : CT_AbstractItemDrawableWithoutPointCloud(model, result)
{
  _qsm = qsm;
}

SF_QSM_Item::SF_QSM_Item(const QString& modelName, const CT_AbstractResult* result, std::shared_ptr<SF_ModelQSM> qsm)
  : CT_AbstractItemDrawableWithoutPointCloud(modelName, result)
{
  _qsm = qsm;
}

CT_AbstractItemDrawable*
SF_QSM_Item::copy(const CT_OutAbstractItemModel* model, const CT_AbstractResult* result, CT_ResultCopyModeList copyModeList)
{
  SF_QSM_Item* ref = new SF_QSM_Item((const CT_OutAbstractSingularItemModel*)model, result, _qsm);
  ref->setAlternativeDrawManager(getAlternativeDrawManager());
  return ref;
}

CT_AbstractItemDrawable*
SF_QSM_Item::copy(const QString& modelName, const CT_AbstractResult* result, CT_ResultCopyModeList copyModeList)
{
  SF_QSM_Item* ref = new SF_QSM_Item(modelName, result, _qsm);
  ref->setAlternativeDrawManager(getAlternativeDrawManager());
  return ref;
}

QString
SF_QSM_Item::getTreeID() const
{
  return _id.completeName();
}

std::shared_ptr<SF_ModelQSM>
SF_QSM_Item::getQsm() const
{
  return _qsm;
}
