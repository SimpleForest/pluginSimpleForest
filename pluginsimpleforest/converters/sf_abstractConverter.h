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
#ifndef SF_ABSTRACT_CONVERTER_H
#define SF_ABSTRACT_CONVERTER_H

#include "ct_cloudindex/tools/ct_cloudindexstdvectortmethodimpl.h"
#include "ct_iterator/ct_pointiterator.h"
#include "ct_pointcloudindex/ct_pointcloudindexvector.h"
#include <ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h>

/**
 * @brief The SF_AbstractConverter class Abstract class for conversion
 * of CT structs to PCL or SimpleForest structs
 */
class SF_AbstractConverter
{
public:
  /**
   * @brief SF_AbstractConverter Standard constructor
   */
  SF_AbstractConverter();
  void setItemCpyCloudInDeprecated(const CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in);
  /**
   * @brief translation Getter for \ref m_translation.
   * @return \ref m_translation
   */
  Eigen::Vector3d translation() const;
  /**
   * @brief compute virtual method to be implemented
   */
  virtual void compute() = 0;

protected:
  /**
   * @brief m_translation The translation of \ref m_itemCpyCloudIn T towards the
   * origin (0,0,0).
   */
  Eigen::Vector3d m_translation;
  /**
   * @brief m_itemCpyCloudIn The CT input cloud.
   */
  const CT_AbstractItemDrawableWithPointCloud* m_itemCpyCloudIn;
  /**
   * @brief computeTranslationToOrigin Computes the \ref m_translation.
   */
  virtual void computeTranslationToOrigin();
};

#endif // SF_ABSTRACT_CONVERTER_H
