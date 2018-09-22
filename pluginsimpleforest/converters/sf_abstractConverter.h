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

#include "ct_pointcloudindex/ct_pointcloudindexvector.h"
#include "ct_cloudindex/tools/ct_cloudindexstdvectortmethodimpl.h"
#include "ct_iterator/ct_pointiterator.h"
#include <ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h>

class SF_AbstractConverter
{
public:
    SF_AbstractConverter();
    void setItemCpyCloudIn(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in);
    void setItemCpyCloudInVector(const std::vector<CT_AbstractItemDrawableWithPointCloud *> itemCpy_cloud_inVector);

    Eigen::Vector3d getCenterOfMass() const;
    void setCenterOfMass(const Eigen::Vector3d &centerOfMass);

protected:
    void sumVector(CT_PointIterator &it);
    /**
     * @brief _centerOfMass The translation vector pointing from the origin to the original cloud center of mass
     */
    Eigen::Vector3d _centerOfMass;
    /**
     * @brief _itemCpyCloudIn The input CT cloud
     */
    const CT_AbstractItemDrawableWithPointCloud * _itemCpyCloudIn;
    /**
     * @brief computeTranslationToOrigin Computes the @see _centerOfMass
     */
    virtual void computeTranslationToOrigin();
    virtual void reset()=0;
    virtual void compute()=0;
    void computeCenterOfMass(size_t size, const CT_AbstractPointCloudIndex* index);

private:
    void normalizeSumVectorBySize(size_t size);
    void addPointVec(CT_PointIterator &it);
};

#endif // SF_ABSTRACT_CONVERTER_H
