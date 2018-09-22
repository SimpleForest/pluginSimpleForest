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

#include "sf_abstractConverter.h"

SF_AbstractConverter::SF_AbstractConverter() {

}

void SF_AbstractConverter::addPointVec(CT_PointIterator &it) {
    const CT_Point &internalPoint = it.next().currentPoint();
    _centerOfMass[0] += internalPoint(0);
    _centerOfMass[1] += internalPoint(1);
    _centerOfMass[2] += internalPoint(2);
}

void SF_AbstractConverter::sumVector(CT_PointIterator &it) {
    while(it.hasNext()) {
        addPointVec(it);
    }
}

Eigen::Vector3d SF_AbstractConverter::getCenterOfMass() const {
    return _centerOfMass;
}

void SF_AbstractConverter::setCenterOfMass(const Eigen::Vector3d &centerOfMass) {
    _centerOfMass = centerOfMass;
}

void SF_AbstractConverter::setItemCpyCloudIn(const CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in) {
    _itemCpyCloudIn = itemCpy_cloud_in;
}

void SF_AbstractConverter::normalizeSumVectorBySize(size_t size) {
    _centerOfMass[0]/=size;
    _centerOfMass[1]/=size;
    _centerOfMass[2]/=size;
}

void SF_AbstractConverter::computeCenterOfMass(size_t size, const CT_AbstractPointCloudIndex* index) {
    assert(size > 0);
    CT_PointIterator it(index);
    sumVector(it);
    normalizeSumVectorBySize(size);
}

void SF_AbstractConverter::computeTranslationToOrigin() {
    const CT_AbstractPointCloudIndex* index = _itemCpyCloudIn->getPointCloudIndex();
    assert(index->size() > 0);
    _centerOfMass[0] = _itemCpyCloudIn->minX() + 0.5*(_itemCpyCloudIn->maxX()-_itemCpyCloudIn->minX());
    _centerOfMass[1] = _itemCpyCloudIn->minY() + 0.5*(_itemCpyCloudIn->maxY()-_itemCpyCloudIn->minY());
    _centerOfMass[2] = _itemCpyCloudIn->minZ() + 0.5*(_itemCpyCloudIn->maxZ()-_itemCpyCloudIn->minZ());
}
