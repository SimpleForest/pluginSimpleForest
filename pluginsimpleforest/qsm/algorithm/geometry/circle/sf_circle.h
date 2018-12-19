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

#ifndef SF_CIRCLE_H
#define SF_CIRCLE_H

#include "pcl/common/geometry.h"
#include "pcl/sf_math.h"
#include "pcl/sf_point.h"
#include "steps/param/sf_paramAllSteps.h"

#include <utility>

template <typename PointType> class SF_Circle {
  typename pcl::PointCloud<PointType>::Ptr m_cloudIn;
  const std::vector<int> m_indices;
  const SF_ParamSpherefollowingBasic<PointType> &m_params;
  const size_t m_paramIndex;
  pcl::ModelCoefficients m_coeff;
  pcl::ModelCoefficients cirlceMedianWithIndices();
  pcl::ModelCoefficients circleMedianWithSubCloud();
  pcl::ModelCoefficients cirlceSACModelWithIndices();
  pcl::ModelCoefficients cirlceSACModelWithSubCloud();
  void setParam(pcl::SACSegmentationFromNormals<PointType, PointType> &seg);
  void chooseModel(const pcl::ModelCoefficients &circleMedian,
                   const pcl::ModelCoefficients &circleSACModel);

public:
  SF_Circle(typename pcl::PointCloud<PointType>::Ptr cloudIn,
            const std::vector<int> &indices,
            const SF_ParamSpherefollowingBasic<PointType> params, size_t paramIndex);
  SF_Circle(typename pcl::PointCloud<PointType>::Ptr cloudIn,
            const SF_ParamSpherefollowingBasic<PointType> params, size_t paramIndex);
  pcl::ModelCoefficients coeff() const;
};

#include "sf_circle.hpp"

#endif // SF_CIRCLE_H
