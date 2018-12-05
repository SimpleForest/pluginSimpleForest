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
#ifndef SF_STEM_FILTER_H
#define SF_STEM_FILTER_H

#include <pcl/cloud/filter/binary/sf_abstractBinaryFilter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>

#include "pcl/cloud/feature/pca/sf_pca.h"
#include "pcl/sf_math.h"

#include "ct_itemdrawable/ct_pointsattributescolor.h"

template <typename PointType>
class SF_StemFilter : public Sf_AbstractBinaryFilter<PointType> {
  SF_ParamStemFilter<PointType> _params;
  void transferStem(
      const SF_ParamStemFilter<PointType> &params,
      typename pcl::PointCloud<PointType>::Ptr down_scaled_cloud,
      typename pcl::PointCloud<PointType>::Ptr cloud_with_growth_direction);
  CT_ColorCloudStdVector *_colors;

public:
  SF_StemFilter();
  virtual void compute();
  void setParams(SF_ParamStemFilter<PointType> &params);
  CT_ColorCloudStdVector *colors() const;
};

#include "pcl/cloud/filter/binary/stem/sf_stemFilter.hpp"
#endif // SF_STEM_FILTER_H
