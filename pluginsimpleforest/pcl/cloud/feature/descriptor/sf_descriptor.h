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
#ifndef SF_DESCRIPTOR_H
#define SF_DESCRIPTOR_H

#include "pcl/cloud/feature/sf_abstractFeature.h"

template<typename PointType>
class SF_Descriptor
{
private:
  float m_k = 2;
  float m_sd;
  float m_mean;
  SF_CloudNormal::Ptr _cloudIn;

public:
  SF_Descriptor(typename pcl::PointCloud<PointType>::Ptr cloudIn);
  virtual void computeFeatures();
  void setParameters(int k);
  float sd() const;
  float mean() const;
};

#include "sf_descriptor.hpp"

#endif // DESCRIPTOR_H
