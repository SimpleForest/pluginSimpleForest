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

#include <algorithm>

#include "sf_interpolation.h"

float
SF_Interpolation::interpolateMedian(const std::vector<float>& values)
{
  std::vector<float> v = values;
  size_t n = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + n, v.end());
  return v[n];
}

float
SF_Interpolation::interpolateIDW(const std::vector<float>& values, const std::vector<float>& distances)
{
  std::vector<float> inverseDistances;
  for (size_t i = 0; i < distances.size(); i++) {
    if (distances[i] == 0)
      return values[i];
    float weightedDist = 1 / distances[i];
    inverseDistances.push_back(weightedDist);
  }
  float normalizingFac = 0;
  for (size_t i = 0; i < distances.size(); i++) {
    normalizingFac += inverseDistances[i];
  }
  float interpolatedValue = 0;
  for (size_t i = 0; i < distances.size(); i++) {
    interpolatedValue += values[i] * inverseDistances[i] / normalizingFac;
  }
  return interpolatedValue;
}

SF_Interpolation::SF_Interpolation() {}
