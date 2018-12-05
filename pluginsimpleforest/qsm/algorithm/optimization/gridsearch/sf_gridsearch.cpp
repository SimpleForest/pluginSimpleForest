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

#include <cmath>

#include "sf_gridsearch.h"

const int SF_GridSearch::gridSize(const int nDim, const int res) {
  int size = static_cast<int>(std::pow(res, nDim));
  return size;
}

void SF_GridSearch::restrictSearchSpace() {
  int max = _params._maxNumberSearches;
  int size = gridSize(_params._nDimensions, _params._resolution);
  while (size > max) {
    _params._resolution = _params._resolution - 1;
    size = gridSize(_params._nDimensions, _params._resolution);
  }
}

SF_GridSearch::SF_GridSearch() {}

void SF_GridSearch::compute() {}
