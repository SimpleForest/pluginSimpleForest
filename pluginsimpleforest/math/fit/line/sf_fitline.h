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

#ifndef SF_FITLINE_H
#define SF_FITLINE_H

#include "pcl/sf_math.h"
#include <random>

template<typename T>
class SF_FitLine
{
  std::pair<T, T> m_equation;
  std::pair<T, T> m_errorEquation = std::make_pair<T, T>(0, std::numeric_limits<T>::min());
  std::vector<T> m_x;
  std::vector<T> m_y;

public:
  SF_FitLine();
  std::vector<T> x() const;
  std::vector<T> y() const;
  void setY(const std::vector<T>& y);
  void setX(const std::vector<T>& x);
  void compute();
  std::pair<T, T> equation();
  void setMinPts(const size_t& minPts);
};

#include "math/fit/line/sf_fitline.hpp"

#endif // SF_FITLINE_H
