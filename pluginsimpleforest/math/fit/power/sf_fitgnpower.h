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

#ifndef SF_FITGNPOWER_H
#define SF_FITGNPOWER_H

#include "pcl/sf_math.h"
#include <random>

template<typename T>
class SF_FitGNPower
{
  bool m_fitWithIntercept;
  const double EULER = std::exp(1.0);
  T m_a;
  T m_b;
  T m_c;
  T m_inlierDistance;
  size_t m_ransacIterations;
  size_t m_gaussNewtonIterations;
  size_t m_minPts;
  std::vector<T> m_x;
  std::vector<T> m_y;

  std::vector<T> log(const std::vector<T>& vector);
  std::vector<T> exp(const std::vector<T>& vector);
  Eigen::MatrixXd getJacobian(const std::vector<T>& xVec);
  Eigen::MatrixXd getResiduals(const std::vector<T>& xVec, const std::vector<T>& yVec);
  void initializeParameters();

public:
  SF_FitGNPower();
  void setY(const std::vector<T>& y);
  void setX(const std::vector<T>& x);
  void setInlierDistance(const T& inlierDistance);
  void compute();
  void setMinPts(const size_t& minPts);
  void setRansacIterations(const size_t& ransacIterations);
  void setGaussNewtonIterations(const size_t& gaussNewtonIterations);
  void setFitWithIntercept(bool fitWithIntercept);
  T a() const;
  T b() const;
  T c() const;
};

#include "math/fit/power/sf_fitgnpower.hpp"

#endif // SF_FITGNPOWER_H
