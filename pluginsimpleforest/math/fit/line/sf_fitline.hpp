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

#ifndef SF_FITLINE_HPP
#define SF_FITLINE_HPP

#include "math/fit/line/sf_fitline.h"

template<typename T>
std::vector<T>
SF_FitLine<T>::x() const
{
  return m_x;
}

template<typename T>
std::vector<T>
SF_FitLine<T>::y() const
{
  return m_y;
}

template<typename T>
void
SF_FitLine<T>::setY(const std::vector<T>& y)
{
  m_y = y;
}

template<typename T>
void
SF_FitLine<T>::setX(const std::vector<T>& x)
{
  m_x = x;
}

template<typename T>
SF_FitLine<T>::SF_FitLine()
{}

template<typename T>
void
SF_FitLine<T>::compute()
{
  // see https://stackoverflow.com/questions/11449617/how-to-fit-the-2d-scatter-data-with-a-line-with-c
  if (m_x.size() != m_y.size() || m_x.size() < 2) {
    throw("Point vectors not well formated in Ransac line fit.");
  }
  int nPoints = m_y.size();
  if( nPoints < 2 ) {
    // Fail: infinitely many lines passing through this single point
    return;
  }
  double sumX=0, sumY=0, sumXY=0, sumX2=0;
  for(int i=0; i<nPoints; i++) {
    sumX += m_x[i];
    sumY += m_y[i];
    sumXY += m_x[i] * m_y[i];
    sumX2 += m_x[i] * m_x[i];
  }
  double xMean = sumX / nPoints;
  double yMean = sumY / nPoints;
  double denominator = sumX2 - sumX * xMean;
  // You can tune the eps (1e-7) below for your specific task
  if( std::fabs(denominator) < 1e-7 ) {
    // Fail: it seems a vertical line
    return;
  }
  m_equation.first = (sumXY - sumX * yMean) / denominator;
  m_equation.second = yMean - m_equation.first * xMean;
}

template<typename T>
std::pair<T, T>
SF_FitLine<T>::equation()
{
  return m_equation;
}


#endif // SF_FITLINE_HPP
