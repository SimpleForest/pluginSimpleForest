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

#ifndef SF_FITRANSACLINE_HPP
#define SF_FITRANSACLINE_HPP

#include "sf_fitransacline.h"

template<typename T>
std::vector<T>
SF_FitRansacLine<T>::x() const
{
  return m_x;
}

template<typename T>
std::vector<T>
SF_FitRansacLine<T>::y() const
{
  return m_y;
}

template<typename T>
void
SF_FitRansacLine<T>::setY(const std::vector<T>& y)
{
  m_y = y;
}

template<typename T>
void
SF_FitRansacLine<T>::setX(const std::vector<T>& x)
{
  m_x = x;
}

template<typename T>
SF_FitRansacLine<T>::SF_FitRansacLine()
{}

template<typename T>
void
SF_FitRansacLine<T>::compute()
{
  if (m_x.size() != m_y.size() || m_x.size() < 2) {
    throw("Point vectors not well formated in Ransac line fit.");
  }
  // we use a fixed seed here to assure results are reproducable.
  std::mt19937 rng(197666);
  std::uniform_int_distribution<std::mt19937::result_type> dist(0, m_x.size() - 1);
  size_t currentMaxNumberInliers = 0;
  for (size_t i = 0; i < m_iterations; i++) {
    size_t index1 = dist(rng);
    size_t index2 = dist(rng);
    while (index1 == index2) {
      index2 = dist(rng);
    }
    auto equation = getEquation(index1, index2);
    size_t currentNumberInliers = numberInliers(equation);
    if (currentNumberInliers > currentMaxNumberInliers && equation.first > 0.2 && equation.second < 0.8) {
      currentMaxNumberInliers = currentNumberInliers;
      m_equation = equation;
    }
  }
  if (currentMaxNumberInliers < m_minPts) {
    throw("RANSAC fit did not return enough inliers.");
  }
}

template<typename T>
void
SF_FitRansacLine<T>::setMinPts(const size_t& minPts)
{
  m_minPts = minPts;
}

template<typename T>
std::pair<T, T>
SF_FitRansacLine<T>::getEquation(size_t index1, size_t index2)
{
  T x1 = m_x[index1];
  T y1 = m_y[index1];
  T x2 = m_x[index2];
  T y2 = m_y[index2];
  if (x2 == x1) {
    return m_errorEquation;
  }
  T slope = (y2 - y1) / (x2 - x1);
  T intercept = y2 - (slope * x2);
  typename std::pair<T, T> result;
  result.first = slope;
  result.second = intercept;
  return result;
}

template<typename T>
std::pair<std::vector<T>, std::vector<T>>
SF_FitRansacLine<T>::inliers(const std::pair<T, T>& equation)
{
  std::vector<T> inliersX;
  std::vector<T> inliersY;
  for (size_t index = 0; index < m_x.size(); index++) {
    T x = m_x[index];
    T y = m_y[index];
    T yPredicted = x * equation.first + equation.second;
    if (std::abs(yPredicted - y) < m_inlierDistance) {
      inliersX.push_back(x);
      inliersY.push_back(y);
    }
  }
  typename std::pair<std::vector<T>, std::vector<T>> result;
  result.first = inliersX;
  result.second = inliersY;
  return result;
}

template<typename T>
std::pair<std::vector<T>, std::vector<T>>
SF_FitRansacLine<T>::inliers()
{
  return inliers(m_equation);
}

template<typename T>
size_t
SF_FitRansacLine<T>::numberInliers(const std::pair<T, T>& equation)
{
  size_t result = 0;
  for (size_t index = 0; index < m_x.size(); index++) {
    T x = m_x[index];
    T y = m_y[index];
    T yPredicted = x * equation.first + equation.second;
    if (std::abs(yPredicted - y) < m_inlierDistance) {
      ++result;
    }
  }
  return result;
}

template<typename T>
void
SF_FitRansacLine<T>::setInlierDistance(const T& inlierDistance)
{
  m_inlierDistance = inlierDistance;
}

template<typename T>
void
SF_FitRansacLine<T>::setIterations(const size_t& iterations)
{
  m_iterations = iterations;
}

template<typename T>
std::pair<T, T>
SF_FitRansacLine<T>::equation()
{
  return m_equation;
}

#endif // SF_FITRANSACLINE_HPP
