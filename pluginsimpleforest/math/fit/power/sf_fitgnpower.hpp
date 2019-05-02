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

#include "sf_fitgnpower.h"

#include "math/fit/line/sf_fitransacline.h"

#include <Eigen/QR>

template<typename T>
SF_FitGNPower<T>::SF_FitGNPower()
{}

template<typename T>
void
SF_FitGNPower<T>::setY(const std::vector<T>& y)
{
  m_y = y;
}

template<typename T>
void
SF_FitGNPower<T>::setFitWithIntercept(bool fitWithIntercept)
{
  m_fitWithIntercept = fitWithIntercept;
}

template<typename T>
T
SF_FitGNPower<T>::a() const
{
  return m_a;
}

template<typename T>
T
SF_FitGNPower<T>::b() const
{
  return m_b;
}

template<typename T>
T
SF_FitGNPower<T>::c() const
{
  return m_c;
}

template<typename T>
void
SF_FitGNPower<T>::setRansacIterations(const size_t& ransacIterations)
{
  m_ransacIterations = ransacIterations;
}

template<typename T>
void
SF_FitGNPower<T>::setX(const std::vector<T>& x)
{
  m_x = x;
}

template<typename T>
std::vector<T>
SF_FitGNPower<T>::log(const std::vector<T>& vector)
{
  std::vector<T> logarithms;
  std::transform(vector.cbegin(), vector.cend(), std::back_inserter(logarithms), [](const T& value) { return std::log(value); });
  return logarithms;
}

template<typename T>
std::vector<T>
SF_FitGNPower<T>::exp(const std::vector<T>& vector)
{
  std::vector<T> exponents;
  std::transform(vector.cbegin(), vector.cend(), std::back_inserter(exponents), [](const T& value) { return std::exp(value); });
  return exponents;
}

template<typename T>
Eigen::MatrixXd
SF_FitGNPower<T>::getJacobian(const std::vector<T>& xVec)
{
  Eigen::MatrixXd jacobian;
  if (m_fitWithIntercept) {
    jacobian.resize(xVec.size(), 3);
    for (size_t index = 0; index < xVec.size(); index++) {
      T x = xVec.at(index);
      jacobian(index, 0) = std::pow(x, m_b);
      jacobian(index, 1) = m_a * (std::log(x)) * std::pow(x, m_b);
      jacobian(index, 2) = 1;
    }
  } else {
    jacobian.resize(xVec.size(), 2);
    for (size_t index = 0; index < xVec.size(); index++) {
      T x = xVec.at(index);
      jacobian(index, 0) = std::pow(x, m_b);
      jacobian(index, 1) = m_a * (std::log(x)) * std::pow(x, m_b);
    }
  }
  return jacobian;
}

template<typename T>
void
SF_FitGNPower<T>::initializeParameters()
{
  std::vector<T> xLog = log(m_x);
  std::vector<T> yLog = log(m_y);
  SF_FitRansacLine<T> rl;
  rl.setX(std::move(xLog));
  rl.setY(std::move(yLog));
  rl.setInlierDistance(m_inlierDistance);
  rl.setIterations(m_ransacIterations);
  rl.setMinPts(m_minPts);
  try {
    rl.compute();
  } catch (const std::exception& e) {
    std::string eStr = std::string(e.what());
    eStr.append("RANSAC line fit failed for powerfit parameter initialization.");
    throw(eStr);
  } catch (...) {
    throw("RANSAC line fit failed for powerfit parameter initialization.");
  }
  std::pair<T, T> line = rl.equation();
  std::pair<std::vector<T>, std::vector<T>> inliers = rl.inliers();
  m_a = std::exp(line.second);
  m_b = line.first;
  m_c = 0;
  m_x = exp(inliers.first);
  m_y = exp(inliers.second);
}

template<typename T>
void
SF_FitGNPower<T>::setMinPts(const size_t& minPts)
{
  m_minPts = minPts;
}

template<typename T>
void
SF_FitGNPower<T>::compute()
{
  try {
    initializeParameters();
  } catch (const std::exception& e) {
    std::string eStr = std::string(e.what());
    std::cout << eStr << std::endl;
    throw(eStr);
  } catch (...) {
    throw("RANSAC line fit failed for powerfit parameter initialization.");
  }
  size_t iteration = 0;
  while (iteration++ < m_gaussNewtonIterations) {
    Eigen::MatrixXd jacobian = getJacobian(m_x);
    Eigen::MatrixXd residuals = getResiduals(m_x, m_y);
    Eigen::FullPivHouseholderQR<Eigen::MatrixXd> jacobianSolver(jacobian);
    Eigen::MatrixXd deltas = jacobianSolver.solve(residuals);
    m_a += deltas(0, 0);
    m_b += deltas(1, 0);
    if (m_fitWithIntercept) {
      m_c += deltas(2, 0);
    }
  }
}

template<typename T>
Eigen::MatrixXd
SF_FitGNPower<T>::getResiduals(const std::vector<T>& xVec, const std::vector<T>& yVec)
{
  Eigen::MatrixXd residuals;
  residuals.resize(xVec.size(), 1);
  if (m_fitWithIntercept) {
    for (size_t index = 0; index < xVec.size(); index++) {
      T x = xVec.at(index);
      T y = yVec.at(index);
      //      residuals(index, 0) = -((m_a * std::pow((1 - std::pow(EULER, (-m_b * x))), m_c)) - y);
      residuals(index, 0) = -((m_a * std::pow(x, m_b)) + m_c - y);
    }
  } else {
    for (size_t index = 0; index < xVec.size(); index++) {
      T x = xVec.at(index);
      T y = yVec.at(index);
      residuals(index, 0) = -((m_a * std::pow(x, m_b)) - y);
    }
  }
  return residuals;
}

template<typename T>
void
SF_FitGNPower<T>::setInlierDistance(const T& inlierDistance)
{
  m_inlierDistance = inlierDistance;
}

template<typename T>
void
SF_FitGNPower<T>::setGaussNewtonIterations(const size_t& gaussNewtonIterations)
{
  m_gaussNewtonIterations = gaussNewtonIterations;
}
