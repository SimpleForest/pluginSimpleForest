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

#ifndef SF_PCAVALUES_H
#define SF_PCAVALUES_H

struct SF_PCAValues {
  Eigen::Vector3f lambdas;
  inline void check_valid() {
    if (lambdas[0] > lambdas[1] || lambdas[0] > lambdas[2] ||
        lambdas[1] > lambdas[2]) {
      throw std::runtime_error("Lambdas have not been computed during PCA.");
    }
  }

  inline float getLambda(const int i) {
    if (lambdas[i] != 0) {
      return (lambdas[i] / (lambdas[0] + lambdas[1] + lambdas[2]));
    }
    return 0;
  }

  inline float getLambda1() {
    check_valid();
    return getLambda(0);
  }

  inline float getLambda2() {
    check_valid();
    return getLambda(1);
  }

  inline float getLambda3() {
    check_valid();
    return getLambda(2);
  }

  Eigen::Matrix3f vectors;

  inline Eigen::Vector3f getVector1() {
    check_valid();
    Eigen::Vector3f vec = vectors.col(0);
    vec.normalize();
    return vec;
  }

  inline Eigen::Vector3f getVector2() {
    check_valid();
    Eigen::Vector3f vec = vectors.col(1);
    vec.normalize();
    return vec;
  }

  inline Eigen::Vector3f getVector3() {
    check_valid();
    Eigen::Vector3f vec = vectors.col(2);
    vec.normalize();
    return vec;
  }
};
#endif // SF_PCAVALUES_H
