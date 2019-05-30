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

#ifndef SF_CLOUD_TO_MODEL_DISTANCE_PARAMETERS_H
#define SF_CLOUD_TO_MODEL_DISTANCE_PARAMETERS_H

enum SF_CLoudToModelDistanceMethod
{
  ZEROMOMENTUMORDER,
  FIRSTMOMENTUMORDERMSAC,
  FIRSTMOMENTUMORDER,
  SECONDMOMENTUMORDERMSAC,
  SECONDMOMENTUMORDER,
  GROWTHDISTANCE
};

struct SF_CloudToModelDistanceParameters
{
  SF_CLoudToModelDistanceMethod _method = SF_CLoudToModelDistanceMethod::SECONDMOMENTUMORDERMSAC;
  float _inlierDistance = 0.05f;
  int _k = 5;
  SF_CloudToModelDistanceParameters() {}
  SF_CloudToModelDistanceParameters(SF_CLoudToModelDistanceMethod& method, float inlierDistance, int k)
  {
    _method = method;
    _inlierDistance = inlierDistance;
    _k = k;
  }
};

#endif // SF_CLOUD_TO_MODEL_DISTANCE_PARAMETERS_H
