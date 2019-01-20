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

#ifndef SF_DOWNHILLSIMPLEX_H
#define SF_DOWNHILLSIMPLEX_H

#include "../../spherefollowing/sf_spherefollowing.h"

#include <gsl/gsl_blas.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_ieee_utils.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_min.h>
#include <gsl/gsl_multifit_nlin.h>
#include <gsl/gsl_multifit_nlinear.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_test.h>

double
downhillSimplex(const gsl_vector* v, void* params);

class SF_DownHillSimplex
{
  SF_ParamSpherefollowingBasic<SF_PointNormal> m_params;
  void serializeParams(std::uintptr_t* par);
  void serializeVec(gsl_vector* x, double fac);

public:
  SF_DownHillSimplex();
  void compute();
  SF_ParamSpherefollowingBasic<SF_PointNormal> params() const;
  void setParams(const SF_ParamSpherefollowingBasic<SF_PointNormal>& params);
};

#endif // SF_DOWNHILLSIMPLEX_H
