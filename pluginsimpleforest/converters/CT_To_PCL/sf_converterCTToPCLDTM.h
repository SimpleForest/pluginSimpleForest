/****************************************************************************

 Copyright (C) 2017-2017 Jan Hackenberg, free software developer
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

#ifndef SF_CONVERTER_CT_TO_PCL_DTM_H
#define SF_CONVERTER_CT_TO_PCL_DTM_H

#include "converters/sf_abstractConverter.h"
#include "ct_itemdrawable/ct_image2d.h"
#include "plot/DTM/sf_modelDTM.h"

class SF_ConverterCTToPCLDTM : public SF_AbstractConverter
{
  std::shared_ptr<SF_ModelDTM> _dtmPCL;
  CT_Image2D<float>* _dtmCT;
  virtual void computeTranslationToOrigin();
  virtual void compute();

public:
  SF_ConverterCTToPCLDTM(CT_Image2D<float>* dtmCT);
  SF_ConverterCTToPCLDTM(Eigen::Vector3d centerOfMass, CT_Image2D<float>* dtmCT);
  std::shared_ptr<SF_ModelDTM> dtmPCL() const;
};

#endif // SF_CONVERTER_CT_TO_PCL_DTM_H
