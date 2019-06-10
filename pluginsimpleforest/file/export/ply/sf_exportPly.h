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

#ifndef SF_EXPORTPLY_H
#define SF_EXPORTPLY_H

#include "file/export/sf_abstractExport.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <random>

enum class SF_ExportPlyPolicy
{
  GROWTH_VOLUME = 0,
  GROWTH_LENGTH = 1,
  STEM = 2
};

class SF_ExportPly : public SF_AbstractExport
{
  int m_resolution;
  SF_ExportPlyPolicy m_exportPolicy = SF_ExportPlyPolicy::GROWTH_LENGTH;
  int getNumberOfPoints();
  int getNumberOfFaces();
  void getMinMax(std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bricks);
  void writeHeader(QTextStream& outStream);
  void writeCloud(QTextStream& outStream, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
  void writeFaces(QTextStream& outStream);
  pcl::PointCloud<pcl::PointXYZI>::Ptr brickToCloud(std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick);
  pcl::PointCloud<pcl::PointXYZI>::Ptr bricksToCloud(std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bricks);
  float brickToIntensity(std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick);
  QString getFullPath(QString path);

public:
  SF_ExportPly();
  void exportQSM(QString path, QString qsmName, std::shared_ptr<SF_ModelQSM> qsm, SF_ExportPlyPolicy exportPolicy, int resolution = 8);
};

#include "math/fit/line/sf_fitransacline.hpp"

#endif // SF_EXPORTPLY_H
