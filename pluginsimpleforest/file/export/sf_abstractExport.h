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

#ifndef SF_ABSTRACTEXPORT_H
#define SF_ABSTRACTEXPORT_H

#include "pcl/sf_math.h"
#include "qsm/model/sf_modelQSM.h"
#include "steps/visualization/sf_colorfactory.h"

#include <QDir>
#include <QLocale>
#include <QString>
#include <QTextStream>

class SF_AbstractExport
{
protected:
  SF_ColorFactory::Color m_firstColor = SF_ColorFactory::Color::GREEN;
  SF_ColorFactory::Color m_secondColor = SF_ColorFactory::Color::RED;
  std::shared_ptr<SF_ModelQSM> m_qsm;
  float m_minIntensity;
  float m_maxIntensity;
  QString getFullName(QString name, QString extension = QString(".ply"));

public:
  SF_AbstractExport();
  void setFirstColor(const SF_ColorFactory::Color firstColor);
  void setSecondColor(const SF_ColorFactory::Color secondColor);
};

#endif // SF_ABSTRACTEXPORT_H
