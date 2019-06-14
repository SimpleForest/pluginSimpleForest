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

#include "sf_exportQSM.h"

QString
SF_ExportQSM::getFullPath(QString path)
{
  path.append("/qsm/detailed/");
  return path;
}

SF_ExportQSM::SF_ExportQSM() : SF_AbstractExport() {}

void
SF_ExportQSM::exportQSM(QString path, QString qsmName, std::shared_ptr<SF_ModelQSM> qsm)
{
  QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
  m_qsm = qsm;
  QString fullPath = getFullPath(path);
  QDir dir(fullPath);
  if (!dir.exists())
    dir.mkpath(".");
  QString fullName = getFullName(qsmName, ".csv");
  fullPath.append(fullName);
  QFile file(fullPath);
  if (file.open(QIODevice::WriteOnly)) {
    QTextStream outStream(&file);
    auto bricks = m_qsm->getBuildingBricks();
    outStream << QString::fromStdString(bricks.front()->toHeaderString());
    for (auto brick : bricks) {
      outStream << QString::fromStdString(brick->toString());
    }
    file.close();
  }
}
