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

#include "sf_exportCloud.h"

void
SF_ExportCloud::getMinMax()
{
  SF_CLoudToModelDistanceMethod type;
  if (m_exportPolicy == SF_ExportCloudPolicy::FIT_QUALITY) {
    type = SF_CLoudToModelDistanceMethod::SECONDMOMENTUMORDERMSAC;
    Sf_CloudToModelDistance<pcl::PointXYZINormal> cmd(m_qsm, m_cloud, type, 0.1, 9);
    m_intensities = cmd.distances();
  } else if (m_exportPolicy == SF_ExportCloudPolicy::GROWTHLENGTH) {
    type = SF_CLoudToModelDistanceMethod::GROWTHDISTANCE;
    Sf_CloudToModelDistance<pcl::PointXYZINormal> cmd(m_qsm, m_cloud, type, 0.1, 9);
    m_intensities = cmd.distances();
  } else if (m_exportPolicy == SF_ExportCloudPolicy::RADIUS) {
    type = SF_CLoudToModelDistanceMethod::RADIUS;
    Sf_CloudToModelDistance<pcl::PointXYZINormal> cmd(m_qsm, m_cloud, type, 0.1, 9);
    m_intensities = cmd.distances();
  }
  m_intensities = getLogarithm(m_intensities);
  m_maxIntensity = 0;
  m_minIntensity = std::numeric_limits<float>::max();
  for (double intensity : m_intensities) {
    if (intensity > m_maxIntensity) {
      m_maxIntensity = intensity;
    }
    if (intensity < m_minIntensity) {
      m_minIntensity = intensity;
    }
  }
}

std::vector<double>
SF_ExportCloud::getLogarithm(std::vector<double>& vector)
{
  if (m_exportPolicy == SF_ExportCloudPolicy::RADIUS) {
    return vector;
  }
  std::vector<double> result;
  result.resize(vector.size());
  std::transform(vector.begin(), vector.end(), result.begin(), [](double value) -> double {
    if (value == 0)
      return 0;
    return (std::log(std::abs(value)));
  });
  return result;
}

QString
SF_ExportCloud::getFullPath(QString path)
{
  if (m_exportPolicy == SF_ExportCloudPolicy::FIT_QUALITY) {
    path.append("/cloud/fitQuality/");
  } else if (m_exportPolicy == SF_ExportCloudPolicy::RADIUS) {
    path.append("/cloud/radius/");
  } else if (m_exportPolicy == SF_ExportCloudPolicy::GROWTHLENGTH) {
    path.append("/cloud/growthLength/");
  } else {
    throw std::runtime_error("SF_ExportCloud: illegal SF_ExportCloudPolicy");
  }
  return path;
}

SF_ExportCloud::SF_ExportCloud() : SF_AbstractExport()
{
  m_firstColor = SF_ColorFactory::Color::BLUE;
  m_secondColor = SF_ColorFactory::Color::YELLOW;
}

void
SF_ExportCloud::exportCloud(QString path,
                            QString cloudName,
                            std::shared_ptr<SF_ModelQSM> qsm,
                            SF_CloudNormal::Ptr cloud,
                            SF_ExportCloudPolicy exportPolicy)
{
  QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
  m_qsm = qsm;
  m_exportPolicy = exportPolicy;
  m_cloud = cloud;
  getMinMax();
  QString fullPath = getFullPath(path);
  QDir dir(fullPath);
  if (!dir.exists())
    dir.mkpath(".");
  QString fullName = getFullName(cloudName, ".csv");
  fullPath.append(fullName);
  QFile file(fullPath);
  if (file.open(QIODevice::WriteOnly)) {
    QTextStream outStream(&file);
    outStream << "x, y, z, red, green, blue, intensity\n";
    size_t index = 0;
    for (const SF_PointNormal point : m_cloud->points) {
      outStream << point.x;
      outStream << ", ";
      outStream << point.y;
      outStream << ", ";
      outStream << point.z;
      outStream << ", ";
      auto intensity = m_intensities[index++];
      intensity = ((m_maxIntensity - m_minIntensity) == 0) ? 1 : (intensity - m_minIntensity) / (m_maxIntensity - m_minIntensity);
      outStream << SF_ColorFactory::getColorString(m_firstColor, m_secondColor, point.intensity, ", ");
      outStream << ", ";
      outStream << intensity;
      outStream << "\n";
    }
    file.close();
  }
}
