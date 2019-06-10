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

#include "sf_exportPly.h"

void
SF_ExportPly::getMinMax(std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bricks)
{
  m_maxIntensity = 0;
  m_minIntensity = std::numeric_limits<float>::max();
  for (std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick : bricks) {
    if (m_exportPolicy == SF_ExportPlyPolicy::GROWTH_LENGTH) {
      float logGrowthLength = std::log(brick->getGrowthLength());
      if (logGrowthLength > m_maxIntensity) {
        m_maxIntensity = logGrowthLength;
      }
      if (logGrowthLength < m_minIntensity) {
        m_minIntensity = logGrowthLength;
      }
    } else if (m_exportPolicy == SF_ExportPlyPolicy::GROWTH_VOLUME) {
      float logGrowthVolume = std::log(brick->getGrowthVolume());
      if (logGrowthVolume > m_maxIntensity) {
        m_maxIntensity = logGrowthVolume;
      }
      if (logGrowthVolume < m_minIntensity) {
        m_minIntensity = logGrowthVolume;
      }
    } else if (m_exportPolicy == SF_ExportPlyPolicy::GROWTH_LENGTH) {
      m_minIntensity = 0;
      m_maxIntensity = 1;
    }
  }
}

int
SF_ExportPly::getNumberOfPoints()
{
  auto bricks = m_qsm->getBuildingBricks();
  return bricks.size() * m_resolution * 2;
}

int
SF_ExportPly::getNumberOfFaces()
{
  // acutally the same size as number of points;
  return getNumberOfPoints();
}

void
SF_ExportPly::writeHeader(QTextStream& outStream)
{
  outStream << "ply\n"
               "format ascii 1.0\n"
               "comment author: Dr. Jan Hackenberg\n"
               "comment object: QSM computed with SimpleForest\n";
  outStream << "element vertex ";
  outStream << QString::number(getNumberOfPoints()).append("\n");
  outStream << "property float x\n"
               "property float y\n"
               "property float z\n"
               "property uchar red\n"
               "property uchar green\n"
               "property uchar blue\n";
  outStream << "element face ";
  outStream << QString::number(getNumberOfFaces()).append("\n");
  outStream << "property list uchar int vertex_index\n"
               "end_header\n";
}

void
SF_ExportPly::writeCloud(QTextStream& outStream, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  for (const pcl::PointXYZI& point : cloud->points) {
    outStream << QString::number(point.x);
    outStream << " ";
    outStream << QString::number(point.y);
    outStream << " ";
    outStream << QString::number(point.z);
    outStream << " ";
    outStream << SF_ColorFactory::getColorString(m_firstColor, m_secondColor, point.intensity);
    outStream << "\n";
  }
}

void
SF_ExportPly::writeFaces(QTextStream& outStream)
{
  for (int i = 0; i < static_cast<int>(m_qsm->getBuildingBricks().size()); i++) {
    int x = 2 * i;
    for (int j = 0; j < m_resolution - 1; j++) {
      outStream << "3 ";
      outStream << QString::number(x * m_resolution + j);
      outStream << " ";
      outStream << QString::number(x * m_resolution + j + 1);
      outStream << " ";
      outStream << QString::number((x + 1) * m_resolution + j + 1);
      outStream << "\n";

      outStream << "3 ";
      outStream << QString::number((x + 1) * m_resolution + j + 1);
      outStream << " ";
      outStream << QString::number((x + 1) * m_resolution + j);
      outStream << " ";
      outStream << QString::number(x * m_resolution + j);
      outStream << "\n";
    }
    outStream << "3 ";
    outStream << QString::number(x * m_resolution + m_resolution - 1);
    outStream << " ";
    outStream << QString::number(x * m_resolution);
    outStream << " ";
    outStream << QString::number((x + 1) * m_resolution);
    outStream << "\n";

    outStream << "3 ";
    outStream << QString::number((x + 1) * m_resolution);
    outStream << " ";
    outStream << QString::number((x + 1) * m_resolution + m_resolution - 1);
    outStream << " ";
    outStream << QString::number(x * m_resolution + m_resolution - 1);
    outStream << "\n";
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
SF_ExportPly::brickToCloud(std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick)
{
  double increment = (2.0 * SF_Math<double>::_PI) / m_resolution;
  double theta = 0;
  const auto center = brick->getCenter();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  double length = brick->getLength();
  double radius = brick->getRadius();
  for (int i = 0; i < m_resolution; i++) {
    float x = radius * std::cos(theta);
    float y = radius * std::sin(theta);
    float z = -length / 2;
    pcl::PointXYZI p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = brickToIntensity(brick);
    cloud->push_back(p);
    theta += increment;
  }
  theta = 0;
  for (int i = 0; i < m_resolution; i++) {
    float x = radius * std::cos(theta);
    float y = radius * std::sin(theta);
    float z = length / 2;
    pcl::PointXYZI p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = brickToIntensity(brick);
    cloud->push_back(p);
    theta += increment;
  }

  Eigen::Vector3f zAxis(0, 0, 1);
  Eigen::Vector3d cylinderAxis = brick->getAxis();
  Eigen::Vector3f cylinderAxisF = Eigen::Vector3f(cylinderAxis.coeff(0), cylinderAxis.coeff(1), cylinderAxis.coeff(2));
  cylinderAxisF = cylinderAxisF.normalized();
  Eigen::Vector3f rotationAxis = zAxis.cross(cylinderAxisF);
  rotationAxis = rotationAxis.normalized();
  float angle = std::acos(zAxis.dot(cylinderAxisF));

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.rotate(Eigen::AngleAxisf(angle, rotationAxis));
  transform.translation() << center.coeff(0), center.coeff(1), center.coeff(2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*cloud, *transformed, transform);
  return transformed;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
SF_ExportPly::bricksToCloud(std::vector<std::shared_ptr<Sf_ModelAbstractBuildingbrick>> bricks)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick : bricks) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr brickCloud = brickToCloud(brick);
    *cloud += *brickCloud;
  }
  return cloud;
}

float
SF_ExportPly::brickToIntensity(std::shared_ptr<Sf_ModelAbstractBuildingbrick> brick)
{
  float intensity = 0;
  if (m_exportPolicy == SF_ExportPlyPolicy::GROWTH_LENGTH) {
    intensity = std::log(brick->getGrowthLength());
  } else if (m_exportPolicy == SF_ExportPlyPolicy::GROWTH_VOLUME) {
    intensity = std::log(brick->getGrowthVolume());
  } else if (m_exportPolicy == SF_ExportPlyPolicy::GROWTH_LENGTH) {
    intensity = (brick->getSegment()->getBranchOrder() == 0) ? 1 : 0;
  }
  if ((m_maxIntensity - m_minIntensity) == 0)
    return 1;
  return (intensity - m_minIntensity) / (m_maxIntensity - m_minIntensity);
}

QString
SF_ExportPly::getFullPath(QString path)
{
  if (m_exportPolicy == SF_ExportPlyPolicy::GROWTH_LENGTH) {
    path.append("/ply/growthLength/");
  } else if (m_exportPolicy == SF_ExportPlyPolicy::GROWTH_VOLUME) {
    path.append("/ply/growthVolume/");
  } else if (m_exportPolicy == SF_ExportPlyPolicy::STEM) {
    path.append("/ply/stem/");
  }
  return path;
}

SF_ExportPly::SF_ExportPly() : SF_AbstractExport()
{
  m_firstColor = SF_ColorFactory::Color::GREEN;
  m_secondColor = SF_ColorFactory::Color::RED;
}

void
SF_ExportPly::exportQSM(QString path,
                        QString qsmName,
                        std::shared_ptr<SF_ModelQSM> qsm,
                        SF_ExportPlyPolicy exportPolicy,
                        int resolution)
{
  QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
  m_qsm = qsm;
  m_exportPolicy = exportPolicy;
  m_resolution = resolution;
  auto bricks = m_qsm->getBuildingBricks();
  getMinMax(bricks);
  QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));
  QString fullPath = getFullPath(path);
  QDir dir(fullPath);
  if (!dir.exists())
    dir.mkpath(".");
  QString fullName = getFullName(qsmName);
  fullPath.append(fullName);
  QFile file(fullPath);
  if (file.open(QIODevice::WriteOnly)) {
    QTextStream outStream(&file);
    writeHeader(outStream);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = bricksToCloud(bricks);
    writeCloud(outStream, cloud);
    writeFaces(outStream);
    file.close();
  }
}
