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
#ifndef SF_ABSTRACT_PARAM_H
#define SF_ABSTRACT_PARAM_H

#include "ct_colorcloud/ct_colorcloudstdvector.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_itemdrawable/ct_image2d.h"
#include "ct_itemdrawable/ct_pointsattributescolor.h"
#include "ct_result/model/outModel/ct_outresultmodelgroup.h"
#include "pcl/sf_point.h"
#include "qsm/algorithm/distance/sf_cloudToModelDistanceParameters.h"
#include "qsm/algorithm/spherefollowing/sf_spherefollowingParameters.h"
#include "qsm/model/sf_modelQSM.h"
#include "steps/sf_stepprogress.h"
#include <pcl/sample_consensus/method_types.h>

struct SF_ParamCT
{
  CT_StandardItemGroup* _grpCpyGrp;
  LogInterface* _log;
  const CT_AbstractItemDrawableWithPointCloud* _itemCpyCloudIn;
  CT_ResultGroup* _resCpyRes;
  virtual QString toString()
  {
    QString str;
    return str;
  }
  std::shared_ptr<SF_StepProgress> _stepProgress;

  virtual void log_import()
  {
    QString str = toStringImport();
    _log->addMessage(LogInterface::info, LogInterface::step, str);
  }

  virtual QString toFilterString(size_t total, size_t noise)
  {
    double percentage;
    if (total == 0) {
      percentage = 0;
    } else {
      percentage = (static_cast<float>(noise) / static_cast<float>(total)) * 100.0f;
    }
    QString str = "The filter removed  ";
    str.append(QString::number(noise));
    str.append(" (");
    str.append(QString::number(percentage, 'f', 1));
    str.append("%) points.");
    return str;
  }

private:
  virtual QString toStringImport()
  {
    QString str = "A cloud with ";
    str.append(QString::number(_itemCpyCloudIn->getPointCloudIndex()->size()));
    str.append(" points was successfully converted.");
    return str;
  }
};

template<typename PointType>
struct SF_ParamCloud : public SF_ParamCT
{
  typename pcl::PointCloud<PointType>::Ptr _cloudIn;
  virtual void logFilter(double percentage)
  {
    QString str = toStringFilter(percentage);
    _log->addMessage(LogInterface::info, LogInterface::step, str);
  }

private:
  virtual QString toStringFilter(double percentage)
  {
    QString str = "From the cloud with ";
    str.append(QString::number(_cloudIn->points.size()));
    str.append(" points remain after filtering ");
    str.append(QString::number(percentage, 'f', 2));
    str.append(" percent remain.");
    return str;
  }
};

template<typename PointType>
struct SF_ParamFilter : public SF_ParamCloud<PointType>
{
  int _sizeOutput;
  std::vector<int> _outputIndices;
};

template<typename PointType>
struct SF_ParamQSM : public SF_ParamFilter<PointType>
{
  std::shared_ptr<SF_ModelQSM> _tree;
  Eigen::Vector3d _translation;
  CT_ColorCloudStdVector* _colors;
};

template<typename PointType>
struct SF_ParamEuclideanClustering : public SF_ParamFilter<PointType>
{
  float _cellSize;
  float _euclideanDistance;
  int _minSize;
};

template<typename PointType>
struct SF_ParamDTMHeight : public SF_ParamFilter<PointType>
{
  float _sliceHeight;
  CT_Image2D<float>* _dtmCT;
  virtual QStringList toStringList()
  {
    QStringList list;
    QString str = "The with the DTM normalized Cloud has been sliced at (";
    list.push_back(str);
    str = ("height                  = ");
    str.append(QString::number(_sliceHeight));
    list.push_back(str);
    str = (") into lower and upper cloud.");
    list.push_back(str);
    return list;
  }

  virtual QString toFilterString(size_t lower, size_t upper)
  {
    if (lower + upper == 0) {
      return QString("The cloud to be sliced is empty.");
    }
    double lowerPerc = static_cast<double>(lower) * 100.0 / (static_cast<double>(lower) + static_cast<double>(upper));
    double upperPerc = static_cast<double>(upper) * 100.0 / (static_cast<double>(lower) + static_cast<double>(upper));
    QString str = "In the lower slice are  ";
    str.append(QString::number(lower / 1000));
    str.append("k (");
    str.append(QString::number(lowerPerc, 'f', 2));
    str.append(" %)");
    str.append(" points, in the upper ");
    str.append(QString::number(upper / 1000));
    str.append("k (");
    str.append(QString::number(upperPerc, 'f', 2));
    str.append(" %).");
    return str;
  }
};

template<typename PointType>
struct SF_ParamGrowthDirection : public SF_ParamCloud<PointType>
{
  float _gdRange = 0.03f;
  float _normalRange = 0.01f;
  virtual QString toString()
  {
    QString str = "The growth direction computation with parameter (";
    str.append(QString::number(_gdRange));
    str.append(" growth direction range) has started.");
    return str;
  }
};

template<typename PointType>
struct SF_ParamVoxelGridDownscale : public SF_ParamCloud<PointType>
{
  float voxelSize = 0.01f;
  float voxelSizeX = 0.01f;
  float voxelSizeY = 0.01f;
  float voxelSizeZ = 0.01f;
  virtual QString toString()
  {
    QString str = "The voxelgrid downscale filter with parameters (cell_size_x = ";
    str.append(QString::number(voxelSizeX));
    str.append("; cell_size_y = ");
    str.append(QString::number(voxelSizeY));
    str.append("; cell_size_z = ");
    str.append(QString::number(voxelSizeZ));
    str.append(") is started.");
    return str;
  }
};

template<typename PointType, typename FeatureType>
struct SF_ParamNormals : public SF_ParamCloud<PointType>
{
  bool _useRadius = true;
  float _radius = 0.03f;
  int _kn = 25;
  virtual QString toString()
  {
    QString str = "The normal estimtion with a neighborhood size of ";
    if (_useRadius) {
      str.append(QString::number(_radius));
      str.append(" m ");
    } else {
      str.append(QString::number(_kn));
      str.append(" nearest neighbors ");
    }
    str.append("is started.");
    return str;
  }
};

template<typename PointType>
struct SF_ParamSegmentTreeFromQSM : public SF_ParamFilter<PointType>
{
  int _numClstrs = 3;
  float _normalRadius = 0.03f;
  SF_CloudToModelDistanceParameters _distanceParams;
  std::shared_ptr<SF_ModelQSM> _qsm;
  CT_ColorCloudStdVector* _colorsGrowthVolume;
  CT_ColorCloudStdVector* _colorsClusters;
  virtual QString toString()
  {
    QString str = "Firstly the cloud got its normals computed with radius ";
    str.append(QString::number(_normalRadius));
    str.append("[m]. Then each tree cloud has been clustered into ");
    str.append(QString::number(_numClstrs));
    str.append(" evensized clusters.");
    return str;
  }
};

template<typename PointType>
struct SF_ParamGroundFilter : public SF_ParamFilter<PointType>
{
  float _voxelSize = 0.04f;
  float _radiusNormal = 0.2f;
  float _x = 0;
  float _y = 0;
  float _z = 1;
  int _angle = 20;
  CT_ColorCloudStdVector* _colors;

  virtual QStringList toStringList()
  {
    QStringList list;
    QString str = "The ground point filter with parameters (";
    list.push_back(str);
    str = ("angle                  = ");
    str.append(QString::number(_angle));
    list.push_back(str);
    str = ("axisX              = ");
    str.append(QString::number(_x));
    list.push_back(str);
    str = ("axisY              = ");
    str.append(QString::number(_y));
    list.push_back(str);
    str = ("axisZ              = ");
    str.append(QString::number(_z));
    list.push_back(str);
    str = ("radiusNormal       = ");
    str.append(QString::number(_radiusNormal));
    list.push_back(str);
    str = ("voxelDownScaleSize = ");
    str.append(QString::number(_voxelSize));
    list.push_back(str);
    str = (") is finished.");
    list.push_back(str);
    return list;
  }
};

template<typename PointType>
struct SF_ParamDtm : public SF_ParamFilter<PointType>
{
  float _voxelSize = 0.03f;
  float _radiusNormal = 0.1f;
  float _minCellSize = 0.2;
  virtual QString toString()
  {
    QString str = "The DTM generation with parameters (voxelSize = ";
    str.append(QString::number(_voxelSize));
    str.append("; normalRadius = ");
    str.append(QString::number(_radiusNormal));
    str.append(" and minSizeCells = ");
    str.append(QString::number(_minCellSize));
    str.append(") is finished.");
    return str;
  }
};

struct SF_ParamStemRansacFilter : public SF_ParamFilter<pcl::PointXYZINormal>
{
  float _voxelSize = 0.01f;
  float _radiusNormal = 0.03f;
  float _inlierDistance = 0.1f;
  float _x = 0;
  float _y = 0;
  float _z = 1;
  int _angle = 20;

  virtual QString toString()
  {
    QString str = "The stem RANSAC filter ";
    str.append(" is started.");
    return str;
  }

  virtual QStringList toStringList()
  {
    QStringList list;
    QString str = "The stem RANSAC filter with parameters (";
    list.push_back(str);
    str = ("_voxelSize = ");
    str.append(QString::number(_voxelSize));
    list.push_back(str);
    str = ("radius for normal computation = ");
    str.append(QString::number(_radiusNormal));
    list.push_back(str);
    str = ("_inlierDistance = ");
    str.append(QString::number(_inlierDistance));
    list.push_back(str);
    str = ("maximum angle = ");
    str.append(QString::number(_angle));
    list.push_back(str);
    str = (") is finished.");
    list.push_back(str);
    return list;
  }
};

template<typename PointType>
struct SF_ParamStemFilter : public SF_ParamFilter<PointType>
{
  float _voxelSize = 0.02f;
  float _radiusNormal = 0.05f;
  float _radiusGrowthDirection = 0.1f;
  float _x = 0;
  float _y = 0;
  float _z = 1;
  int _angle = 10;
  CT_ColorCloudStdVector* _colors;
  virtual QString toString()
  {
    QString str = "The stem filter with parameters (angle = ";
    str.append(QString::number(_angle));
    str.append("); radius_growth_direction =");
    str.append(QString::number(_radiusGrowthDirection));
    str.append("); radius_normal = ");
    str.append(QString::number(_radiusNormal));
    str.append(" and voxel_down_scale_size = ");
    str.append(QString::number(_voxelSize));
    str.append(") is started.");
    return str;
  }

  virtual QStringList toStringList()
  {
    QStringList list;
    QString str = "The stem filter with parameters (";
    list.push_back(str);
    str = ("_voxelSize = ");
    str.append(QString::number(_voxelSize));
    list.push_back(str);
    str = ("radius for normal computation = ");
    str.append(QString::number(_radiusNormal));
    list.push_back(str);
    str = ("radius for growth direction computation = ");
    str.append(QString::number(_radiusGrowthDirection));
    list.push_back(str);
    str = ("maximum angle = ");
    str.append(QString::number(_angle));
    list.push_back(str);
    str = (") is finished.");
    list.push_back(str);
    return list;
  }
};

template<typename PointType>
struct SF_ParamSpherefollowingBasic : public SF_ParamQSM<PointType>
{
  SF_SphereFollowingParameters _sphereFollowingParams;
  SF_CloudToModelDistanceParameters _distanceParams;
  float _voxelSize = 0.01;
  float _clusteringDistance = 0.2;
  float _modelCloudError;
  int _fittedGeometries;
  size_t m_numClstrs = 1;

  QString methodToString(int method)
  {
    QString str;
    switch (method) {
      case pcl::SAC_RANSAC:
        str = "RANSAC";
        break;
      case pcl::SAC_LMEDS:
        str = "LMEDS";
        break;
      case pcl::SAC_MLESAC:
        str = "MLESAC";
        break;
      case pcl::SAC_MSAC:
        str = "MSAC";
        break;
      case pcl::SAC_PROSAC:
        str = "PROSAC";
        break;
      case pcl::SAC_RMSAC:
        str = "SAC_RMSAC";
        break;
      case pcl::SAC_RRANSAC:
        str = "SAC_RRANSAC";
        break;
      default:
        break;
    }
    return str;
  }

  virtual QString toString()
  {
    QString str = "The SphereFollwing method with parameters \n (_voxelSize = ";
    str.append(QString::number(_voxelSize));
    str.append("; _ransacCircleInlierDistance = ");
    str.append(QString::number(_sphereFollowingParams._inlierDistance));
    str.append("; Optimizable Parameters Output Following: \n {");
    for (size_t i = 0; i < _sphereFollowingParams.m_optimizationParams.size(); i++) {
      str.append(" { _euclideanClusteringDistance = ");
      str.append(QString::number(_sphereFollowingParams.m_optimizationParams.at(i)._euclideanClusteringDistance));
      str.append("; _sphereRadiusMultiplier = ");
      str.append(QString::number(_sphereFollowingParams.m_optimizationParams.at(i)._sphereRadiusMultiplier));
      str.append("; _epsilonSphere = ");
      str.append(QString::number(_sphereFollowingParams.m_optimizationParams.at(i)._epsilonSphere));
      str.append("}");
    }
    str.append("}\n Hyper parameters following: _minPtsCircle = ");
    str.append(QString::number(_sphereFollowingParams._minPtsGeometry));
    str.append("; _heightStartSphere = ");
    str.append(QString::number(_sphereFollowingParams._heightInitializationSlice));
    str.append("; _medianRadiusMultiplier = ");
    str.append(QString::number(_sphereFollowingParams._medianRadiusMultiplier));
    str.append("; _ransacCircleInlierDistance = ");
    str.append(QString::number(_sphereFollowingParams._inlierDistance));
    str.append("; _ransacIterations = ");
    str.append(QString::number(_sphereFollowingParams._RANSACIterations));
    str.append("; _minGlobalRadius = ");
    str.append(QString::number(_sphereFollowingParams._minGlobalRadius));

    str.append("; _fittingMethod = ");
    str.append(methodToString(_sphereFollowingParams._fittingMethod));
    str.append(") \n has been optimized. During the Parameter search ");
    str.append(QString::number(_sphereFollowingParams.m_optimizationParams.size() * 6));
    str.append(" parameters have been optimized.");
    str.append(" The error has been reduced from \n  max:");
    str.append(QString::number(1337));
    str.append("\n to min:");
    str.append(QString::number(_modelCloudError));
    str.append(". \n During the procedure ");
    str.append(QString::number(_fittedGeometries));
    str.append(" geometries have been fitted.");
    return str;
  }
};

template<typename PointType>
struct SF_ParamStatisticalOutlierFilter : public SF_ParamFilter<PointType>
{
  int _k = 25;
  float _stdMult = 3.5;
  int _iterations = 15;
  virtual QString toString()
  {
    QString str = "The statistical outlier filter with parameters (kn = ";
    str.append(QString::number(_k));
    str.append("; std_mult = ");
    str.append(QString::number(_stdMult));
    str.append("; iterations = ");
    str.append(QString::number(_iterations));
    str.append("; iterations = ");
    str.append(") is started.");
    return str;
  }

  virtual QStringList toStringList()
  {
    QStringList list;
    QString str = "The statistical outlier filter with parameters (";
    list.push_back(str);
    str = ("number nearest neighbors = ");
    str.append(QString::number(_k));
    list.push_back(str);
    str = ("standard devation multiplier = ");
    str.append(QString::number(_stdMult));
    list.push_back(str);
    str = ("iterations              = ");
    str.append(QString::number(_iterations));
    list.push_back(str);
    str = (") is finished.");
    list.push_back(str);
    return list;
  }
};

template<typename PointType>
struct SF_ParamRadiusOutlierFilter : public SF_ParamFilter<PointType>
{
  int _minPts = 15;
  double _radius = 10;
  CT_ColorCloudStdVector* _colors;
  virtual QString toString()
  {
    QString str = "The radius outlier filter with parameters (_min_pts = ";
    str.append(QString::number(_minPts));
    str.append("; radius = ");
    str.append(QString::number(_radius));
    str.append(") is started.");
    return str;
  }

  virtual QStringList toStringList()
  {
    QStringList list;
    QString str = "The radius outlier filter with parameters (";
    list.push_back(str);
    str = ("minPts                  = ");
    str.append(QString::number(_minPts));
    list.push_back(str);
    str = ("radius              = ");
    str.append(QString::number(_radius));
    list.push_back(str);
    str = (") is finished.");
    list.push_back(str);
    return list;
  }
};

#endif // SF_ABSTRACT_PARAM_H
