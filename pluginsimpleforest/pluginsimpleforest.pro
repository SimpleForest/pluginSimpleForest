CT_PREFIX = ../../computreev3
exists(../../computreev5) {
    CT_PREFIX = ../../computreev5
    DEFINES += COMPUTREE_V5
}

MUST_USE_PCL = 1
MUST_USE_OPENCV = 1
QT += concurrent
COMPUTREE += ctlibpcl

include($${CT_PREFIX}/shared.pri)
include($${PLUGIN_SHARED_DIR}/include.pri)

TARGET = plug_simpleforest

HEADERS += $${PLUGIN_SHARED_INTERFACE_DIR}/interfaces.h \
    sf_pluginentry.h \
    sf_pluginmanager.h \
    pcl/sf_point.h \
    steps/param/sf_abstract_param.h \
    steps/filter/multiple/euclideanclustering/sf_binary_filter.h \
    pcl/cloud/feature/normals/sf_normal.h \
    pcl/cloud/feature/normals/sf_normal.hpp \
    pcl/sf_math.h \
    pcl/sf_math.hpp \
    pcl/cloud/feature/pca/sf_pca.h \
    pcl/cloud/feature/pca/sf_pca.hpp \
    pcl/geometry/DTM/sf_pyramidlayer.h \
    pcl/geometry/DTM/sf_pyramidlayer.hpp \
    pcl/geometry/DTM/sf_dtm.h \
    pcl/geometry/DTM/sf_dtm.hpp \
    steps/dtm/sf_dtm_step.h \
    math/interpolation/sf_interpolation.h \
    pcl/cloud/segmentation/dijkstra/sf_dijkstra.h \
    pcl/cloud/segmentation/dijkstra/sf_dijkstra.hpp \
    qsm/algorithm/spherefollowing/sf_spherefollowing_parameters.h \
    qsm/algorithm/spherefollowing/sf_sphere_following.h \
    qsm/algorithm/detection/sf_idetection.h \
    qsm/algorithm/optimization/gridsearch/sf_gridsearch.h \
    qsm/algorithm/optimization/gridsearch/sf_gridsearchparameters.h \
    qsm/sf_modelAbstractBuildingbrick.h \
    qsm/sf_modelCylinderBuildingbrick.h \
    qsm/sf_modelQSM.h \
    qsm/sf_modelAbstractSegment.h \
    qsm/algorithm/sf_buildQSM.h \
    qsm/algorithm/sf_QSMAlgorithm.h \
    qsm/algorithm/sf_QSMCylinder.h \
    qsm/algorithm/distance/sf_cloudToModelDistance.h \
    qsm/algorithm/distance/sf_cloudToModelDistanceParameters.h \
    plot/sf_modelRaster.h \
    plot/sf_modelPlot.h \
    plot/DTM/sf_modelDTM.h \
    pcl/geometry/sf_geometryPoint.h \
    pcl/geometry/DTM/sf_dtmCell.h \
    pcl/geometry/DTM/sf_dtmCell.hpp \
    pcl/cloud/sf_abstractCloud.h \
    pcl/cloud/sf_abstractCloud.hpp \
    pcl/cloud/filter/sf_abstractFilter.h \
    pcl/cloud/filter/sf_abstractFilter.hpp \
    converters/CT_To_PCL/sf_converterCTToPCL.h \
    converters/CT_To_PCL/sf_converterCTToPCL.hpp \
    converters/CT_To_PCL/sf_converterCTToPCLDTM.h \
    converters/sf_abstractConverter.h \
    pcl/cloud/filter/binary/sf_abstractBinaryFilter.h \
    pcl/cloud/filter/binary/sf_abstractBinaryFilter.hpp \
    pcl/cloud/filter/binary/stem/sf_stemFilter.h \
    pcl/cloud/filter/binary/stem/sf_stemFilter.hpp \
    pcl/cloud/filter/binary/stem/sf_stemRansacFilter.h \
    pcl/cloud/filter/binary/statisticaloutlier/sf_statisticalOutlierFilter.hpp \
    pcl/cloud/filter/binary/statisticaloutlier/sf_statisticalOutlierFilter.h \
    pcl/cloud/filter/binary/radiusoutlier/sf_radiusOutlierFilter.h \
    pcl/cloud/filter/binary/radiusoutlier/sf_radiusOutlierFilter.hpp \
    pcl/cloud/filter/binary/ground/sf_groundFilter.h \
    pcl/cloud/filter/binary/ground/sf_groundFilter.hpp \
    pcl/cloud/feature/sf_abstractFeature.h \
    pcl/cloud/feature/sf_abstractFeature.hpp \
    pcl/cloud/feature/pca/sf_pcavalues.h \
    pcl/cloud/feature/growth_direction/sf_growthDirection.h \
    pcl/cloud/feature/growth_direction/sf_growthDirection.hpp \
    steps/sf_abstractStep.h \
    steps/filter/sf_abstractFilterStep.h \
    steps/filter/multiple/voxel/sf_filter3dGridSubCloud.h \
    steps/filter/multiple/euclideanclustering/sf_euclideanClusteringStep.h \
    steps/filter/multiple/sf_abstractFilterMultipleStep.h \
    steps/filter/binary/sf_abstractFilterBinaryStep.h \
    steps/filter/binary/stem_filter/sf_stepStemFilter.h \
    steps/filter/binary/stem_filter/sf_stepStemFilterAdapter.h \
    steps/filter/binary/stem_filter/sf_stepStemRANSACFilter.h \
    steps/filter/binary/stem_filter/sf_stepStemFilterRANSACAdapter.h \
    steps/filter/binary/statistical_outlier_filter/sf_stepStatisticalOutlierRemoval.h \
    steps/filter/binary/statistical_outlier_filter/sf_statisticalOutlierRemovalAdapter.h \
    steps/filter/binary/radius_outlier_filter/sf_radiusOutlierFilterStep.h \
    steps/filter/binary/radius_outlier_filter/sf_radiusOutlierFilterAdapter.h \
    steps/filter/binary/ground_filter/sf_stepGroundFilter.h \
    steps/filter/binary/ground_filter/sf_stepGroundFilterAdapter.h \
    steps/filter/binary/cut_cloud_above_dtm/sf_stepCutCloudAboveDTM.h \
    steps/filter/binary/cut_cloud_above_dtm/sf_stepCutCloudAboveDTMAdapter.h \
    steps/qsm/modelling/sf_stepSpherefollowingRoot.h \
    steps/qsm/modelling/sf_stepSpherefollowingBasicAdapter.h \
    steps/segmentation/sf_AbstractStepSegmentation.h \
    steps/segmentation/voronoi/sf_stepSegmentationVoronoi.h \
    steps/segmentation/dijkstra/sf_stepSegemtationDijkstra.h
SOURCES += \
    sf_pluginentry.cpp \
    sf_pluginmanager.cpp \
    steps/dtm/sf_dtm_step.cpp \
    plot/DTM/sf_dtm.cpp \
    math/interpolation/sf_interpolation.cpp \
    qsm/algorithm/spherefollowing/sf_sphere_following.cpp \
    qsm/algorithm/optimization/gridsearch/sf_gridsearch.cpp \
    qsm/sf_modelCylinderBuildingbrick.cpp \
    qsm/sf_modelAbstractSegment.cpp \
    qsm/sf_modelQSM.cpp \
    qsm/sf_modelAbstractBuildingbrick.cpp \
    qsm/algorithm/sf_buildQSM.cpp \
    qsm/algorithm/distance/sf_cloudToModelDistance.cpp \
    converters/sf_abstractConverter.cpp \
    converters/CT_To_PCL/sf_converterCTToPCLDTM.cpp \
    plot/sf_modelRaster.cpp \
    plot/sf_modelPlot.cpp \
    pcl/geometry/sf_pointGeometry.cpp \
    pcl/cloud/filter/binary/stem/sf_stemRansacFilter.cpp \
    steps/sf_abstractStep.cpp \
    steps/filter/sf_abstractFilterStep.cpp \
    steps/filter/multiple/sf_abstractFilterMultipleStep.cpp \
    steps/filter/multiple/voxel/sf_filter3dGridSubCloud.cpp \
    steps/filter/multiple/euclideanclustering/sf_euclideanClusteringStep.cpp \
    steps/filter/binary/sf_abstractFilterBinaryStep.cpp \
    steps/filter/binary/stem_filter/sf_stepStemFilter.cpp \
    steps/filter/binary/stem_filter/sf_stepStemRANSACFilter.cpp \
    steps/filter/binary/statistical_outlier_filter/sf_stepStatisticalOutlierRemoval.cpp \
    steps/filter/binary/radius_outlier_filter/sf_radiusOutlierFilterStep.cpp \
    steps/filter/binary/ground_filter/sf_stepGroundFilter.cpp \
    steps/filter/binary/cut_cloud_above_dtm/sf_stepCutCloudAboveDTM.cpp \
    steps/qsm/modelling/sf_stepSpherefollowingRoot.cpp \
    steps/segmentation/sf_AbstractStepSegmentation.cpp \
    steps/segmentation/voronoi/sf_stepSegmentationVoronoi.cpp \
    steps/segmentation/dijkstra/sf_stepSegemtationDijkstra.cpp

TRANSLATIONS += languages/pluginsimpleforest_en.ts \
                languages/pluginsimpleforest_fr.ts
