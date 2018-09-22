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
    pcl/cloud/filter/binary/sf_binary_filter.h \
    pcl/cloud/filter/binary/sf_binary_filter.hpp \
    pcl/cloud/filter/binary/statisticaloutlier/sf_statistical_outlier_filter.h \
    pcl/cloud/filter/binary/statisticaloutlier/sf_statistical_outlier_filter.hpp \
    steps/filter/binary/statistical_outlier_filter/sf_step_statistical_outlier_removal.h \
    steps/param/sf_abstract_param.h \
    steps/sf_abstract_step.h \
    steps/filter/sf_abstract_filter_step.h \
    steps/filter/binary/sf_abstract_filter_binary_step.h \
    steps/filter/multiple/sf_abstract_filter_multiple_step.h \
    steps/filter/multiple/voxel/sf_filter_3d_grid_sub_cloud.h \
    steps/filter/binary/radius_outlier_filter/sf_radius_outlier_filter_step.h \
    steps/filter/binary/ground_filter/sf_step_ground_filter_adapter.h \
    steps/filter/binary/ground_filter/sf_step_ground_filter.h \
    steps/filter/multiple/euclideanclustering/sf_binary_filter.h \
    pcl/cloud/filter/binary/radiusoutlier/sf_radius_outlier_filter.h \
    pcl/cloud/filter/binary/radiusoutlier/sf_radius_outlier_filter.hpp \
    steps/filter/binary/radius_outlier_filter/sf_radius_outlier_filter_adapter.h \
    steps/filter/binary/statistical_outlier_filter/sf_statistical_outlier_removal_adapter.h \
    pcl/cloud/feature/sf_abstract_feature.h \
    pcl/cloud/feature/sf_abstract_feature.hpp \
    pcl/cloud/feature/normals/sf_normal.h \
    pcl/cloud/feature/normals/sf_normal.hpp \
    pcl/cloud/filter/unitary/sf_unitary_filter.h \
    pcl/cloud/filter/unitary/sf_unitary_filter.hpp \
    pcl/cloud/filter/unitary/voxelgrid/sf_voxel_grid_ds.h \
    pcl/cloud/filter/unitary/voxelgrid/sf_voxel_grid_ds.hpp \
    pcl/cloud/filter/unitary/pca/sf_pca.h \
    pcl/cloud/filter/unitary/pca/sf_pca.hpp \
    pcl/cloud/filter/unitary/growthdirection/sf_growth_direction.h \
    pcl/cloud/filter/unitary/growthdirection/sf_growth_direction.hpp \
    pcl/cloud/filter/binary/stem/sf_stem_filter.h \
    pcl/cloud/filter/binary/stem/sf_stem_filter.hpp \
    pcl/cloud/filter/binary/ground/sf_ground_filter.h \
    pcl/cloud/filter/binary/ground/sf_ground_filter.hpp \
    steps/filter/binary/stem_filter/sf_step_stem_filter.h \
    steps/filter/binary/stem_filter/sf_step_stem_filter_adapter.h \
    pcl/sf_math.h \
    pcl/sf_math.hpp \
    pcl/cloud/feature/pca/sf_pca.h \
    pcl/cloud/feature/pca/sf_pca.hpp \
    pcl/cloud/feature/growth_direction/sf_growth_direction.h \
    pcl/cloud/feature/growth_direction/sf_growth_direction.hpp \
    pcl/geometry/DTM/sf_pyramidlayer.h \
    pcl/geometry/DTM/sf_pyramidlayer.hpp \
    pcl/geometry/DTM/sf_dtm.h \
    pcl/geometry/DTM/sf_dtm.hpp \
    steps/dtm/sf_dtm_step.h \
    math/interpolation/sf_interpolation.h \
    steps/filter/binary/cut_cloud_above_dtm/sf_step_cut_cloud_above_dtm.h \
    steps/filter/binary/cut_cloud_above_dtm/sf_cut_above_dtm_adapter.h \
    pcl/cloud/filter/multiple/sf_multiple_filter.h \
    pcl/cloud/filter/multiple/sf_multiple_filter.hpp \
    pcl/cloud/filter/multiple/euclideanclustering/sf_euclidean_clustering.h \
    pcl/cloud/filter/multiple/euclideanclustering/sf_euclidean_clustering.hpp \
    steps/filter/multiple/euclideanclustering/sf_euclidean_clustering_step.h \
    steps/segmentation/dijkstra/sf_dijkstra_segemtation.h \
    pcl/cloud/segmentation/dijkstra/sf_dijkstra.h \
    pcl/cloud/segmentation/dijkstra/sf_dijkstra.hpp \
    steps/segmentation/sf_segmentation_step.h \
    steps/segmentation/voronoi/sf_voronoi_segmentation.h \
    steps/filter/binary/stem_filter/sf_step_stem_ransac_filter.h \
    pcl/cloud/filter/binary/stem/sf_stem_ransac_filter.h \
    steps/filter/binary/stem_filter/sf_step_stem_filter_ransac_adapter.h \
    steps/qsm/modelling/sf_step_spherefollowing_basic.h \
    steps/qsm/modelling/sf_step_spherefollowing_basic_adapter.h \
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
    converters/sf_abstractConverter.h
SOURCES += \
    sf_pluginentry.cpp \
    sf_pluginmanager.cpp \
    steps/sf_abstract_step.cpp \
    steps/filter/binary/statistical_outlier_filter/sf_step_statistical_outlier_removal.cpp \
    steps/filter/sf_abstract_filter_step.cpp \
    steps/filter/binary/sf_abstract_filter_binary_step.cpp \
    steps/filter/multiple/sf_abstract_filter_multiple_step.cpp \
    steps/filter/multiple/voxel/sf_filter_3d_grid_sub_cloud.cpp \
    steps/filter/binary/radius_outlier_filter/sf_radius_outlier_filter_step.cpp \
    steps/filter/binary/ground_filter/sf_step_ground_filter.cpp \
    steps/filter/binary/stem_filter/sf_step_stem_filter.cpp \
    steps/dtm/sf_dtm_step.cpp \
    plot/DTM/sf_dtm.cpp \
    math/interpolation/sf_interpolation.cpp \
    steps/filter/binary/cut_cloud_above_dtm/sf_step_cut_cloud_above_dtm.cpp \
    steps/filter/multiple/euclideanclustering/sf_euclidean_clustering_step.cpp \
    steps/segmentation/dijkstra/sf_dijkstra_segemtation.cpp \
    steps/segmentation/sf_segmentation_step.cpp \
    steps/segmentation/voronoi/sf_voronoi_segmentation.cpp \
    steps/filter/binary/stem_filter/sf_step_stem_ransac_filter.cpp \
    pcl/cloud/filter/binary/stem/sf_stem_ransac_filter.cpp \
    steps/qsm/modelling/sf_step_spherefollowing_basic.cpp \
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
    pcl/geometry/sf_pointGeometry.cpp

TRANSLATIONS += languages/pluginsimpleforest_en.ts \
                languages/pluginsimpleforest_fr.ts
