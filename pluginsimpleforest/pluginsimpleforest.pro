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
    qsm/sf_model_abstract_segment.h \
    qsm/sf_model_abstract_buildingbrick.h \
    pcl/sf_point.h \
    pcl/cloud/filter/sf_abstract_filter.h \
    pcl/cloud/filter/binary/sf_binary_filter.h \
    pcl/cloud/sf_abstract_cloud.hpp \
    pcl/cloud/sf_abstract_cloud.h \
    pcl/cloud/filter/sf_abstract_filter.hpp \
    pcl/cloud/filter/binary/sf_binary_filter.hpp \
    pcl/cloud/filter/binary/statisticaloutlier/sf_statistical_outlier_filter.h \
    pcl/cloud/filter/binary/statisticaloutlier/sf_statistical_outlier_filter.hpp \
    steps/filter/binary/statistical_outlier_filter/sf_step_statistical_outlier_removal.h \
    converters/CT_To_PCL/sf_converter_ct_to_pcl.h \
    converters/CT_To_PCL/sf_converter_ct_to_pcl.hpp \
    converters/sf_abstract_converter.h \
    steps/param/sf_abstract_param.h \
    steps/sf_abstract_step.h \
    steps/filter/sf_abstract_filter_step.h \
    steps/filter/binary/sf_abstract_filter_binary_step.h \
    steps/filter/multiple/sf_abstract_filter_multiple_step.h \
    steps/filter/multiple/voxel/sf_filter_3d_grid_sub_cloud.h \
    steps/filter/binary/radius_outlier_filter/sf_radius_outlier_filter_step.h \
    steps/filter/binary/ground_filter/sf_step_ground_filter_adapter.h \
    steps/filter/binary/ground_filter/sf_step_ground_filter.h \
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
    pcl/geometry/sf_point_geometry.h \
    pcl/cloud/feature/pca/sf_pca.h \
    pcl/cloud/feature/pca/sf_pca.hpp \
    pcl/cloud/feature/growth_direction/sf_growth_direction.h \
    pcl/cloud/feature/growth_direction/sf_growth_direction.hpp \
    pcl/geometry/DTM/sf_pyramidlayer.h \
    pcl/geometry/DTM/sf_cell.h \
    pcl/geometry/DTM/sf_pyramidlayer.hpp \
    pcl/geometry/DTM/sf_dtm.h \
    pcl/geometry/DTM/sf_dtm.hpp \
    steps/dtm/sf_dtm_step.h \
    pcl/geometry/DTM/sf_cell.hpp \
    plot/sf_plot_model.h \
    plot/DTM/sf_dtm.h \
    plot/sf_raster_model.h \
    math/interpolation/sf_interpolation.h \
    converters/CT_To_PCL/sf_converter_ct_to_pcl_dtm.h
SOURCES += \
    sf_pluginentry.cpp \
    sf_pluginmanager.cpp \
    qsm/sf_model_abstract_segment.cpp \
    qsm/sf_model_abstract_buildingbrick.cpp \
    converters/sf_abstract_converter.cpp \
    steps/sf_abstract_step.cpp \
    steps/filter/binary/statistical_outlier_filter/sf_step_statistical_outlier_removal.cpp \
    steps/filter/sf_abstract_filter_step.cpp \
    steps/filter/binary/sf_abstract_filter_binary_step.cpp \
    steps/filter/multiple/sf_abstract_filter_multiple_step.cpp \
    steps/filter/multiple/voxel/sf_filter_3d_grid_sub_cloud.cpp \
    steps/filter/binary/radius_outlier_filter/sf_radius_outlier_filter_step.cpp \
    steps/filter/binary/ground_filter/sf_step_ground_filter.cpp \
    steps/filter/binary/stem_filter/sf_step_stem_filter.cpp \
    pcl/geometry/sf_point_geometry.cpp \
    steps/dtm/sf_dtm_step.cpp \
    plot/sf_plot_model.cpp \
    plot/DTM/sf_dtm.cpp \
    plot/sf_raster_model.cpp \
    math/interpolation/sf_interpolation.cpp \
    converters/CT_To_PCL/sf_converter_ct_to_pcl_dtm.cpp

TRANSLATIONS += languages/pluginsimpleforest_en.ts \
                languages/pluginsimpleforest_fr.ts
