CT_PREFIX = ../../computreev3

exists(../../computreev5) {
    CT_PREFIX = ../../computreev5
    DEFINES += COMPUTREE_V5
}

MUST_USE_PCL = 1
MUST_USE_OPENCV = 1

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
    steps/abstract_step/sf_abstract_step.h \
    steps/abstract_param/sf_abstract_param.h \
    steps/abstract_step/sf_template_step.h \
    pcl/cloud/filter/sf_abstract_filter.h \
    pcl/cloud/filter/binary/sf_binary_filter.h \
    pcl/cloud/sf_abstract_cloud.hpp \
    pcl/cloud/sf_abstract_cloud.h \
    pcl/cloud/filter/sf_abstract_filter.hpp \
    pcl/cloud/filter/binary/sf_binary_filter.hpp \
    pcl/cloud/filter/binary/statisticaloutlier/sf_statistical_outlier_filter.h \
    pcl/cloud/filter/binary/statisticaloutlier/sf_statistical_outlier_filter.hpp \
    steps/filter/binary/sf_step_statistical_outlier_removal.h \
    converters/CT_To_PCL/sf_converter_ct_to_pcl.h \
    converters/CT_To_PCL/sf_converter_ct_to_pcl.hpp \
    converters/sf_abstract_converter.h \
    steps/abstract_step/pcl/sf_abstract_pcl_step.h \
    steps/abstract_step/pcl/sf_abstract_pcl_step.hpp \
    steps/filter/binary/sf_step_statistical_outlier_removal_adapter.h
SOURCES += \
    sf_pluginentry.cpp \
    sf_pluginmanager.cpp \
    qsm/sf_model_abstract_segment.cpp \
    qsm/sf_model_abstract_buildingbrick.cpp \
    steps/abstract_step/sf_abstract_step.cpp \
    steps/abstract_step/sf_template_step.cpp \
    steps/filter/binary/sf_step_statistical_outlier_removal.cpp \
    converters/sf_abstract_converter.cpp \
    steps/abstract_step/pcl/sf_abstract_pcl_step.cpp \
    steps/filter/binary/sf_step_statistical_outlier_removal_adapter.cpp

TRANSLATIONS += languages/pluginsimpleforest_en.ts \
                languages/pluginsimpleforest_fr.ts
