CT_PREFIX = ../../computreev5
DEFINES += COMPUTREE_V5

CHECK_CAN_USE_PCL = 1
CHECK_CAN_USE_OPENCV = 1
MUST_USE_USE_PCL = 1
MUST_USE_USE_GSL = 1
MUST_USE_OPENCV = 1

COMPUTREE += ctlibpcl ctliblas ctlibfilters ctlibmetrics ctlibstdactions

include($${CT_PREFIX}/shared.pri)
include($${CT_PREFIX}/include_ct_library.pri)
include($${PLUGIN_SHARED_DIR}/include.pri)

greaterThan(QT_MAJOR_VERSION, 4): QT += concurrent

QT += concurrent
QT +=  testlib
QT -= gui
CONFIG += c++11
CONFIG += testcase
CONFIG += console
CONFIG += qt console warn_on depend_includepath testcase
CONFIG -= app_bundle
TEMPLATE = app
SOURCES += \
    tests/sf_unittestqsmsort.cpp

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../ComputreeInstallRelease/plugins/release/ -lplug_simpleforest
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../ComputreeInstallRelease/plugins/debug/ -lplug_simpleforest
else:unix: LIBS += -L$$PWD/../../ComputreeInstallRelease/plugins/ -lplug_simpleforest

INCLUDEPATH += $$PWD/../../ComputreeInstallRelease/plugins
DEPENDPATH += $$PWD/../../ComputreeInstallRelease/plugins
