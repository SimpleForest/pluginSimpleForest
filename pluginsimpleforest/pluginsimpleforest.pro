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
CONFIG += c++11
CONFIG   += console
TARGET = plug_simpleforest

HEADERS += $${PLUGIN_SHARED_INTERFACE_DIR}/interfaces.h \
    cloud/filter/unary/maxIntensity/sf_maxintensity.h \
    cloud/filter/unary/sf_abstractUnaryFilter.h \
    cloud/sf_transferfeature.h \
    converters/CT_To_PCL/sf_converterCTIDToPCLCloud.h \
    converters/CT_To_PCL/sf_converterCTIDToPCLCloud.hpp \
    file/export/cloud/sf_exportCloud.h \
    file/export/ply/sf_exportPly.h \
    file/export/qsm/sf_exportQSM.h \
    file/export/sf_abstractExport.h \
    math/fit/line/sf_fitline.h \
    math/fit/line/sf_fitline.hpp \
    math/fit/line/sf_fitransacline.h \
    math/fit/line/sf_fitransacline.hpp \
    math/fit/power/sf_fitgnpower.h \
    math/fit/power/sf_fitgnpower.hpp \
    pcl/cloud/feature/djikstra/sf_djikstra.h \
    pcl/cloud/feature/djikstra/sf_djikstraVisitor.h \
    qsm/algorithm/distance/sf_extractFittedPoints.h \
    qsm/algorithm/distance/sf_extractFittedPoints.hpp \
    qsm/algorithm/optimization/recursion/sf_dijkstraLightRecursive.h \
    qsm/algorithm/optimization/recursion/sf_spherefollowingRecursive.h \
    qsm/algorithm/postprocessing/sf_QSMAllometricCorrectionParameterEstimation.h \
    qsm/algorithm/postprocessing/sf_QSMAllometryCorrectionNeighboring.h \
    qsm/algorithm/postprocessing/sf_QSMInversePipeModelParamaterEstimation.h \
    qsm/algorithm/postprocessing/sf_correctbranchjunction.h \
    qsm/algorithm/postprocessing/sf_qsmrefitcylinder.h \
    qsm/algorithm/visualization/sf_visualizefitquality.h \
    sf_pluginentry.h \
    sf_pluginmanager.h \
    pcl/sf_point.h \
    steps/filter/multiple/euclideanclustering/sf_euclideanClusteringSegmentationStep.h \
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
    math/interpolation/sf_interpolation.h \
    pcl/cloud/segmentation/dijkstra/sf_dijkstra.h \
    qsm/algorithm/detection/sf_idetection.h \
    qsm/model/sf_modelAbstractBuildingbrick.h \
    qsm/model/sf_modelCylinderBuildingbrick.h \
    qsm/model/sf_modelQSM.h \
    qsm/model/sf_modelAbstractSegment.h \
    qsm/build/sf_buildQSM.h \
    qsm/algorithm/sf_QSMAlgorithm.h \
    qsm/algorithm/sf_QSMCylinder.h \
    qsm/algorithm/distance/sf_cloudToModelDistance.h \
    qsm/algorithm/distance/sf_cloudToModelDistance.hpp \
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
    steps/item/sf_qsm_item.h \
    steps/item/sf_spherefollowing_parameters_item.h \
    steps/qsm/export/sf_stepExportQSMList.h \
    steps/qsm/modelling/sf_stepDijkstraLightRecursive.h \
    steps/qsm/modelling/sf_stepDijkstraLightRecursiveAdapter.h \
    steps/qsm/postprocessing/sf_stepQSMAllometricCorrection.h \
    steps/qsm/postprocessing/sf_stepQSMAllometricCorrectionAdapter.h \
    steps/qsm/modelling/sf_stepSpherefollowingAdvanced.h \
    steps/qsm/modelling/sf_stepSpherefollowingAdvancedAdapter.h \
    steps/qsm/modelling/sf_stepSpherefollowingBasic.h \
    steps/qsm/modelling/sf_stepSpherefollowingRecursive.h \
    steps/qsm/modelling/sf_stepSpherefollowingRecursiveAdapter.h \
    steps/qsm/postprocessing/sf_stepQSMCorrectBranchJunctions.h \
    steps/qsm/postprocessing/sf_stepQSMCorrectBranchJunctionsAdapter.h \
    steps/qsm/postprocessing/sf_stepQSMMedianFilter.h \
    steps/qsm/postprocessing/sf_stepQSMRefitCylinders.h \
    steps/qsm/postprocessing/sf_stepQSMRefitCylindersAdapter.h \
    steps/qsm/postprocessing/sf_stepReversePipeModelCorrection.h \
    steps/qsm/postprocessing/sf_stepreversepipemodelcorrectionadapter.h \
    steps/qsm/sf_abstractStepQSM.h \
    steps/segmentation/tree/sf_stepSegmentTreeCloudFromQSM.h \
    steps/segmentation/tree/sf_stepSegmentTreeCloudFromQSMAdapter.h \
    steps/sf_abstractStep.h \
    steps/manipulation/merge/sf_stepMergeClouds.h \
    steps/filter/sf_abstractFilterStep.h \
    steps/filter/multiple/voxel/sf_filter3dGridSubCloud.h \
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
    steps/qsm/modelling/sf_stepSpherefollowingBasicAdapter.h \
    steps/segmentation/sf_AbstractStepSegmentation.h \
    steps/segmentation/voronoi/sf_stepSegmentationVoronoi.h \
    steps/segmentation/dijkstra/sf_stepSegemtationDijkstra.h \
    steps/filter/multiple/euclideanclustering/sf_euclideanClusteringSegmentationStep.h \
    steps/dtm/sf_stepDTM.h \
    steps/param/sf_paramAllSteps.h \
    parameter/sf_abstractParameterSet.h \
    parameter/sf_parameterSetVoxelization.h \
    converters/CT_To_PCL/sf_converterCTCloudToPCLCloud.h \
    converters/CT_To_PCL/sf_converterCTCloudToPCLCloud.h \
    converters/CT_To_PCL/sf_converterCTCloudToPCLCloud.hpp \
    cloud/filter/sf_abstractfilter.h \
    cloud/sf_includestypedefs.h \
    cloud/filter/multiple/sf_abstractmultiplefilter.h \
    cloud/filter/multiple/voxel/sf_filtervoxelclustering.h \
    cloud/filter/multiple/voxel/sf_filtervoxelclustering.hpp \
    cloud/filter/binary/sf_abstractbinaryfilter.h \
    cloud/filter/binary/voxelgriddownscale/sf_voxelgriddownscale.h \
    cloud/filter/binary/voxelgriddownscale/sf_voxelgriddownscale.hpp \
    cloud/feature/principaldirection/sf_principaldirection.h \
    cloud/feature/principaldirection/sf_principaldirection.hpp \
    cloud/sf_abstractcloud.h \
    steps/feature/principaldirection/sf_stepprincipaldirection.h \
    steps/feature/sf_abstractstepfeature.h \
    cloud/filter/multiple/clusterscaling/sf_clustertransfer.h \
    cloud/filter/multiple/clusterscaling/sf_clustertransfer.hpp \
    parameter/sf_parametersetvoxelgriddownscale.h \
    parameter/sf_parameterSetVoxelgridDownscaling.h \
    parameter/sf_parameterSetPrincipalDirection.h \
    steps/feature/principaldirection/sf_adapterprincipaldirection.h \
    qsm/model/sf_modelLevelOne.h \
    qsm/algorithm/spherefollowing/sf_spherefollowingParameters.h \
    qsm/algorithm/spherefollowing/sf_spherefollowing.h \
    qsm/algorithm/geometry/circle/sf_circle.h \
    qsm/algorithm/geometry/circle/sf_circle.hpp \
    pcl/cloud/feature/descriptor/sf_descriptor.hpp \
    pcl/cloud/feature/descriptor/sf_descriptor.h \
    qsm/algorithm/optimization/gridsearch/sf_spherefollowingrastersearch.h \
    qsm/algorithm/cloudQSM/sf_clustercloudbyqsm.h \
    qsm/algorithm/cloudQSM/sf_clustercloudbyqsm.hpp \
    qsm/algorithm/optimization/downHillSimplex/sf_downhillsimplex.h \
    qsm/algorithm/postprocessing/sf_removefalseconnections.h \
    qsm/algorithm/postprocessing/sf_qsmmedianfilter.h \
    qsm/algorithm/postprocessing/sf_mergeonechildsegments.h \
    qsm/algorithm/postprocessing/sf_sortqsm.h \
    steps/sf_stepprogress.h \
    steps/visualization/sf_colorfactory.h \
    tests/factory/sf_qsmfactory.h

SOURCES += \
    file/export/cloud/sf_exportCloud.cpp \
    file/export/ply/sf_exportPly.cpp \
    file/export/qsm/sf_exportQSM.cpp \
    file/export/sf_abstractExport.cpp \
    pcl/cloud/feature/djikstra/sf_djikstra.cpp \
    pcl/cloud/feature/djikstra/sf_djikstraVisitor.cpp \
    pcl/cloud/segmentation/dijkstra/sf_dijkstra.cpp \
    qsm/algorithm/optimization/recursion/sf_dijkstraLightRecursive.cpp \
    qsm/algorithm/optimization/recursion/sf_spherefollowingRecursive.cpp \
    qsm/algorithm/postprocessing/sf_QSMAllometricCorrectionParameterEstimation.cpp \
    qsm/algorithm/postprocessing/sf_QSMAllometryCorrectionNeighboring.cpp \
    qsm/algorithm/postprocessing/sf_QSMInversePipeModelParamaterEstimation.cpp \
    qsm/algorithm/postprocessing/sf_correctbranchjunction.cpp \
    qsm/algorithm/postprocessing/sf_qsmrefitcylinder.cpp \
    qsm/algorithm/visualization/sf_visualizefitquality.cpp \
    sf_pluginentry.cpp \
    sf_pluginmanager.cpp \
    plot/DTM/sf_dtm.cpp \
    math/interpolation/sf_interpolation.cpp \
    qsm/model/sf_modelCylinderBuildingbrick.cpp \
    qsm/model/sf_modelAbstractSegment.cpp \
    qsm/model/sf_modelQSM.cpp \
    qsm/model/sf_modelAbstractBuildingbrick.cpp \
    qsm/build/sf_buildQSM.cpp \
    converters/sf_abstractConverter.cpp \
    converters/CT_To_PCL/sf_converterCTToPCLDTM.cpp \
    plot/sf_modelRaster.cpp \
    plot/sf_modelPlot.cpp \
    pcl/geometry/sf_pointGeometry.cpp \
    pcl/cloud/filter/binary/stem/sf_stemRansacFilter.cpp \
    steps/item/sf_qsm_item.cpp \
    steps/item/sf_spherefollowing_parameters_item.cpp \
    steps/qsm/export/sf_stepExportQSMList.cpp \
    steps/qsm/modelling/sf_stepDijkstraLightRecursive.cpp \
    steps/qsm/postprocessing/sf_stepQSMAllometricCorrection.cpp \
    steps/qsm/modelling/sf_stepSpherefollowingAdvanced.cpp \
    steps/qsm/modelling/sf_stepSpherefollowingBasic.cpp \
    steps/qsm/modelling/sf_stepSpherefollowingRecursive.cpp \
    steps/qsm/postprocessing/sf_stepQSMCorrectBranchJunctions.cpp \
    steps/qsm/postprocessing/sf_stepQSMMedianFilter.cpp \
    steps/qsm/postprocessing/sf_stepQSMRefitCylinders.cpp \
    steps/qsm/postprocessing/sf_stepReversePipeModelCorrection.cpp \
    steps/qsm/sf_abstractStepQSM.cpp \
    steps/segmentation/tree/sf_stepSegmentTreeCloudFromQSM.cpp \
    steps/sf_abstractStep.cpp \
    steps/filter/sf_abstractFilterStep.cpp \
    steps/filter/multiple/sf_abstractFilterMultipleStep.cpp \
    steps/filter/multiple/voxel/sf_filter3dGridSubCloud.cpp \
    steps/filter/binary/sf_abstractFilterBinaryStep.cpp \
    steps/filter/binary/stem_filter/sf_stepStemFilter.cpp \
    steps/filter/binary/stem_filter/sf_stepStemRANSACFilter.cpp \
    steps/manipulation/merge/sf_stepMergeClouds.cpp \
    steps/filter/binary/statistical_outlier_filter/sf_stepStatisticalOutlierRemoval.cpp \
    steps/filter/binary/radius_outlier_filter/sf_radiusOutlierFilterStep.cpp \
    steps/filter/binary/ground_filter/sf_stepGroundFilter.cpp \
    steps/filter/binary/cut_cloud_above_dtm/sf_stepCutCloudAboveDTM.cpp \
    steps/segmentation/sf_AbstractStepSegmentation.cpp \
    steps/segmentation/voronoi/sf_stepSegmentationVoronoi.cpp \
    steps/segmentation/dijkstra/sf_stepSegemtationDijkstra.cpp \
    steps/filter/multiple/euclideanclustering/sf_euclideanClusteringSegmentationStep.cpp \
    steps/dtm/sf_stepDTM.cpp \
    steps/feature/principaldirection/sf_stepprincipaldirection.cpp \
    steps/feature/sf_abstractstepfeature.cpp \
    qsm/model/sf_modelLevelOne.cpp \
    qsm/algorithm/spherefollowing/sf_spherefollowing.cpp \
    qsm/algorithm/optimization/gridsearch/sf_spherefollowingrastersearch.cpp \
    qsm/algorithm/optimization/downHillSimplex/sf_downhillsimplex.cpp \
    qsm/algorithm/postprocessing/sf_removefalseconnections.cpp \
    qsm/algorithm/postprocessing/sf_qsmmedianfilter.cpp \
    qsm/algorithm/postprocessing/sf_mergeonechildsegments.cpp \
    qsm/algorithm/postprocessing/sf_sortqsm.cpp \
    steps/sf_stepprogress.cpp \
    steps/visualization/sf_colorfactory.cpp \
    tests/factory/sf_qsmfactory.cpp

TRANSLATIONS += languages/pluginsimpleforest_en.ts \
                languages/pluginsimpleforest_fr.ts

unix:!macx: LIBS += -L/usr/lib/ -lgsl -lgslcblas -lm

