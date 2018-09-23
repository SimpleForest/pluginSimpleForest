#include "sf_pluginmanager.h"
#include "ct_stepseparator.h"
#include "ct_steploadfileseparator.h"
#include "ct_stepcanbeaddedfirstseparator.h"
#include "ct_actions/ct_actionsseparator.h"
#include "ct_exporter/ct_standardexporterseparator.h"
#include "ct_reader/ct_standardreaderseparator.h"
#include "ct_actions/abstract/ct_abstractaction.h"

#include "steps/filter/binary/statistical_outlier_filter/sf_stepStatisticalOutlierRemoval.h"
#include "steps/filter/binary/stem_filter/sf_stepStemFilter.h"
#include "steps/filter/binary/stem_filter/sf_stepStemRANSACFilter.h"
#include "steps/filter/binary/radius_outlier_filter/sf_radiusOutlierFilterStep.h"
#include "steps/filter/binary/ground_filter/sf_stepGroundFilter.h"
#include "steps/filter/binary/cut_cloud_above_dtm/sf_stepCutCloudAboveDTM.h"
#include "steps/filter/multiple/euclideanclustering/sf_euclideanClusteringSegmentationStep.h"
#include "steps/segmentation/dijkstra/sf_stepSegemtationDijkstra.h"
#include "steps/segmentation/voronoi/sf_stepSegmentationVoronoi.h"
#include "steps/dtm/sf_stepDTM.h"
#include "steps/qsm/modelling/sf_stepSpherefollowingRoot.h"

#include "steps/filter/multiple/voxel/sf_filter3dGridSubCloud.h"

// Inclure ici les entetes des classes definissant des Ã©tapes/actions/exporters ou readers

SF_PluginManager::SF_PluginManager() : CT_AbstractStepPlugin() {
}

SF_PluginManager::~SF_PluginManager() {
}

QString SF_PluginManager::getPluginOfficialName() const {
    return "SimpleForest";
}

QString SF_PluginManager::getPluginRISCitation() const {
    return             "TY  - JOUR\n"
                       "T1  - SimpleTree - an efficient open source tool to build tree models from TLS clouds\n"
                       "A1  - Hackenberg, Jan\n"
                       "A1  - Spiecker, Heinrich\n"
                       "A1  - Calders, Kim\n"
                       "A1  - Disney, Mathias\n"
                       "A1  - Raumonen, Pasi\n"
                       "JO  - Forests\n"
                       "VL  - 6\n"
                       "IS  - 11\n"
                       "SP  - 4245\n"
                       "EP  - 4294\n"
                       "Y1  - 2015\n"
                       "PB  - Multidisciplinary Digital Publishing Institute\n"
                       "UL  - http://www.simpletree.uni-freiburg.de/\n"
                       "ER  - \n";
}

bool SF_PluginManager::init() {
    return CT_AbstractStepPlugin::init();
}

bool SF_PluginManager::loadGenericsStep() {
    addNewPointsStep<SF_StepStatisticalOutlierRemoval>(CT_StepsMenu::LP_Filter);
    addNewPointsStep<SF_RadiusOutlierFilterStep>(CT_StepsMenu::LP_Filter);
    addNewPointsStep<SF_StepStemFilter>(CT_StepsMenu::LP_Filter);
    addNewPointsStep<SF_StepStemRANSACFilter>(CT_StepsMenu::LP_Filter);
    addNewPointsStep<SF_StepGroundFilter>(CT_StepsMenu::LP_Filter);
    addNewPointsStep<SF_StepCutCloudAboveDTM>(CT_StepsMenu::LP_Filter);
    addNewPointsStep<SF_Filter3dGridSubCloud>(CT_StepsMenu::LP_Voxels);
    addNewPointsStep<SF_EuclideanClusteringSegmentationStep>(CT_StepsMenu::LP_Clusters);
    addNewPointsStep<SF_StepSegmentationDijkstra>(CT_StepsMenu::LP_Clusters);
    addNewPointsStep<SF_StepSegmentationVoronoi>(CT_StepsMenu::LP_Clusters);
    addNewRastersStep<SF_StepDTM>(CT_StepsMenu::LP_DEM);
    addNewGeometricalShapesStep<SF_StepSpherefollowingRoot> ("QSM");

    return true;
}

bool SF_PluginManager::loadOpenFileStep() {

    return true;
}

bool SF_PluginManager::loadCanBeAddedFirstStep() {

    return true;
}

bool SF_PluginManager::loadFilters() {

    return true;
}

bool SF_PluginManager::loadMetrics() {

    return true;
}

bool SF_PluginManager::loadItemDrawables() {

    return true;
}

bool SF_PluginManager::loadActions() {

    return true;
}

bool SF_PluginManager::loadExporters() {

    return true;
}

bool SF_PluginManager::loadReaders() {

    return true;
}

