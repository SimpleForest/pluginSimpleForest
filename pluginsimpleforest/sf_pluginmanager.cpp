#include "sf_pluginmanager.h"
#include "ct_stepseparator.h"
#include "ct_steploadfileseparator.h"
#include "ct_stepcanbeaddedfirstseparator.h"
#include "ct_actions/ct_actionsseparator.h"
#include "ct_exporter/ct_standardexporterseparator.h"
#include "ct_reader/ct_standardreaderseparator.h"
#include "ct_actions/abstract/ct_abstractaction.h"

#include "steps/filter/binary/statistical_outlier_filter/sf_step_statistical_outlier_removal.h"
#include "steps/filter/binary/stem_filter/sf_step_stem_filter.h"
#include "steps/filter/binary/radius_outlier_filter/sf_radius_outlier_filter_step.h"
#include "steps/filter/binary/ground_filter/sf_step_ground_filter.h"
#include "steps/dtm/sf_dtm_step.h"

#include "steps/filter/multiple/voxel/sf_filter_3d_grid_sub_cloud.h"

// Inclure ici les entetes des classes definissant des Ã©tapes/actions/exporters ou readers

SF_PluginManager::SF_PluginManager() : CT_AbstractStepPlugin()
{
}

SF_PluginManager::~SF_PluginManager()
{
}

QString SF_PluginManager::getPluginOfficialName() const
{
    return "SimpleForest";
}

QString SF_PluginManager::getPluginRISCitation() const
{
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
//    QStringList list;

//    list.append(QString("TY  - COMP\n"
//              "TI  - Plugin SimpleForest for Computree\n"
//              "AU  - Hackenberg, Jan\n"
//              "PB  - SimpleTree project\n"
//              "PY  - 2017\n"
//              "UR  - http://rdinnovation.onf.fr/projects/simpleforest\n"
//              "ER  - \n"));

//    list.append(QString("TY  - JOUR\n"
//                   "T1  - SimpleTree - an efficient open source tool to build tree models from TLS clouds\n"
//                   "A1  - Hackenberg, Jan\n"
//                   "A1  - Spiecker, Heinrich\n"
//                   "A1  - Calders, Kim\n"
//                   "A1  - Disney, Mathias\n"
//                   "A1  - Raumonen, Pasi\n"
//                   "JO  - Forests\n"
//                   "VL  - 6\n"
//                   "IS  - 11\n"
//                   "SP  - 4245\n"
//                   "EP  - 4294\n"
//                   "Y1  - 2015\n"
//                   "PB  - Multidisciplinary Digital Publishing Institute\n"
//                   "UL  - http://www.simpletree.uni-freiburg.de/\n"
//                   "ER  - \n"));

//    list.append(QString("TY  - CONF\n"
//                        "T1  - 3d is here: Point cloud library (pcl)\n"
//                        "A1  - Rusu, Radu Bogdan\n"
//                        "A1  - Cousins, Steve\n"
//                        "JO  - Robotics and Automation (ICRA), 2011 IEEE International Conference on\n"
//                        "SP  - 1\n"
//                        "EP  - 4\n"
//                        "SN  - 1612843859\n"
//                        "Y1  - 2011\n"
//                        "PB  - IEEE\n"
//                        "UL  - http://pointclouds.org\n"
//                        "ER  - \n"));


//    return list;
}

bool SF_PluginManager::init()
{
    return CT_AbstractStepPlugin::init();
}

bool SF_PluginManager::loadGenericsStep()
{
    addNewPointsStep<SF_Step_Statistical_Outlier_Removal>(CT_StepsMenu::LP_Filter);
    addNewPointsStep<SF_Radius_Outlier_Filter_Step>(CT_StepsMenu::LP_Filter);
    addNewPointsStep<SF_Step_Stem_Filter>(CT_StepsMenu::LP_Filter);
    addNewPointsStep<SF_Step_Ground_Filter>(CT_StepsMenu::LP_Filter);
    addNewPointsStep<SF_Filter_3d_Grid_Sub_Cloud>(CT_StepsMenu::LP_Voxels);
    addNewRastersStep<SF_DTM_Step>("SimpleForest");

    return true;
}

bool SF_PluginManager::loadOpenFileStep()
{

    return true;
}

bool SF_PluginManager::loadCanBeAddedFirstStep()
{

    return true;
}

bool SF_PluginManager::loadFilters()
{

    return true;
}

bool SF_PluginManager::loadMetrics()
{

    return true;
}

bool SF_PluginManager::loadItemDrawables()
{

    return true;
}

bool SF_PluginManager::loadActions()
{

    return true;
}

bool SF_PluginManager::loadExporters()
{

    return true;
}

bool SF_PluginManager::loadReaders()
{

    return true;
}

