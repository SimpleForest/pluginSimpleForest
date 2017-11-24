#include "sf_pluginmanager.h"
#include "ct_stepseparator.h"
#include "ct_steploadfileseparator.h"
#include "ct_stepcanbeaddedfirstseparator.h"
#include "ct_actions/ct_actionsseparator.h"
#include "ct_exporter/ct_standardexporterseparator.h"
#include "ct_reader/ct_standardreaderseparator.h"
#include "ct_actions/abstract/ct_abstractaction.h"

#include "steps/filter/binary/sf_step_statistical_outlier_removal.h"

// Inclure ici les entetes des classes definissant des Ã©tapes/actions/exporters ou readers

SF_PluginManager::SF_PluginManager() : CT_AbstractStepPlugin()
{
}

SF_PluginManager::~SF_PluginManager()
{
}

bool SF_PluginManager::loadGenericsStep()
{
    CT_StepSeparator *sep = addNewSeparator(new CT_StepSeparator());

    sep->addStep( new SF_Step_Statistical_Outlier_Removal(*createNewStepInitializeData(NULL)));
    // Ajouter ici les etapes
    //sep->addStep(new NomDeLEtape(*createNewStepInitializeData(NULL)));

    return true;
}

bool SF_PluginManager::loadOpenFileStep()
{
    clearOpenFileStep();

    CT_StepLoadFileSeparator *sep = addNewSeparator(new CT_StepLoadFileSeparator(("TYPE DE FICHIER")));
    //sep->addStep(new NomDeLEtape(*createNewStepInitializeData(NULL)));

    return true;
}

bool SF_PluginManager::loadCanBeAddedFirstStep()
{
    clearCanBeAddedFirstStep();

    CT_StepCanBeAddedFirstSeparator *sep = addNewSeparator(new CT_StepCanBeAddedFirstSeparator());
    //sep->addStep(new NomDeLEtape(*createNewStepInitializeData(NULL)));

    return true;
}

bool SF_PluginManager::loadActions()
{
    clearActions();

    CT_ActionsSeparator *sep = addNewSeparator(new CT_ActionsSeparator(CT_AbstractAction::TYPE_SELECTION));
    //sep->addAction(new NomDeLAction());

    return true;
}

bool SF_PluginManager::loadExporters()
{
    clearExporters();

    CT_StandardExporterSeparator *sep = addNewSeparator(new CT_StandardExporterSeparator("TYPE DE FICHIER"));
    //sep->addExporter(new NomDeLExporter());

    return true;
}

bool SF_PluginManager::loadReaders()
{
    clearReaders();

    CT_StandardReaderSeparator *sep = addNewSeparator(new CT_StandardReaderSeparator("TYPE DE FICHIER"));
    //sep->addReader(new NomDuReader());

    return true;
}

