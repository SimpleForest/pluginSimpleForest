#ifndef SF_PLUGINMANAGER_H
#define SF_PLUGINMANAGER_H

#include "ct_abstractstepplugin.h"
#include "ct_log/ct_fileloglistener.h"
class SF_PluginManager : public CT_AbstractStepPlugin
{
public:
  SF_PluginManager();
  ~SF_PluginManager();

  QString getPluginURL() const { return QString("http://rdinnovation.onf.fr/projects/PLUGINS-PROJECT-NAME-HERE/wiki"); }

  virtual QString getPluginOfficialName() const;
  virtual QString getPluginRISCitation() const;

  virtual bool init();

protected:
  bool loadGenericsStep();
  bool loadOpenFileStep();
  bool loadCanBeAddedFirstStep();
  bool loadFilters();
  bool loadMetrics();
  bool loadItemDrawables();

  bool loadActions();
  bool loadExporters();
  bool loadReaders();
};

#endif // SF_PLUGINMANAGER_H
