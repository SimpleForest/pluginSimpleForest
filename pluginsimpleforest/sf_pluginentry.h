#ifndef SF_PLUGINENTRY_H
#define SF_PLUGINENTRY_H

#include "interfaces.h"

class SF_PluginManager;

class SF_PluginEntry : public PluginEntryInterface
{
  Q_OBJECT

#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
  Q_PLUGIN_METADATA(IID PluginEntryInterface_iid)
#endif

  Q_INTERFACES(PluginEntryInterface)

public:
  SF_PluginEntry();
  ~SF_PluginEntry();

  QString getVersion() const;
  CT_AbstractStepPlugin* getPlugin() const;

private:
  SF_PluginManager* _pluginManager;
};

#endif // SF_PLUGINENTRY_H