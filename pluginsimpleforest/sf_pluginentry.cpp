#include "sf_pluginentry.h"
#include "sf_pluginmanager.h"

SF_PluginEntry::SF_PluginEntry()
{
    _pluginManager = new SF_PluginManager();
}

SF_PluginEntry::~SF_PluginEntry()
{
    delete _pluginManager;
}

QString SF_PluginEntry::getVersion() const
{
    return "1.0";
}

CT_AbstractStepPlugin* SF_PluginEntry::getPlugin() const
{
    return _pluginManager;
}

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
    Q_EXPORT_PLUGIN2(plug_simpleforest, SF_PluginEntry)
#endif
