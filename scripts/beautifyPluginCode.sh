#!/bin/bash

for f in $(find /home/drsnuggles/Documents/computree/pluginSimpleForest/pluginsimpleforest/ -name '*.h' -or -name '*.hpp' -or -name '*.cpp');do clang-format -i -style=file $f; done
