#pragma once

#include <cnoid/ToolBar>
#include <cnoid/Dialog>
#include <cnoid/SpinBox>
// #include <cnoid/Separator>
#include <cnoid/Buttons>
#include <cnoid/CheckBox>
#include <cnoid/LineEdit>
#include <QDialogButtonBox>

#include <cnoid/Archive>

#include "ROSPlugin.h"

namespace cnoid {
  class ROSPlugin;

  class ROSBar : public ToolBar
  {
  public:
    ROSBar(ROSPlugin* plugin);

    void initialize(ROSPlugin* plugin);
  };
}
