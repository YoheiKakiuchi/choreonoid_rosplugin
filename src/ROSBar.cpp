#include "ROSBar.h"

using namespace cnoid;

ROSBar::ROSBar(ROSPlugin* plugin) : ToolBar("ROSBar")
{
  initialize(plugin);
}

void ROSBar::initialize(ROSPlugin* plugin)
{
  // SetupToolBar::initialize(dialog);
  addButton("ROS")->sigClicked().connect(boost::bind(&ROSPlugin::timer_callback, plugin));
  std::cerr << "ros bar initialized" << std::endl;

  setVisibleByDefault(true);// 効かない?
}
