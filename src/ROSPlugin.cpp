#include "ROSPlugin.h"
#include "ROSBar.h"

#include <cnoid/SimulationBar>
#include <cnoid/ItemTreeView>
#include <cnoid/RootItem>

#include <random>

using namespace std;
using namespace cnoid;

ROSPlugin::ROSPlugin() : Plugin("ROS")
{
  require("Body");
}

bool ROSPlugin::initialize()
{
  std::cerr << "ROSPlugin::initialize" << std::endl;
  int argc = 0;
  char** argv = 0;
  bool primary_plugin = false;
  if(!ros::isInitialized()){
    ros::init(argc, argv, "choreonoid", ros::init_options::NoSigintHandler);
    primary_plugin = true;
  }

  if(!ros::master::check()){
    MessageView::instance()->putln(
                                   MessageView::WARNING, "The ROS master is not found.");
    return false;
  }


  mBar = new ROSBar(this);
  addToolBar(mBar);

  nh = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("choreonoid"));

  std::string reset_simulation_service_name("reset_simulation");
  ros::AdvertiseServiceOptions reset_simulation_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(reset_simulation_service_name,
                                                          boost::bind(&ROSPlugin::resetSimulation,this,_1,_2),
                                                          ros::VoidPtr(), &srv_queue);
  reset_simulation_service_ = nh->advertiseService(reset_simulation_aso);

  std::string stop_simulation_service_name("stop_simulation");
  ros::AdvertiseServiceOptions stop_simulation_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(stop_simulation_service_name,
                                                          boost::bind(&ROSPlugin::stopSimulation,this,_1,_2),
                                                          ros::VoidPtr(), &srv_queue);
  stop_simulation_service_ = nh->advertiseService(reset_simulation_aso);

  spinner.reset(new ros::AsyncSpinner(0));
  spinner->start();

  spinTimer.sigTimeout().connect([&](){ timer_callback(); });
  spinTimer.start(1.0);
  BodyPublisherItem::initialize(this);
  WorldRosItem::initialize(this);

  //// initialize ros service ...
  return true;
}

bool ROSPlugin::finalize()
{
  std::cerr << "final" << std::endl;
  //ros::requestShutdown();
  //ros::waitForShutdown();
  if (!!reset_simulation_service_) {
    ////
  }
}

bool ROSPlugin::stopSimulation(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr << "stp-cb" << std::endl;
  //sim = SimulatorItem::findActiveSimulatorItemFor(this))
  ItemList<SimulatorItem> simulators =
    ItemTreeView::mainInstance()->selectedItems<SimulatorItem>();
  std::cerr << "sim: " << simulators.size() << std::endl;
  for(size_t i=0; i < simulators.size(); ++i) {
    SimulatorItem* simulator = simulators.get(i);
    simulator->stopSimulation();
  }
  std::cerr << "stopped" << std::endl;
}

bool ROSPlugin::resetSimulation(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr << "reset-cb" << std::endl;

  ItemTreeView* itemTreeView = ItemTreeView::instance();
  ItemList<BodyItem> bodyItems;
  bodyItems.extractChildItems(RootItem::instance());
  BodyItem* bodyItem;
  for(size_t i=0; i < bodyItems.size(); ++i){
    BodyItem* bdI = bodyItems.get(i);
    std::cerr << "nm[" << i << "] : " << bdI->name() << std::endl;
    if (bdI->name() == "InvPendulum") { //// fixed string
      bodyItem = bdI;
    }
  }

  //ROS_WARN("start count = %d", start_count);
  std::cerr << "restore" << std::endl;
  if (!!bodyItem) {
    bodyItem->restoreInitialState(true);
    /// specific code
    std::random_device rnd;
    double rand = 0.5 * (rnd() / (double)std::random_device::max()) - 0.25;
    Link *lk = bodyItem->body()->rootLink();
    AngleAxis aa(rand ,Vector3d::UnitY());
    lk->setRotation(aa);
    Vector3d p(0, rand, 0.1);
    lk->setTranslation(p);
    ///
    bodyItem->storeInitialState();
  }
  std::cerr << "st" << std::endl;
  ItemList<SimulatorItem> simulators =
    ItemTreeView::mainInstance()->selectedItems<SimulatorItem>();
  std::cerr << "sim: " << simulators.size() << std::endl;
  for(size_t i=0; i < simulators.size(); ++i) {
    SimulatorItem* simulator = simulators.get(i);
    simulator->startSimulation(true);
  }
  std::cerr << "ed" << std::endl;

  return true;
}

void ROSPlugin::timer_callback()
{
  // std::cerr << "tm-cb" << std::endl;
  srv_queue.callAvailable(ros::WallDuration());
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(ROSPlugin)
