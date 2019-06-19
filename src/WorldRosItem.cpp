/**
   @file WorldRosItem.cpp
   @author
 */

#include "WorldRosItem.h"

#include <cnoid/BodyItem>
#include <cnoid/RootItem>
#include <cnoid/Link>
#include <cnoid/Sensor>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/ItemTreeView>
#include <cnoid/EigenUtil>
#include <fstream>

using namespace cnoid;

/*
  Namepsace of parent of topics and services.
  NOTE: If you renaming, do not include '-'.
 */
const static std::string cnoidrospkg_parent_namespace_ = "choreonoid";

void WorldRosItem::initialize(ExtensionManager* ext)
{
  std::cerr << "ROS initialize" << std::endl;
  ext->itemManager().registerClass<WorldRosItem>("WorldRosItem");
  ext->itemManager().addCreationPanel<WorldRosItem>();
}

WorldRosItem::WorldRosItem()
{
  ROS_WARN("created 0");
  hooked_simulators_.clear();
  post_dynamics_function_regid = -1;

  RootItem::instance()->sigTreeChanged().connect(boost::bind(&WorldRosItem::hookSimulationStartAndStopEvent, this));

  //if (ros::ok() && !nh) {
  //  startROS();
  //}
}

WorldRosItem::WorldRosItem(const WorldRosItem& org)
  : Item(org)
{
  ROS_WARN("created 1");
  hooked_simulators_.clear();
  post_dynamics_function_regid = -1;

  RootItem::instance()->sigTreeChanged().connect(boost::bind(&WorldRosItem::hookSimulationStartAndStopEvent, this));
}

WorldRosItem::~WorldRosItem()
{
  ROS_WARN("dispose");
  stop();
}

bool WorldRosItem::store(Archive& archive)
{
  ROS_WARN("store");
  return true;
}

bool WorldRosItem::restore(const Archive& archive)
{
  ROS_WARN("restore");
  return true;
}

Item* WorldRosItem::doDuplicate() const
{
  return new WorldRosItem(*this);
}

void WorldRosItem::doPutProperties(PutPropertyFunction& putProperty)
{
  return;
}

void WorldRosItem::hookSimulationStartAndStopEvent()
{
  WorldItemPtr parent;
  std::map<std::string, bool>::iterator it;

  if (! (parent = this->findOwnerItem<WorldItem>())) {
    return;
  }

  for (Item* child = parent->childItem(); child; child = child->nextItem()) {
    SimulatorItemPtr p = dynamic_cast<SimulatorItem*>(child);

    if (p && ! p->isRunning()) {
      it = hooked_simulators_.find(p->name());
      if (it == hooked_simulators_.end()) {
        p->sigSimulationStarted().connect(std::bind(&WorldRosItem::start, this));
        p->sigSimulationFinished().connect(std::bind(&WorldRosItem::stop, this));
        hooked_simulators_[p->name()] = true;
        ROS_DEBUG("Hooked simulator %s", p->name().c_str());
      } else {
        ROS_DEBUG("Simulator %s is already hooked", it->first.c_str());
      }
    }
  }

  return;
}

void WorldRosItem::start()
{
  ROS_WARN("start");
  if (! (world = this->findOwnerItem<WorldItem>())) {
    return;
  } else if (! (sim = SimulatorItem::findActiveSimulatorItemFor(this))) {
    return;
  }
  if (ros::ok() && !nh) {
    startROS();
  }
  // apply uppper limits
  post_dynamics_function_regid = sim->addPostDynamicsFunction(std::bind(&WorldRosItem::onPostDynamics, this));
}

bool WorldRosItem::startROS()
{
  //ROS_WARN("Found WorldItem: %s", world->name().c_str());
  //ROS_WARN("Found SimulatorItem: %s", sim->name().c_str());
  ROS_WARN("startROS");

  // rosnode_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(cnoidrospkg_parent_namespace_));
  nh = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("choreonoid"));

  std::string pause_physics_service_name("pause_physics");
  ros::AdvertiseServiceOptions pause_physics_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          pause_physics_service_name,
                                                          boost::bind(&WorldRosItem::pausePhysics,this,_1,_2),
                                                          ros::VoidPtr(), ros::getGlobalCallbackQueue());
  pause_physics_service_ = nh->advertiseService(pause_physics_aso);

  std::string unpause_physics_service_name("unpause_physics");
  ros::AdvertiseServiceOptions unpause_physics_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          unpause_physics_service_name,
                                                          boost::bind(&WorldRosItem::unpausePhysics,this,_1,_2),
                                                          ros::VoidPtr(), ros::getGlobalCallbackQueue());
  unpause_physics_service_ = nh->advertiseService(unpause_physics_aso);

  std::string reset_simulation_service_name("reset_simulation");
  ros::AdvertiseServiceOptions reset_simulation_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          reset_simulation_service_name,
                                                          boost::bind(&WorldRosItem::resetSimulation,this,_1,_2),
                                                          ros::VoidPtr(), ros::getGlobalCallbackQueue());
  reset_simulation_service_ = nh->advertiseService(reset_simulation_aso);

  //async_ros_spin_.reset(new ros::AsyncSpinner(0));
  //async_ros_spin_->start();
}

void WorldRosItem::onPostDynamics()
{
  //
}

bool WorldRosItem::pausePhysics(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  sim->pauseSimulation();
  return true;
}

bool WorldRosItem::unpausePhysics(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  sim->restartSimulation();
  return true;
}

bool WorldRosItem::resetSimulation(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  if (!sim) {
    ROS_WARN("not sim");
  }
  sim->startSimulation(true);
  return true;
}
////
void WorldRosItem::stop()
{
  ROS_WARN("stop");
  if (post_dynamics_function_regid != -1) {
    sim->removePostDynamicsFunction(post_dynamics_function_regid);
    post_dynamics_function_regid = -1;
  }
  // stop ROS ???
  return;
}
