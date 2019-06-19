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
  ext->itemManager().registerClass<WorldRosItem>("WorldRosItem");
  ext->itemManager().addCreationPanel<WorldRosItem>();
}

WorldRosItem::WorldRosItem()
{
  hooked_simulators_.clear();
  post_dynamics_function_regid = -1;

  RootItem::instance()->sigTreeChanged().connect(boost::bind(&WorldRosItem::hookSimulationStartAndStopEvent, this));
}

WorldRosItem::WorldRosItem(const WorldRosItem& org)
  : Item(org)
{
  hooked_simulators_.clear();
  post_dynamics_function_regid = -1;

  RootItem::instance()->sigTreeChanged().connect(boost::bind(&WorldRosItem::hookSimulationStartAndStopEvent, this));
}

WorldRosItem::~WorldRosItem()
{
  stop();
}

bool WorldRosItem::store(Archive& archive)
{
  return true;
}

bool WorldRosItem::restore(const Archive& archive)
{
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
  if (! (world = this->findOwnerItem<WorldItem>())) {
    return;
  } else if (! (sim = SimulatorItem::findActiveSimulatorItemFor(this))) {
    return;
  }

  ROS_DEBUG("Found WorldItem: %s", world->name().c_str());
  ROS_DEBUG("Found SimulatorItem: %s", sim->name().c_str());

  double now = sim->currentTime();

  // rosnode_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(cnoidrospkg_parent_namespace_));

  // apply uppper limits
  post_dynamics_function_regid = sim->addPostDynamicsFunction(std::bind(&WorldRosItem::onPostDynamics, this));

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
  sim->startSimulation(true);
  return true;
}
////
void WorldRosItem::stop()
{
  if (post_dynamics_function_regid != -1) {
    sim->removePostDynamicsFunction(post_dynamics_function_regid);
    post_dynamics_function_regid = -1;
  }

  return;
}
