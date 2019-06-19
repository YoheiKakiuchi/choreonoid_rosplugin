/**
   @file WorldRosItem.h
   @author
 */

#ifndef CNOID_ROS_PLUGIN_WORLD_ROS_ITEM_H_INCLUDED
#define CNOID_ROS_PLUGIN_WORLD_ROS_ITEM_H_INCLUDED

#include <cnoid/Item>
#include <cnoid/Body>
#include <cnoid/DyBody>
#include <cnoid/WorldItem>
#include <cnoid/SimulatorItem>
#include <cnoid/AISTSimulatorItem>
#include <cnoid/TimeBar>
#include "exportdecl.h"

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Wrench.h>
#include <std_srvs/Empty.h>
#include <rosgraph_msgs/Clock.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ContactsState.h>

#include <vector>
#include <boost/thread.hpp>

namespace cnoid {

/**
   @brief This class is for accessing protected 'getCollisions' method in the SimulatorItem class.
   The use of this class is limited to this only. How to use, please see following example.

   @code
   SimulatorItemPtr               p;
   WorldRosSimulatorItemAccessor* sim_access;

   p          = <set SimulatorItem's instance>;
   sim_access = static_cast<WorldRosSimulatorItemAccessor*>(p.get());

   CollisionsLinkPairListPtr link_pairs = sim_access->get_collisions();

   if (link_pairs) {
     // This physics engine are collision output supported.
   } else {
     // This physics engine are not collision output supported.
   }
   @endcode

   @attention This class does not consider usage other than the contents described in the explanation.
 */
class CNOID_EXPORT WorldRosSimulatorItemAccessor : public SimulatorItem
{
public:
  WorldRosSimulatorItemAccessor() { }
  CollisionLinkPairListPtr get_collisions() { return getCollisions(); }
  virtual SimulationBody* createSimulationBody(Body* orgBody) { return 0; }
  virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies) { return true; }
  virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies) { return true; }
};

typedef ref_ptr<WorldRosSimulatorItemAccessor> WorldRosSimulatorItemAccessorPtr;

/**
   @brief This class is provides all services and some topics (link state, model state, contact state).
 */
class CNOID_EXPORT WorldRosItem : public Item
{
public:
    static void initialize(ExtensionManager* ext);

    WorldRosItem();
    WorldRosItem(const WorldRosItem& org);
    virtual ~WorldRosItem();

    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

    void start();
    void stop();

    void setModuleName(const std::string& name);

protected:
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);

private:
    WorldItemPtr world;
    SimulatorItemPtr sim;
    boost::shared_ptr<ros::NodeHandle>   nh;

    /// The registration id of calling function from physics engine. (physics engine is SimulatorItem's subclass)
    int post_dynamics_function_regid;

    std::map<std::string, bool> hooked_simulators_;

    /**
      @brief Hook simulation start and stop event
     */
    void hookSimulationStartAndStopEvent();

    void onPostDynamics();

    /*
      For services and topics.
     */

    bool resetSimulation(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool pausePhysics(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool unpausePhysics(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    ros::ServiceServer reset_simulation_service_;
    //ros::ServiceServer reset_world_service_;
    ros::ServiceServer pause_physics_service_;
    ros::ServiceServer unpause_physics_service_;
    /// For getting collision data.
    WorldRosSimulatorItemAccessor* sim_access_;
};

typedef ref_ptr<WorldRosItem> WorldRosItemPtr;

}
#endif /* #ifndef CNOID_ROS_PLUGIN_WORLD_ROS_ITEM_H_INCLUDED */
