#pragma once

#include "BodyPublisherItem.h"
#include "URDFSaveItem.h"
//
#include <cnoid/Plugin>
#include <cnoid/MessageView>
#include <cnoid/Timer>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <rosgraph_msgs/Clock.h>
#include <ros/spinner.h>

#include <geometry_msgs/Wrench.h>
#include <std_srvs/Empty.h>

namespace cnoid {

class ROSBar;

class ROSPlugin : public Plugin
{
protected:
  ROSBar* mBar;
  std::unique_ptr<ros::AsyncSpinner> spinner;
  boost::shared_ptr<ros::NodeHandle> nh;

  ros::ServiceServer reset_simulation_service_;
  ros::ServiceServer stop_simulation_service_;

  ros::CallbackQueue srv_queue;
  Timer spinTimer;

public:
  ROSPlugin();
  virtual bool initialize();
  virtual bool finalize();
  void timer_callback();

  bool resetSimulation(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool stopSimulation(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
};

}
