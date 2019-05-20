#include "BodyPublisherItem.h"
#include <cnoid/BodyItem>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/Body>
#include <cnoid/Device>
#include <cnoid/DeviceList>
#include <cnoid/Camera>
#include <cnoid/ItemManager>
#include <cnoid/TimeBar>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <memory>
#include "gettext.h"

#include "cnoid_robot_hardware.h"
// clock
#include "rosgraph_msgs/Clock.h"

using namespace std;
using namespace cnoid;

namespace {

class BodyNode
{
public:
    unique_ptr<ros::NodeHandle> rosNode;
    BodyItem* bodyItem;
    ScopedConnectionSet connections;
    ScopedConnection connectionOfKinematicStateChange;
    ScopedConnectionSet sensorConnections;
    TimeBar* timeBar;

    Body* ioBody;
    double time;
    double timeToPublishNext;
    double minPublishCycle;
    double timeStep;
    
    ros::Publisher jointStatePublisher;
    sensor_msgs::JointState jointState;

    DeviceList<Camera> cameras;
    vector<image_transport::Publisher> cameraImagePublishers;
    
    BodyNode(BodyItem* bodyItem);

    void start(ControllerIO* io, double maxPublishRate);
    void input();    
    void control();    
    void output();    
    void stop();    
    
    void startToPublishKinematicStateChangeOnGUI();
    void stopToPublishKinematicStateChangeOnGUI();
    void initializeJointState(Body* body);
    void publishJointState(Body* body, double time);
    void publishCameraImage(int index);

    DeviceList<RateGyroSensor> gyroSensors;
    DeviceList<AccelerationSensor> accelSensors;

    vector<ros::Publisher> gyroPublishers;
    vector<ros::Publisher> accelPublishers;
    void publishGyro(int index);
    void publishAccel(int index);

    //
    ros::Publisher clock_pub;
    // ros control
    cnoid_robot_hardware::CnoidRobotHW *cnoid_hw_;
    controller_manager::ControllerManager *ros_cm_;
};

}

namespace cnoid {

class BodyPublisherItemImpl
{
public:
    BodyPublisherItem* self;
    unique_ptr<BodyNode> bodyNode;
    ControllerIO* io;
    double maxPublishRate;
    
    BodyPublisherItemImpl(BodyPublisherItem* self);
    BodyPublisherItemImpl(BodyPublisherItem* self, const BodyPublisherItemImpl& org);
    ~BodyPublisherItemImpl();
    void setBodyItem(BodyItem* bodyItem, bool forceUpdate);
};

}


void BodyPublisherItem::initialize(ExtensionManager* ext)
{
    ext->itemManager().registerClass<BodyPublisherItem>("BodyPublisherItem");
    ext->itemManager().addCreationPanel<BodyPublisherItem>();
}


BodyPublisherItem::BodyPublisherItem()
{
    impl = new BodyPublisherItemImpl(this);
}


BodyPublisherItemImpl::BodyPublisherItemImpl(BodyPublisherItem* self)
    : self(self)
{
    io = nullptr;
    maxPublishRate = 30.0;
}


BodyPublisherItem::BodyPublisherItem(const BodyPublisherItem& org)
    : ControllerItem(org)
{
    impl = new BodyPublisherItemImpl(this, *org.impl);
}
    

BodyPublisherItemImpl::BodyPublisherItemImpl(BodyPublisherItem* self, const BodyPublisherItemImpl& org)
    : self(self)
{
    io = nullptr;
    maxPublishRate = org.maxPublishRate;
}


BodyPublisherItem::~BodyPublisherItem()
{
    delete impl;
}


BodyPublisherItemImpl::~BodyPublisherItemImpl()
{

}


Item* BodyPublisherItem::doDuplicate() const
{
    return new BodyPublisherItem(*this);
}


void BodyPublisherItem::onPositionChanged()
{
    impl->setBodyItem(findOwnerItem<BodyItem>(), false);
}


void BodyPublisherItemImpl::setBodyItem(BodyItem* bodyItem, bool forceUpdate)
{
    if(bodyNode){
        if(forceUpdate || bodyItem != bodyNode->bodyItem){
            bodyNode.reset();
        }
    }
    
    if(bodyItem && !bodyNode){
        bodyNode.reset(new BodyNode(bodyItem));

        bodyNode->connections.add(
            bodyItem->sigNameChanged().connect(
                [&](const std::string& oldName){ setBodyItem(bodyItem, true); }));
    }
}


void BodyPublisherItem::onDisconnectedFromRoot()
{
    impl->bodyNode.reset();
}


double BodyPublisherItem::timeStep() const
{
    return 0.0;
}


bool BodyPublisherItem::initialize(ControllerIO* io)
{
    impl->io = io;
    return true;
}


bool BodyPublisherItem::start()
{
    impl->bodyNode->start(impl->io, impl->maxPublishRate);
    return true;
}


void BodyPublisherItem::input()
{
    impl->bodyNode->input();
}


bool BodyPublisherItem::control()
{
    impl->bodyNode->control();
    return true;
}


void BodyPublisherItem::output()
{
    impl->bodyNode->output();
}


void BodyPublisherItem::stop()
{
    impl->bodyNode->stop();
    impl->io = nullptr;
}


void BodyPublisherItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Max publish rate"), impl->maxPublishRate, changeProperty(impl->maxPublishRate));
}


bool BodyPublisherItem::store(Archive& archive)
{
    archive.write("maxPublishRate", impl->maxPublishRate);
    return true;
}


bool BodyPublisherItem::restore(const Archive& archive)
{
    archive.read("maxPublishRate", impl->maxPublishRate);
    return true;
}


BodyNode::BodyNode(BodyItem* bodyItem)
    : bodyItem(bodyItem),
      timeBar(TimeBar::instance())
{
    string name = bodyItem->name();
    std::replace(name.begin(), name.end(), '-', '_');

    rosNode.reset(new ros::NodeHandle(name));

    jointStatePublisher = rosNode->advertise<sensor_msgs::JointState>("orig_joint_state", 1000);
    startToPublishKinematicStateChangeOnGUI();

    auto body = bodyItem->body();
    DeviceList<> devices = body->devices();

    cameras.assign(devices.extract<Camera>());
    image_transport::ImageTransport it(*rosNode);
    cameraImagePublishers.resize(cameras.size());
    for(size_t i=0; i < cameras.size(); ++i){
        auto camera = cameras[i];
        cameraImagePublishers[i] = it.advertise(camera->name() + "/image", 1);
    }

    gyroSensors.assign(devices.extract<RateGyroSensor> ());
    gyroPublishers.resize(gyroSensors.size());
    for(size_t i = 0; i < gyroSensors.size(); ++i) {
        auto gyro = gyroSensors[i];
        std::string name = gyro->name();
        gyroPublishers[i] = rosNode->advertise<sensor_msgs::Imu>(name, 1);
    }
    accelSensors.assign(devices.extract<AccelerationSensor> ());
    accelPublishers.resize(accelSensors.size());
    for(size_t i = 0; i < accelSensors.size(); ++i) {
        auto accel = accelSensors[i];
        std::string name = accel->name();
        accelPublishers[i] = rosNode->advertise<sensor_msgs::Imu>(name, 1);
    }

    /// clock
    clock_pub = rosNode->advertise<rosgraph_msgs::Clock>("/clock", 5);
    /// trajectory
    cnoid_hw_ = new cnoid_robot_hardware::CnoidRobotHW();
    cnoid_hw_->cnoid_body = bodyItem->body();
    //ros::NodeHandle nh;
    ros::NodeHandle robot_nh("~");
    if (!cnoid_hw_->init(*rosNode, robot_nh)) {
      ROS_ERROR("Faild to initialize hardware");
      //exit(1);
    }
    ros_cm_ = new controller_manager::ControllerManager (cnoid_hw_, *rosNode);
}


void BodyNode::start(ControllerIO* io, double maxPublishRate)
{
    cnoid_hw_->cnoid_body = io->body();

    ioBody = io->body();
    time = 0.0;
    minPublishCycle = maxPublishRate > 0.0 ? (1.0 / maxPublishRate) : 0.0;
    timeToPublishNext = minPublishCycle;
    timeStep = io->timeStep();

    stopToPublishKinematicStateChangeOnGUI();
    initializeJointState(ioBody);

    sensorConnections.disconnect();
    DeviceList<> devices = ioBody->devices();

    cameras.assign(devices.extract<Camera>());
    for(size_t i=0; i < cameras.size(); ++i){
        auto camera = cameras[i];
        sensorConnections.add(
            camera->sigStateChanged().connect(
                [&, i](){ publishCameraImage(i); }));
    }

    gyroSensors.assign(devices.extract<RateGyroSensor> ());
    for(size_t i=0; i < gyroSensors.size(); ++i){
      auto gyro = gyroSensors[i];
        sensorConnections.add(
            gyro->sigStateChanged().connect(
                [&, i](){ publishGyro(i); }));
    }

    accelSensors.assign(devices.extract<AccelerationSensor> ());
    for(size_t i=0; i < accelSensors.size(); ++i){
      auto accel = accelSensors[i];
        sensorConnections.add(
            accel->sigStateChanged().connect(
                [&, i](){ publishAccel(i); }));
    }
}


void BodyNode::input()
{
    timeToPublishNext += timeStep;
    if(timeToPublishNext > minPublishCycle){
        publishJointState(ioBody, time);
        timeToPublishNext -= minPublishCycle;
    }
}


void BodyNode::control()
{
    time += timeStep;

    ros::Time now(time);
    ros::Duration period(timeStep);

    // clock pub
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = now;
    clock_pub.publish(clock_msg);

    //
    cnoid_hw_->read(now, period);
    // read q from choreonoid
    ros_cm_->update(now, period);
    // write tau to choreonoid
    cnoid_hw_->write(now, period);
}


void BodyNode::output()
{

}


void BodyNode::stop()
{
    stopToPublishKinematicStateChangeOnGUI();
    sensorConnections.disconnect();
}


void BodyNode::startToPublishKinematicStateChangeOnGUI()
{
    if(jointStatePublisher){
        auto body = bodyItem->body();
        initializeJointState(body);
        publishJointState(body, timeBar->time());
        connectionOfKinematicStateChange.reset(
            bodyItem->sigKinematicStateChanged().connect(
                [&](){ publishJointState(bodyItem->body(), timeBar->time()); }));
    }
}


void BodyNode::stopToPublishKinematicStateChangeOnGUI()
{
    connectionOfKinematicStateChange.disconnect();
}

        
void BodyNode::initializeJointState(Body* body)
{
    const int n = body->numJoints();
    jointState.name.resize(n);
    jointState.position.resize(n);
    jointState.velocity.resize(n);
    jointState.effort.resize(n);
    for(int i=0; i < n; ++i){
        jointState.name[i] = body->joint(i)->name();
    }
}
    

void BodyNode::publishJointState(Body* body, double time)
{
#if 0
    jointState.header.stamp.fromSec(time);

    for(int i=0; i < body->numJoints(); ++i){
        Link* joint = body->joint(i);
        jointState.position[i] = joint->q();
        jointState.velocity[i] = joint->dq();
        jointState.effort[i] = joint->u();
    }

    jointStatePublisher.publish(jointState);
#endif
}


void BodyNode::publishCameraImage(int index)
{
    auto camera = cameras[index];
    sensor_msgs::Image image;
    image.header.stamp.fromSec(time);
    image.header.frame_id = camera->name();
    image.height = camera->image().height();
    image.width = camera->image().width();
    if(camera->image().numComponents() == 3){
        image.encoding = sensor_msgs::image_encodings::RGB8;
    } else if (camera->image().numComponents() == 1){
        image.encoding = sensor_msgs::image_encodings::MONO8;
    } else {
        ROS_WARN("unsupported image component number: %i", camera->image().numComponents());
    }
    image.is_bigendian = 0;
    image.step = camera->image().width() * camera->image().numComponents();
    image.data.resize(image.step * image.height);
    std::memcpy(&(image.data[0]), &(camera->image().pixels()[0]), image.step * image.height);
    cameraImagePublishers[index].publish(image);
}

void BodyNode::publishGyro(int index)
{
  auto gyro = gyroSensors[index];

  sensor_msgs::Imu msg;
  msg.header.stamp.fromSec(time);
  msg.header.frame_id = gyro->name();
  msg.angular_velocity.x = gyro->w()[0];
  msg.angular_velocity.y = gyro->w()[1];
  msg.angular_velocity.z = gyro->w()[2];

  gyroPublishers[index].publish(msg);
}

void BodyNode::publishAccel(int index)
{
  auto accel = accelSensors[index];

  sensor_msgs::Imu msg;
  msg.header.stamp.fromSec(time);
  msg.header.frame_id = accel->name();
  msg.linear_acceleration.x = accel->dv()[0];
  msg.linear_acceleration.y = accel->dv()[1];
  msg.linear_acceleration.z = accel->dv()[2];

  accelPublishers[index].publish(msg);
}
