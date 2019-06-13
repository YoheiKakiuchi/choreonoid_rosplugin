#include "BodyPublisherItem.h"
#include <cnoid/BodyItem>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/Body>
#include <cnoid/Device>
#include <cnoid/DeviceList>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/ItemManager>
#include <cnoid/TimeBar>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <memory>
#include "gettext.h"

#include "cnoid_robot_hardware.h"
// clock
#include "rosgraph_msgs/Clock.h"
#include "std_srvs/Empty.h"

using namespace std;
using namespace cnoid;

namespace {

class BodyNode
{
public:
    unique_ptr<ros::NodeHandle> rosNode;
    string name;
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
    DeviceList<RangeCamera> depth_cameras;
    vector<image_transport::Publisher> depth_cameraImagePublishers;
    vector<ros::Publisher> points_cameraImagePublishers;
    /// Range
    
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
    void createCameraImage(Camera *camera, sensor_msgs::Image &image);
    void createCameraImageDepth(RangeCamera *camera, sensor_msgs::PointCloud2 &points);
    DeviceList<RateGyroSensor> gyroSensors;
    DeviceList<AccelerationSensor> accelSensors;

    vector<ros::Publisher> gyroPublishers;
    vector<ros::Publisher> accelPublishers;
    void publishGyro(int index);
    void publishAccel(int index);

    //
    bool use_trajectory_controller;
    ros::ServiceServer control_toggle_service;
    bool toggle_controller(std_srvs::Empty::Request& req,
                           std_srvs::Empty::Response& res);
    //
    bool publish_clock;
    unsigned long clock_counter;
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
    name = bodyItem->name();
    std::replace(name.begin(), name.end(), '-', '_');

    rosNode.reset(new ros::NodeHandle(name));

    jointStatePublisher = rosNode->advertise<sensor_msgs::JointState>("orig_joint_state", 1000);
    startToPublishKinematicStateChangeOnGUI();

    auto body = bodyItem->body();
    DeviceList<> devices = body->devices();

    cameras.assign(devices.extract<Camera>());
    depth_cameras.assign(devices.extract<RangeCamera>());

    image_transport::ImageTransport it(*rosNode);
    cameraImagePublishers.resize(cameras.size());
    for(size_t i=0; i < cameras.size(); ++i) {
        Camera *camera = cameras[i];
        std::cerr << "cam: " << camera->name() << std::endl;
        if (!!camera) {
          cameraImagePublishers[i] = it.advertise(camera->name() + "/image", 1);
          RangeCamera* rcamera = dynamic_cast<RangeCamera*>(camera);
          std::cerr << "rcam: " << rcamera->name() << std::endl;
          if (!!rcamera) {
            depth_cameras.push_back(rcamera);
          }
        }
    }

    points_cameraImagePublishers.resize(depth_cameras.size());
    for(size_t i=0; i < depth_cameras.size(); ++i){
        auto camera = depth_cameras[i];
        std::cerr << "dcam: " << camera->name() << std::endl;
        points_cameraImagePublishers[i] = rosNode->advertise<sensor_msgs::PointCloud2>(camera->name() + "/points", 1);
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

    bool published_clock = false;
    std::vector<std::string> topics;
    ros::this_node::getAdvertisedTopics(topics);
    for (auto tp = topics.begin(); tp != topics.end(); tp++) {
      //ROS_ERROR("tp: %s", tp->c_str());
      if (*tp == "/clock") {
        published_clock = true;
      }
    }
    publish_clock = false;
    clock_counter = 0;
    if (!published_clock) {
      ROS_WARN("publish /clock");
      /// clock
      publish_clock = true;
      clock_pub = rosNode->advertise<rosgraph_msgs::Clock>("/clock", 5);
      /// trajectory
    }
    use_trajectory_controller = true;
    control_toggle_service = rosNode->advertiseService("toggle_controller",
                                                       &BodyNode::toggle_controller, this);

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
    //DeviceList<> devices = ioBody->devices();
    //cameras.assign(devices.extract<Camera>());
    for(size_t i=0; i < cameras.size(); ++i){
        auto camera = cameras[i];
        sensorConnections.add(
            camera->sigStateChanged().connect(
                [&, i](){ publishCameraImage(i); }));
    }
    //depth_cameras.assign(devices.extract<RangeCamera>());
    for(size_t i=0; i < depth_cameras.size(); ++i){
        auto camera = depth_cameras[i];
        int j = i + cameras.size();
        sensorConnections.add(
            camera->sigStateChanged().connect(
                [&, j](){ publishCameraImage(j); }));
    }
    //gyroSensors.assign(devices.extract<RateGyroSensor> ());
    for(size_t i=0; i < gyroSensors.size(); ++i){
      auto gyro = gyroSensors[i];
        sensorConnections.add(
            gyro->sigStateChanged().connect(
                [&, i](){ publishGyro(i); }));
    }
    //accelSensors.assign(devices.extract<AccelerationSensor> ());
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

    if (publish_clock && clock_counter % 5 == 0) {
      // clock pub
      rosgraph_msgs::Clock clock_msg;
      clock_msg.clock = now;
      clock_pub.publish(clock_msg);
    }
    //
    cnoid_hw_->read(now, period);
    // read q from choreonoid
    ros_cm_->update(now, period);

    if (use_trajectory_controller) {
      // write tau to choreonoid
      cnoid_hw_->write(now, period);
    } else {
      cnoid_hw_->dummy_write(now, period);
    }

    clock_counter++;
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
    if ( index >= cameras.size() ) {
      int idx = index - cameras.size();
      RangeCamera *camera = depth_cameras[idx];
      sensor_msgs::Image image;
      sensor_msgs::PointCloud2 points;
      createCameraImage(camera, image);
      createCameraImageDepth(camera, points);
      depth_cameraImagePublishers[idx].publish(image);
      points_cameraImagePublishers[idx].publish(points);
      return;
    }
    Camera *camera = cameras[index];
    sensor_msgs::Image image;
    createCameraImage(camera, image);
    cameraImagePublishers[index].publish(image);
}
void BodyNode::createCameraImageDepth(RangeCamera *camera, sensor_msgs::PointCloud2 &points)
{
  //sensor_msgs::PointCloud2 range;
  points.header.stamp.fromSec(time);
  points.header.frame_id = name + "/" + camera->name();
  points.width  = camera->resolutionX();
  points.height = camera->resolutionY();
  points.is_bigendian = false;
  points.is_dense     = true;
  if (camera->imageType() == cnoid::Camera::COLOR_IMAGE) {
    points.fields.resize(6);
    points.fields[3].name = "rgb";
    points.fields[3].offset = 12;
    points.fields[3].count = 1;
    points.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    points.point_step = 16;
  } else {
    points.fields.resize(3);
    points.point_step = 12;
  }
  points.row_step = points.point_step * points.width;
  points.fields[0].name = "x";
  points.fields[0].offset = 0;
  points.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  points.fields[0].count = 4;
  points.fields[1].name = "y";
  points.fields[1].offset = 4;
  points.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  points.fields[1].count = 4;
  points.fields[2].name = "z";
  points.fields[2].offset = 8;
  points.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  points.fields[2].count = 4;
  const std::vector<Vector3f>& pts = camera->constPoints();
  const unsigned char* pixels = camera->constImage().pixels();
  points.data.resize(pts.size() * points.point_step);
  unsigned char* dst = (unsigned char*)&(points.data[0]);
  for (size_t j = 0; j < pts.size(); ++j) {
    float x =   pts[j].x();
    float y = - pts[j].y();
    float z = - pts[j].z();
    std::memcpy(&dst[0], &x, 4);
    std::memcpy(&dst[4], &y, 4);
    std::memcpy(&dst[8], &z, 4);
    if (camera->imageType() == cnoid::Camera::COLOR_IMAGE) {
      dst[14] = *pixels++;
      dst[13] = *pixels++;
      dst[12] = *pixels++;
      dst[15] = 0;
    }
    dst += points.point_step;
  }
}
void BodyNode::createCameraImage(Camera *camera, sensor_msgs::Image &image)
{
    image.header.stamp.fromSec(time);
    image.header.frame_id = name + "/" + camera->name();
    image.height = camera->image().height();
    image.width  = camera->image().width();
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
}

void BodyNode::publishGyro(int index)
{
  auto gyro = gyroSensors[index];

  sensor_msgs::Imu msg;
  msg.header.stamp.fromSec(time);
  msg.header.frame_id = name + "/" + gyro->name();
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
  msg.header.frame_id = name + "/" + accel->name();
  msg.linear_acceleration.x = accel->dv()[0];
  msg.linear_acceleration.y = accel->dv()[1];
  msg.linear_acceleration.z = accel->dv()[2];

  accelPublishers[index].publish(msg);
}

bool BodyNode::toggle_controller(std_srvs::Empty::Request& req,
                                 std_srvs::Empty::Response& res)
{
  bool now = !use_trajectory_controller;
  use_trajectory_controller = now;
  if (use_trajectory_controller) {
    ROS_ERROR("start trajectory controller");
  } else {
    ROS_ERROR("stop trajectory controller");
  }

  return true;
}
