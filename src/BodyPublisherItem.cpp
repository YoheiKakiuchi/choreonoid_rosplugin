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
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <memory>
#include "gettext.h"

//#include "cnoid_robot_hardware.h"
// clock
#include "rosgraph_msgs/Clock.h"
//#include "std_srvs/Empty.h"
#include <cmath>
#include <cstdlib>

#include <boost/tokenizer.hpp>

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

    //ros::Publisher posePublisher;

    DeviceList<Camera> cameras;
    //vector<image_transport::Publisher> cameraImagePublishers;
    vector<image_transport::CameraPublisher> cameraImagePublishers;
    DeviceList<RangeCamera> depth_cameras;
    vector<ros::Publisher> points_cameraImagePublishers;
    /// TODO: Range, FishEye

    BodyNode(BodyItem* bodyItem);

    void initialize(ControllerIO* io, std::vector<std::string> &option);
    void start(ControllerIO* io, double maxPublishRate);
    void input();
    void control();
    void output();
    void stop();

    void startToPublishKinematicStateChangeOnGUI();
    void stopToPublishKinematicStateChangeOnGUI();

    //void publishPose(Body* body, double time);
    void publishCameraImage(int index);
    void createCameraImage(Camera *camera, sensor_msgs::Image &image);
    void createCameraImageDepth(RangeCamera *camera, sensor_msgs::PointCloud2 &points);
    //DeviceList<RateGyroSensor> gyroSensors;
    //DeviceList<AccelerationSensor> accelSensors;

    //vector<ros::Publisher> gyroPublishers;
    //vector<ros::Publisher> accelPublishers;
    //void publishGyro(int index);
    //void publishAccel(int index);
    // TODO: force

    //
    bool publish_clock;
    unsigned long clock_counter;
    ros::Publisher clock_pub;
    // ros control
    //cnoid_robot_hardware::CnoidRobotHW *cnoid_hw_;
    //controller_manager::ControllerManager *ros_cm_;
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

    // Copy from ControllerIO
    typedef boost::escaped_list_separator<char> separator;
    separator sep('\\', ' ');

    std::string s = optionString();
    boost::tokenizer<separator> tok(s, sep);

    std::vector<std::string> options;
    for(boost::tokenizer<separator>::iterator p = tok.begin(); p != tok.end(); ++p){
        const string& token = *p;
        if(!token.empty()){
            options.push_back(token);
        }
    }
    //

    impl->bodyNode->initialize(io, options);

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
    ControllerItem::doPutProperties(putProperty);
    putProperty(_("Max publish rate"), impl->maxPublishRate, changeProperty(impl->maxPublishRate));
}


bool BodyPublisherItem::store(Archive& archive)
{
    ControllerItem::store(archive);
    archive.write("maxPublishRate", impl->maxPublishRate);
    return true;
}


bool BodyPublisherItem::restore(const Archive& archive)
{
    ControllerItem::restore(archive);
    archive.read("maxPublishRate", impl->maxPublishRate);
    return true;
}

} // cnoid

BodyNode::BodyNode(BodyItem* bodyItem)
    : bodyItem(bodyItem),
      timeBar(TimeBar::instance())
{
    name = bodyItem->name();
    std::replace(name.begin(), name.end(), '-', '_');

    rosNode.reset(new ros::NodeHandle(name));

    //posePublisher = rosNode->advertise<geometry_msgs::PoseStamped>("ground_truth_pose", 1000);
    startToPublishKinematicStateChangeOnGUI();

    auto body = bodyItem->body();
    DeviceList<> devices = body->devices();

    cameras.assign(devices.extract<Camera>());
    depth_cameras.assign(devices.extract<RangeCamera>());

    image_transport::ImageTransport it(*rosNode);
    cameraImagePublishers.resize(cameras.size());
    for(size_t i=0; i < cameras.size(); ++i) {
        Camera *camera = cameras[i];
        if (!!camera) {
          //cameraImagePublishers[i] = it.advertise(camera->name() + "/image", 1);
          cameraImagePublishers[i] = it.advertiseCamera(camera->name() + "/image", 1);
          RangeCamera* rcamera = dynamic_cast<RangeCamera*>(camera);
          if (!!rcamera) {
            depth_cameras.push_back(rcamera);
          }
        }
    }

    points_cameraImagePublishers.resize(depth_cameras.size());
    for(size_t i=0; i < depth_cameras.size(); ++i){
        auto camera = depth_cameras[i];
        points_cameraImagePublishers[i] = rosNode->advertise<sensor_msgs::PointCloud2>(camera->name() + "/points", 1);
    }

    // gyroSensors.assign(devices.extract<RateGyroSensor> ());
    // gyroPublishers.resize(gyroSensors.size());
    // for(size_t i = 0; i < gyroSensors.size(); ++i) {
    //     auto gyro = gyroSensors[i];
    //     std::string name = gyro->name();
    //     gyroPublishers[i] = rosNode->advertise<sensor_msgs::Imu>(name, 1);
    // }
    // accelSensors.assign(devices.extract<AccelerationSensor> ());
    // accelPublishers.resize(accelSensors.size());
    // for(size_t i = 0; i < accelSensors.size(); ++i) {
    //     auto accel = accelSensors[i];
    //     std::string name = accel->name();
    //     accelPublishers[i] = rosNode->advertise<sensor_msgs::Imu>(name, 1);
    // }

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
      /// clock
      publish_clock = true;
      clock_pub = rosNode->advertise<rosgraph_msgs::Clock>("/clock", 5);
      /// trajectory
    }

    //cnoid_hw_ = NULL;
}

void BodyNode::initialize(ControllerIO* io, std::vector<std::string> &opt)
{
#if 0
    if (cnoid_hw_ != NULL) return;

    cnoid_hw_ = new cnoid_robot_hardware::CnoidRobotHW();
    cnoid_hw_->cnoid_body = bodyItem->body();

    // option parse
    {
      int count = -1;
      //std::string option = io->optionString();
      for(auto& option : opt) {
        count++;
        std::size_t ppos, pos;
        pos = option.find(':');
        if (pos == std::string::npos) {
          //
          continue;
        }
        std::string nm (option, 0, pos);
        //std::cerr << "name: " << nm << std::endl;

        cnoid::Link* joint = cnoid_hw_->cnoid_body->link(nm);
        if (!joint || joint->isFixedJoint()) {
          //
          continue;
        }

        cnoid_hw_->use_joints.push_back(nm);
        cnoid_hw_->p_gain.push_back(0.0);
        cnoid_hw_->i_gain.push_back(0.0);
        cnoid_hw_->d_gain.push_back(0.0);

        ppos = pos;
        pos = option.find(':', ppos+1);
        if (pos == std::string::npos) {
          //
          continue;
        }
        std::string pgain (option, ppos+1, pos-ppos-1);
        if(pos-ppos-1 != 0) {
          //std::cerr << "pgain: " << std::atof(pgain.c_str()) << std::endl;
          cnoid_hw_->p_gain[count] = std::atof(pgain.c_str());
        }

        ppos = pos;
        pos = option.find(':', ppos+1);
        if (pos == std::string::npos) {
          //
          continue;
        }
        std::string igain (option, ppos+1, pos-ppos-1);
        if(pos-ppos-1 != 0) {
          //std::cerr << "igain: " << std::atof(igain.c_str()) << std::endl;
          cnoid_hw_->i_gain[count] = std::atof(igain.c_str());
        }

        if(pos+1 == option.size()) {
          //
          continue;
        }
        std::string dgain (option, pos+1);
        if(pos-ppos-1 != 0) {
          //std::cerr << "dgain: " << std::atof(dgain.c_str()) << std::endl;
          cnoid_hw_->d_gain[count] = std::atof(dgain.c_str());
        }
      }
    }
    //ros::NodeHandle nh;
    ros::NodeHandle robot_nh("~");
    if (!cnoid_hw_->init(*rosNode, robot_nh)) {
      ROS_ERROR("Faild to initialize hardware");
      //exit(1);
    }
    ros_cm_ = new controller_manager::ControllerManager (cnoid_hw_, *rosNode);
#endif

}

void BodyNode::start(ControllerIO* io, double maxPublishRate)
{
    //cnoid_hw_->cnoid_body = io->body();

    ioBody = io->body();
    time = 0.0;
    minPublishCycle = maxPublishRate > 0.0 ? (1.0 / maxPublishRate) : 0.0;
    timeToPublishNext = minPublishCycle;
    timeStep = io->timeStep();

    stopToPublishKinematicStateChangeOnGUI();

    sensorConnections.disconnect();
    DeviceList<> devices = ioBody->devices();
    cameras.assign(devices.extract<Camera>());
    depth_cameras.assign(devices.extract<RangeCamera>());
    for(size_t i=0; i < cameras.size(); ++i){
        auto camera = cameras[i];
        sensorConnections.add(
            camera->sigStateChanged().connect(
                [&, i](){ publishCameraImage(i); }));
        {
          Camera *cam = cameras[i];
          if (!!cam) {
            RangeCamera* rcamera = dynamic_cast<RangeCamera*>(cam);
            if (!!rcamera) {
              depth_cameras.push_back(rcamera);
            }
          }
        }
    }
    for(size_t i=0; i < depth_cameras.size(); ++i){
        auto camera = depth_cameras[i];
        int j = i + cameras.size();
        sensorConnections.add(
            camera->sigStateChanged().connect(
                [&, j](){ publishCameraImage(j); }));
    }
#if 0
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
#endif
}


void BodyNode::input()
{
    timeToPublishNext += timeStep;
    if(timeToPublishNext > minPublishCycle){
        //publishPose(ioBody, time);
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
    //cnoid_hw_->read(now, period);
    // read q from choreonoid
    //ros_cm_->update(now, period);

    // write tau to choreonoid
    //cnoid_hw_->write(now, period);

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
#if 0
    if(posePublisher){
        auto body = bodyItem->body();
        //publishPose(body, timeBar->time());
        connectionOfKinematicStateChange.reset(
            bodyItem->sigKinematicStateChanged().connect(
                [&](){ publishPose(bodyItem->body(), timeBar->time()); }));
    }
#endif
}


void BodyNode::stopToPublishKinematicStateChangeOnGUI()
{
    connectionOfKinematicStateChange.disconnect();
}
#if 0
void BodyNode::publishPose(Body* body, double time)
{
  geometry_msgs::PoseStamped pose;
  pose.header.stamp.fromSec(time);
  pose.header.frame_id = "choreonoid_origin";

  Vector3d trans = body->rootLink()->translation();
  Quaterniond quat(body->rootLink()->rotation());

  pose.pose.position.x = trans.x();
  pose.pose.position.y = trans.y();
  pose.pose.position.z = trans.z();
  pose.pose.orientation.x = quat.x();
  pose.pose.orientation.y = quat.y();
  pose.pose.orientation.z = quat.z();
  pose.pose.orientation.w = quat.w();

  posePublisher.publish(pose);
}
#endif
void BodyNode::publishCameraImage(int index)
{
    if ( index >= cameras.size() ) {
      int idx = index - cameras.size();
      RangeCamera *camera = depth_cameras[idx];
      sensor_msgs::PointCloud2 points;
      createCameraImageDepth(camera, points);
      points_cameraImagePublishers[idx].publish(points);
      return;
    }
    Camera *camera = cameras[index];
    sensor_msgs::Image image;
    createCameraImage(camera, image);
    sensor_msgs::CameraInfo info;
    info.width  = image.width;
    info.height = image.height;
    info.header.stamp    = image.header.stamp;
    info.header.frame_id = image.header.frame_id;
    info.distortion_model = "plumb_bob";
    info.D.resize(5);
    for (int i = 0; i < 5; i++ ) info.D[i] = 0;
    {
      double fovy2 = camera->fieldOfView() / 2.0;
      double width  = camera->resolutionX();
      double height = camera->resolutionY();
      double minlength = std::min(width, height);
      double fu, fv;
      fv = fu = minlength / tan(fovy2) / 2.0;
      double u0 = (width - 1)/2.0;
      double v0 = (height - 1)/2.0;

      info.K[0] = fu;
      info.K[1] = 0.0;
      info.K[2] = u0;
      info.K[3] = 0.0;
      info.K[4] = fv;
      info.K[5] = v0;
      info.K[6] = info.K[7] = 0.0; info.K[8] = 1.0;

      info.P[0] = fu;
      info.P[1] = 0.0;
      info.P[2] = u0;
      info.P[4] = 0.0;
      info.P[5] = fv;
      info.P[6] = v0;
      info.P[3] = info.P[7] = info.P[8] = info.P[9] = info.P[11] = 0.0;
      info.P[10] = 1.0;
    }
    for (int i = 0; i < 9; i++ ) info.R[i] = 0;
    info.R[0] = info.R[4] = info.R[8] = 1;

    cameraImagePublishers[index].publish(image, info);
}
void BodyNode::createCameraImageDepth(RangeCamera *camera, sensor_msgs::PointCloud2 &points)
{
  //sensor_msgs::PointCloud2 range;
  points.header.stamp.fromSec(time);
  //points.header.frame_id = name + "/" + camera->name();
  points.header.frame_id = camera->name(); // not using robot namespace
  points.width  = camera->resolutionX();
  points.height = camera->resolutionY();
  points.is_bigendian = false;
  points.is_dense     = true;
  if (camera->imageType() == cnoid::Camera::COLOR_IMAGE) {
    points.fields.resize(4);
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
  points.fields[0].count = 1;
  points.fields[1].name = "y";
  points.fields[1].offset = 4;
  points.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  points.fields[1].count = 1;
  points.fields[2].name = "z";
  points.fields[2].offset = 8;
  points.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  points.fields[2].count = 1;
  const std::vector<Vector3f>& pts = camera->constPoints();
  const unsigned char* pixels = camera->constImage().pixels();
  points.data.resize(pts.size() * points.point_step);
  unsigned char* dst = (unsigned char*)&(points.data[0]);
  for (size_t j = 0; j < pts.size(); ++j) {
    float x =   pts[j].x();
    float y = - pts[j].y();
    float z = - pts[j].z();
    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
      x = y = z = 0;
    }
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
    //image.header.frame_id = name + "/" + camera->name();
    image.header.frame_id = camera->name(); // not using robot namespace
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
#if 0
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
#endif
