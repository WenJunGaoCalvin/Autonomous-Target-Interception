/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <string>
#include <vector>

#include "ignition/common/Profiler.hh"
#include "gazebo/common/PID.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "GimbalSmall2dPluginYawRate.hh"

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(GimbalSmall2dPluginYawRate)

/// \brief Private data class
class gazebo::GimbalSmall2dPluginYawRatePrivate
{
  /// \brief Callback when a command string is received.
  /// \param[in] _msg Mesage containing the command string
  public: void OnStringMsg(ConstGzStringPtr &_msg);

  /// \brief A list of event connections
  public: std::vector<event::ConnectionPtr> connections;

  /// \brief Subscriber to the gimbal command topic
  public: transport::SubscriberPtr sub;
  public: transport::SubscriberPtr yaw_sub;

  /// \brief Publisher to the gimbal status topic
  public: transport::PublisherPtr pub;

  /// \brief Parent model of this plugin
  public: physics::ModelPtr model;

  /// \brief Joint for yawing the gimbal
  public: physics::JointPtr yawRateJoint;

  /// \brief Command that updates the gimbal yaw angle
  public: double yawRate_command = IGN_PI_2;

  /// \brief Pointer to the transport node
  public: transport::NodePtr node;

  /// \brief PID controller for the gimbal
  public: common::PID pid;

  /// \brief Last update sim time
  public: common::Time lastUpdateTime;

  public: ros::NodeHandle nh;           // ros node handler
  public: ros::Subscriber ros_sub;         // ros subscriber
  public: ros::Publisher ros_pub; // ros publisher
  public: float yaw_rate_ros;    // to store boolen ros callback data
  public: std_msgs::String gimbal_yaw_gazebo;

  public: void Activate_Callback(const std_msgs::Float64::ConstPtr& msg);

  public: void UpdateYaw(ConstGzStringPtr &_msg);

};

/////////////////////////////////////////////////
GimbalSmall2dPluginYawRate::GimbalSmall2dPluginYawRate()
  : dataPtr(new GimbalSmall2dPluginYawRatePrivate)
{
  // this->dataPtr->pid.Init(0.0001, 0, 0.0001, 0, 0, 1.0, -1.0);
  // this PID shall take vel values between -180 and 180 rad per sec
  this->dataPtr->pid.Init(0.5, 1000, 0.000025, 0, 0, 1.56995, -1.56995); //KMAX = 0.57, f0 = 500
}

/////////////////////////////////////////////////
void GimbalSmall2dPluginYawRate::Load(physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
      ROS_FATAL_STREAM_NAMED("Gimbal","A ROS node for Gazebo has not been initialized");
      return;
  }
  ROS_INFO("ROS Model Plugin Loaded!");

  this->dataPtr->model = _model;



  std::string jointName = "yaw_joint";
  if (_sdf->HasElement("joint"))
  {
    jointName = _sdf->Get<std::string>("joint");
  }
  this->dataPtr->yawRateJoint = this->dataPtr->model->GetJoint(jointName);
  if (!this->dataPtr->yawRateJoint)
  {
    std::string scopedJointName = _model->GetScopedName() + "::" + jointName;
    gzwarn << "joint [" << jointName
           << "] not found, trying again with scoped joint name ["
           << scopedJointName << "]\n";
    this->dataPtr->yawRateJoint = this->dataPtr->model->GetJoint(scopedJointName);
  }
  if (!this->dataPtr->yawRateJoint)
  {
    gzerr << "GimbalSmall2dPluginYawRate::Load ERROR! Can't get joint '"
          << jointName << "' " << endl;
  }
}

/////////////////////////////////////////////////
void GimbalSmall2dPluginYawRate::Init()
{
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->dataPtr->model->GetWorld()->Name());

  this->dataPtr->lastUpdateTime =
    this->dataPtr->model->GetWorld()->SimTime();

  std::string topic = std::string("~/") +  this->dataPtr->model->GetName() +
    "/gimbal_yaw_rate_cmd";

  this->dataPtr->sub = this->dataPtr->node->Subscribe(topic,
      &GimbalSmall2dPluginYawRatePrivate::OnStringMsg, this->dataPtr.get());

  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&GimbalSmall2dPluginYawRate::OnUpdate, this)));

  topic = std::string("~/") +
    this->dataPtr->model->GetName() + "/gimbal_yaw_rate_status";

  this->dataPtr->pub =
    this->dataPtr->node->Advertise<gazebo::msgs::GzString>(topic);

  this->dataPtr->ros_sub = this->dataPtr->nh.subscribe("/ros_yaw_rate", 1, 
    &GimbalSmall2dPluginYawRatePrivate::Activate_Callback, this->dataPtr.get());

  this->dataPtr->ros_pub =
    this->dataPtr->nh.advertise<std_msgs::String>("/ros_gimbal_yaw_status", 1);

  topic = std::string("~/") +
    this->dataPtr->model->GetName() + "/gimbal_yaw_status";

  this->dataPtr->yaw_sub = this->dataPtr->node->Subscribe(topic,
    &GimbalSmall2dPluginYawRatePrivate::UpdateYaw, this->dataPtr.get());

}

/////////////////////////////////////////////////
void GimbalSmall2dPluginYawRatePrivate::OnStringMsg(ConstGzStringPtr &_msg)
{
  this->yawRate_command = atof(_msg->data().c_str());
}

/////////////////////////////////////////////////
void GimbalSmall2dPluginYawRatePrivate::Activate_Callback(const std_msgs::Float64::ConstPtr& msg)
{
  // ROS_INFO("Received Message");
  // ROS_INFO("Received Message = %g", msg->data);
  this->yaw_rate_ros = msg->data;
}

/////////////////////////////////////////////////
void GimbalSmall2dPluginYawRatePrivate::UpdateYaw(ConstGzStringPtr &_msg)
{
  this->gimbal_yaw_gazebo.data = _msg->data(); //String
}

/////////////////////////////////////////////////
void GimbalSmall2dPluginYawRate::OnUpdate()
{
  if (!this->dataPtr->yawRateJoint)
    return;

  IGN_PROFILE("GimbalSmall2dPluginYawRate::OnUpdate");
  IGN_PROFILE_BEGIN("Update");

  double yaw_rate = this->dataPtr->yawRateJoint->GetVelocity(0);
  double yaw_rate_setpt = this->dataPtr->yaw_rate_ros;
  // ROS_INFO("yaw_rate = %g", yaw_rate);
  // ROS_INFO("yaw_rate_setpt = %g", yaw_rate_setpt);
  common::Time time = this->dataPtr->model->GetWorld()->SimTime();
  if (time < this->dataPtr->lastUpdateTime)
  {
    this->dataPtr->lastUpdateTime = time;
    IGN_PROFILE_END();
    return;
  }
  else if (time > this->dataPtr->lastUpdateTime)
  {
    double dt = (this->dataPtr->lastUpdateTime - time).Double();
    double error = yaw_rate - 2*yaw_rate_setpt;
    double force = this->dataPtr->pid.Update(error, dt);
    // ROS_INFO("force = %g", force);
    this->dataPtr->yawRateJoint->SetForce(0, force);
    this->dataPtr->lastUpdateTime = time;
  }

  static int i = 1000;
  if (++i > 100)
  {
    i = 0;
    std::stringstream ss;
    ss << yaw_rate;
    gazebo::msgs::GzString m;
    m.set_data(ss.str());
    this->dataPtr->pub->Publish(m);
    this->dataPtr->ros_pub.publish(this->dataPtr->gimbal_yaw_gazebo);
  }
  IGN_PROFILE_END();
}


