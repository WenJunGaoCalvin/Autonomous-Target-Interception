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
#include "GimbalSmall2dPluginRoll.hh"

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(GimbalSmall2dPluginRoll)

/// \brief Private data class
class gazebo::GimbalSmall2dPluginRollPrivate
{
  /// \brief Callback when a command string is received.
  /// \param[in] _msg Mesage containing the command string
  public: void OnStringMsg(ConstGzStringPtr &_msg);

  /// \brief A list of event connections
  public: std::vector<event::ConnectionPtr> connections;

  /// \brief Subscriber to the gimbal command topic
  public: transport::SubscriberPtr sub;

  /// \brief Publisher to the gimbal status topic
  public: transport::PublisherPtr pub;

  /// \brief Parent model of this plugin
  public: physics::ModelPtr model;

  /// \brief Joint for Rolling the gimbal
  public: physics::JointPtr rollJoint;

  /// \brief Command that updates the gimbal roll angle
  public: double roll_command = IGN_PI_2;

  /// \brief Pointer to the transport node
  public: transport::NodePtr node;

  /// \brief PID controller for the gimbal
  public: common::PID roll_pid;

  /// \brief Last update sim time
  public: common::Time lastUpdateTime;
};

/////////////////////////////////////////////////
GimbalSmall2dPluginRoll::GimbalSmall2dPluginRoll()
  : dataPtr(new GimbalSmall2dPluginRollPrivate)
{
  this->dataPtr->roll_pid.Init(1, 0, 0, 0, 0, 1.0, -1.0);
}

/////////////////////////////////////////////////
void GimbalSmall2dPluginRoll::Load(physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  this->dataPtr->model = _model;

  std::string jointName = "roll_joint";
  if (_sdf->HasElement("joint"))
  {
    jointName = _sdf->Get<std::string>("joint");
  }
  this->dataPtr->rollJoint = this->dataPtr->model->GetJoint(jointName);
  if (!this->dataPtr->rollJoint)
  {
    std::string scopedJointName = _model->GetScopedName() + "::" + jointName;
    gzwarn << "joint [" << jointName
           << "] not found, trying again with scoped joint name ["
           << scopedJointName << "]\n";
    this->dataPtr->rollJoint = this->dataPtr->model->GetJoint(scopedJointName);
  }
  if (!this->dataPtr->rollJoint)
  {
    gzerr << "GimbalSmall2dPluginRoll::Load ERROR! Can't get joint '"
          << jointName << "' " << endl;
  }
}

/////////////////////////////////////////////////
void GimbalSmall2dPluginRoll::Init()
{
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->dataPtr->model->GetWorld()->Name());

  this->dataPtr->lastUpdateTime =
    this->dataPtr->model->GetWorld()->SimTime();

  std::string topic = std::string("~/") +  this->dataPtr->model->GetName() +
    "/gimbal_roll_cmd";
  this->dataPtr->sub = this->dataPtr->node->Subscribe(topic,
      &GimbalSmall2dPluginRollPrivate::OnStringMsg, this->dataPtr.get());

  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&GimbalSmall2dPluginRoll::OnUpdate, this)));

  topic = std::string("~/") +
    this->dataPtr->model->GetName() + "/gimbal_roll_status";

  this->dataPtr->pub =
    this->dataPtr->node->Advertise<gazebo::msgs::GzString>(topic);
}

/////////////////////////////////////////////////
void GimbalSmall2dPluginRollPrivate::OnStringMsg(ConstGzStringPtr &_msg)
{
  this->roll_command = atof(_msg->data().c_str());
}

/////////////////////////////////////////////////
void GimbalSmall2dPluginRoll::OnUpdate()
{
  if (!this->dataPtr->rollJoint)
    return;

  IGN_PROFILE("GimbalSmall2dPluginRoll::OnUpdate");
  IGN_PROFILE_BEGIN("Update");

  double angle = this->dataPtr->rollJoint->Position(0);

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
    // double error = angle - this->dataPtr->roll_command;
    double error = angle - 0;
    double force = this->dataPtr->roll_pid.Update(error, dt);
    this->dataPtr->rollJoint->SetForce(0, force);
    this->dataPtr->lastUpdateTime = time;
  }

  static int i = 1000;
  if (++i > 100)
  {
    i = 0;
    std::stringstream ss;
    ss << angle;
    gazebo::msgs::GzString m;
    m.set_data(ss.str());
    this->dataPtr->pub->Publish(m);
  }
  IGN_PROFILE_END();
}
