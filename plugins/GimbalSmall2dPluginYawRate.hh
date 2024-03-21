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
#ifndef GAZEBO_PLUGINS_GIMBALSMALL2DPLUGINYAWRATE_HH_
#define GAZEBO_PLUGINS_GIMBALSMALL2DPLUGINYAWRATE_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo
{
  // Forward declare private data class
  class GimbalSmall2dPluginYawRatePrivate;

  /// \brief A plugin for controlling the angle of a gimbal joint
  class GZ_PLUGIN_VISIBLE GimbalSmall2dPluginYawRate : public ModelPlugin
  {
    /// \brief Constructor
    public: GimbalSmall2dPluginYawRate();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    /// \brief Callback on world update
    private: void OnUpdate();

    // public: void Activate_Callback(const std_msgs::Float64::ConstPtr& msg);

    /// \brief Private data pointer
    private: std::unique_ptr<GimbalSmall2dPluginYawRatePrivate> dataPtr;
   

  };
}
#endif
