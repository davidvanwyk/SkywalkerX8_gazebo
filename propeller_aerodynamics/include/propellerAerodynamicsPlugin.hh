/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef GAZEBO_PLUGINS_PROPELLERAERODYNAMICSPLUGIN_HH_
#define GAZEBO_PLUGINS_PROPELLERAERODYNAMICSPLUGIN_HH_

#include <string>
#include <vector>

#include <ignition/math/Vector3.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"


namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
}

namespace gazebo
{

class GAZEBO_VISIBLE propellerAerodynamicsPlugin : public ModelPlugin
{

  /// \brief Constructor.
  public: propellerAerodynamicsPlugin();

  /// \brief Destructor.
  public: ~propellerAerodynamicsPlugin();

  // Documentation Inherited.
  public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Callback for World Update events.
  public: virtual void OnUpdate(const common::UpdateInfo& _info);

  /// \brief Connection to World Update events.
  protected: event::ConnectionPtr updateConnection;

  /// \brief Pointer to world.
  protected: physics::WorldPtr world;

  /// \brief Pointer to physics engine.
  protected: physics::PhysicsEnginePtr physics;

  /// \brief Pointer to model containing plugin.
  protected: physics::ModelPtr model;

  /// \brief Aliasing constants
  protected: double sampling_time, prev_sim_time, rotor_velocity_slowdown_sim;

  /// \brief Propeller aerodynamic coefficients
  /// Propeller Force = 0.5 * rho * Sprop * Cprop * ( (kmotor*dt)^2 - Va^2 )
  /// Propeller Moment = -rotation_direction*kT_p*(komega*dt)^2
  /// dt = motor_rot_vel/max_motor_speed;
  protected: double Sprop, Cprop, kmotor;
  protected: double rotation_direction, kT_p, komega;
  protected: double max_motor_speed;

  /// \brief air density
  /// To be pulled from the atmospheric model, dependent on system altitude
  protected: double rho;

  /// \brief Pointer to link describing vehicle body.
  protected: physics::LinkPtr link;

  /// \brief Pointers to the actuator joints for a standard config aircraft (default config)
  protected: physics::JointPtr propeller_joint;

  /// \brief SDF for this plugin;
  protected: sdf::ElementPtr sdf;

};
}

#endif /* GAZEBO_PLUGINS_INCLUDE_GAZEBO_PLUGINS_PROPELLERPLUGIN_H_ */
