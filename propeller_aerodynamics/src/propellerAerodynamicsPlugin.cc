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

#include <algorithm>
#include <functional>
#include <string>
#include <math.h>

#include <ignition/math/Pose3.hh>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/gazebo.hh"
#include "propellerAerodynamicsPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(propellerAerodynamicsPlugin);

propellerAerodynamicsPlugin::propellerAerodynamicsPlugin()
: sampling_time(0.001), prev_sim_time(0.0), rotor_velocity_slowdown_sim(1.0),
  Sprop(0.0), Cprop(0.0), kmotor(0.0), rotation_direction(0.0), kT_p(0.0),
  komega(0.0), max_motor_speed(0.0), rho(0.0)
{

}

propellerAerodynamicsPlugin::~propellerAerodynamicsPlugin()
{
  updateConnection->~Connection();
}

void propellerAerodynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

  GZ_ASSERT(_model, "propellerAerodynamicsPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "propellerAerodynamicsPlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "propellerAerodynamicsPlugin world pointer is NULL");

  this->physics = this->world->Physics();
  GZ_ASSERT(this->physics, "propellerAerodynamicsPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "propellerAerodynamicsPlugin _sdf pointer is NULL");

  // Getting propeller coeffecients from model .sdf

	if (_sdf->HasElement("Sprop"))
		this->Sprop = _sdf->Get<double>("Sprop");

	if (_sdf->HasElement("Cprop"))
		this->Cprop = _sdf->Get<double>("Cprop");

	if (_sdf->HasElement("kmotor"))
		this->kmotor = _sdf->Get<double>("kmotor");

	if (_sdf->HasElement("rotation_direction"))
		this->rotation_direction = _sdf->Get<double>("rotation_direction");

	if (_sdf->HasElement("kT_p"))
		this->kT_p = _sdf->Get<double>("kT_p");

	if (_sdf->HasElement("komega"))
		this->komega = _sdf->Get<double>("komega");

	if (_sdf->HasElement("rotor_velocity_slowdown_sim"))
		this->rotor_velocity_slowdown_sim = _sdf->Get<double>("rotor_velocity_slowdown_sim");

  // getting vehicle link
  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
    GZ_ASSERT(this->link, "Link was NULL");

    if (!this->link)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The propellerAerodynamicsPlugin will not generate forces\n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&propellerAerodynamicsPlugin::OnUpdate, this, _1));
    }
  }

  if (_sdf->HasElement("propeller_joint"))
  {
	sdf::ElementPtr elem = _sdf->GetElement("propeller_joint");
	GZ_ASSERT(elem, "Element propeller_joint doesn't exist!");
	std::string jointName = elem->Get<std::string>();
	this->propeller_joint = this->model->GetJoint(jointName);
	GZ_ASSERT(this->propeller_joint, "Joint was NULL");

	if (!this->propeller_joint)
	{
	  gzerr << "Joint with name[" << jointName << "] not found. ";
	}
  }

}

// This gets called by the world update start event.
void propellerAerodynamicsPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  sampling_time = _info.simTime.Double() - prev_sim_time;
  prev_sim_time = _info.simTime.Double();

  // Motor rotor velocity is the velocity around the y axis of the joint
  double motor_rot_vel = propeller_joint->GetVelocity(1);

	if (motor_rot_vel / (2 * M_PI) > 1 / (2 * sampling_time))
	{
	  gzerr << "Aliasing on motor might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
	}

  double real_motor_velocity = motor_rot_vel * rotor_velocity_slowdown_sim;
  double dt = ignition::math::clamp(real_motor_velocity/kmotor, -1.0, 1.0);

  // ------------------------------ Body Frame Airspeed Calculation ----------------------------- //

  // Get body linear velocity in the ROS standard ENU coordinate system
  ignition::math::Vector3d bodyLinearVelENU = this->link->RelativeLinearVel();

  // Convert this to the standard NED coordinate system and assign to standard variables (u, v, w)
  double v = bodyLinearVelENU.X();
  double u = bodyLinearVelENU.Y();
  double w = -bodyLinearVelENU.Z();

  // Get atmospheric conditions (in inertial frame)

  ignition::math::Pose3d inertialPose = this->link->WorldPose();
  double alt = inertialPose.Pos()[2];

  double rho = this->world->Atmosphere().MassDensity(alt);

  double uw, vw, ww = 0;

  if (this->world->WindEnabled())
  {

	  if (this->link->WindMode())
	  {

		  ignition::math::Vector3d bodyWindENU = this->link->RelativeWindLinearVel();

		  vw = bodyWindENU[0];
		  uw = bodyWindENU[1];
		  ww = -bodyWindENU[2];

	  }

  }

  // Calculate aircraft airspeed

  ignition::math::Vector3d bodyAirspeedNED;
  bodyAirspeedNED.X(u - uw);
  bodyAirspeedNED.Y(v - vw);
  bodyAirspeedNED.Z(w - ww);

  double Va = bodyAirspeedNED.Length();

  double Fx = 0.5 * rho * Sprop * Cprop *( pow(kmotor*dt, 2) - pow(Va, 2) );

  if (Fx <= 0)
  {
	  Fx = 0;
  }

  double l = -ignition::math::signum(dt)*kT_p*pow(komega*dt, 2);

  ignition::math::Vector3d aerodynamicForces(0, Fx, 0);
  ignition::math::Vector3d aerodynamicMoments(0, l, 0);

  // Correcting for NaN or Inf
  aerodynamicForces.Correct();
  aerodynamicMoments.Correct();

  this->link->AddRelativeForce(aerodynamicForces);
  this->link->AddRelativeTorque(aerodynamicMoments);

}


