/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include "vehicleAerodynamicsPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(vehicleAerodynamicsPlugin);

/////////////////////////////////////////////////
vehicleAerodynamicsPlugin::vehicleAerodynamicsPlugin()
: CL0(0.0), CLalpha(0.0), CLq(0.0), CLde(0.0),
  CD0(0.0), CDalpha(0.0), CDalpha2(0.0), CDq(0.0), CDbeta2(0.0), CDbeta(0.0), CDde(0.0),
  CY0(0.0), CYbeta(0.0), CYp(0.0), CYr(0.0), CYda(0.0), CYdr(0.0),
  Cl0(0.0), Clbeta(0.0), Clp(0.0), Clr(0.0), Clda(0.0), Cldr(0.0),
  Cm0(0.0), Cmalpha(0.0), Cmq(0.0), Cmde(0.0),
  Cn0(0.0), Cnbeta(0.0), Cnp(0.0), Cnr(0.0), Cnda(0.0), Cndr(0.0),
  b(0.0), c(0.0), S(0.0),
  alpha_0(0.0), M_trans(0.0), C_mfp(0.0), rho(1.2)
{

  this->vehicleConfig = VC_error;

}

/////////////////////////////////////////////////
vehicleAerodynamicsPlugin::~vehicleAerodynamicsPlugin()
{

	updateConnection->~Connection();

}

/////////////////////////////////////////////////
void vehicleAerodynamicsPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "vehicleAerodynamicsPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "vehicleAerodynamicsPlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "vehicleAerodynamicsPlugin world pointer is NULL");

  this->physics = this->world->Physics();
  GZ_ASSERT(this->physics, "vehicleAerodynamicsPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "vehicleAerodynamicsPlugin _sdf pointer is NULL");

  // Getting aerodynamic coeffecients from model .sdf

	if (_sdf->HasElement("CL0"))
		this->CL0 = _sdf->Get<double>("CL0");

	if (_sdf->HasElement("CLalpha"))
		this->CLalpha = _sdf->Get<double>("CLalpha");

	if (_sdf->HasElement("CLq"))
		this->CLq = _sdf->Get<double>("CLq");

	if (_sdf->HasElement("CLde"))
		this->CLde = _sdf->Get<double>("CLde");

	if (_sdf->HasElement("CD0"))
		this->CD0 = _sdf->Get<double>("CD0");

	if (_sdf->HasElement("CDalpha"))
		this->CDalpha = _sdf->Get<double>("CDalpha");

	if (_sdf->HasElement("CDalpha2"))
		this->CDalpha2 = _sdf->Get<double>("CDalpha2");

	if (_sdf->HasElement("CDq"))
		this->CDq = _sdf->Get<double>("CDq");

	if (_sdf->HasElement("CDbeta2"))
		this->CDbeta2 = _sdf->Get<double>("CDbeta2");

	if (_sdf->HasElement("CDbeta"))
		this->CDbeta = _sdf->Get<double>("CDbeta");

	if (_sdf->HasElement("CDde"))
		this->CDde = _sdf->Get<double>("CDde");

	if (_sdf->HasElement("CY0"))
		this->CY0 = _sdf->Get<double>("CY0");

	if (_sdf->HasElement("CYbeta"))
		this->CYbeta = _sdf->Get<double>("CYbeta");

	if (_sdf->HasElement("CYp"))
		this->CYp = _sdf->Get<double>("CYp");

	if (_sdf->HasElement("CYr"))
		this->CYr = _sdf->Get<double>("CYr");

	if (_sdf->HasElement("CYda"))
		this->CYda = _sdf->Get<double>("CYda");

	if (_sdf->HasElement("CYdr"))
		this->CYdr = _sdf->Get<double>("CYdr");

	if (_sdf->HasElement("Cl0"))
		this->Cl0 = _sdf->Get<double>("Cl0");

	if (_sdf->HasElement("Clbeta"))
		this->Clbeta = _sdf->Get<double>("Clbeta");

	if (_sdf->HasElement("Clp"))
		this->Clp = _sdf->Get<double>("Clp");

	if (_sdf->HasElement("Clr"))
		this->Clr = _sdf->Get<double>("Clr");

	if (_sdf->HasElement("Clda"))
		this->Clda = _sdf->Get<double>("Clda");

	if (_sdf->HasElement("Cldr"))
		this->Cldr = _sdf->Get<double>("Cldr");

	if (_sdf->HasElement("Cm0"))
		this->Cm0 = _sdf->Get<double>("Cm0");

	if (_sdf->HasElement("Cmalpha"))
		this->Cmalpha = _sdf->Get<double>("Cmalpha");

	if (_sdf->HasElement("Cmq"))
		this->Cmq = _sdf->Get<double>("Cmq");

	if (_sdf->HasElement("Cmde"))
		this->Cmde = _sdf->Get<double>("Cmde");

	if (_sdf->HasElement("Cn0"))
		this->Cn0 = _sdf->Get<double>("Cn0");

	if (_sdf->HasElement("Cnbeta"))
		this->Cnbeta = _sdf->Get<double>("Cnbeta");

	if (_sdf->HasElement("Cnp"))
		this->Cnp = _sdf->Get<double>("Cnp");

	if (_sdf->HasElement("Cnr"))
		this->Cnr = _sdf->Get<double>("Cnr");

	if (_sdf->HasElement("Cnda"))
		this->Cnda = _sdf->Get<double>("Cnda");

	if (_sdf->HasElement("Cndr"))
		this->Cndr = _sdf->Get<double>("Cndr");

	// Getting geometric parameters from model .sdf

	if (_sdf->HasElement("b"))
		this->b = _sdf->Get<double>("b");

	if (_sdf->HasElement("c"))
		this->c = _sdf->Get<double>("c");

	if (_sdf->HasElement("S"))
		this->S = _sdf->Get<double>("S");

	// Getting stall parameters from model .sdf

	if (_sdf->HasElement("alpha_0"))
		this->alpha_0 = _sdf->Get<double>("alpha_0");

	if (_sdf->HasElement("M_trans"))
		this->M_trans = _sdf->Get<double>("M_trans");

	if (_sdf->HasElement("C_mfp"))
		this->C_mfp = _sdf->Get<double>("C_mfp");

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
        << "The vehicleAerodynamicsPlugin will not generate forces\n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&vehicleAerodynamicsPlugin::OnUpdate, this));
    }
  }

  if (_sdf->HasElement("vehicle_config"))
  {
	sdf::ElementPtr elem = _sdf->GetElement("vehicle_config");
	this->vehicleConfig = this->stringHash(elem->Get<std::string>());

	switch(this->vehicleConfig)
	{

	case VC_standard:

		if (_sdf->HasElement("controlJoint_Elevator"))
		{
			std::string controlJoint_Elevator = _sdf->Get<std::string>("controlJoint_Elevator");
			this->controlJoint_Elevator = this->model->GetJoint(controlJoint_Elevator);
			if (!this->controlJoint_Elevator)
			{
			  gzerr << "Joint with name[" << controlJoint_Elevator << "] does not exist.\n";
			}
		}

		if (_sdf->HasElement("controlJoint_portAileron"))
		{
			std::string controlJoint_portAileron = _sdf->Get<std::string>("controlJoint_portAileron");
			this->controlJoint_portAileron = this->model->GetJoint(controlJoint_portAileron);
			if (!this->controlJoint_portAileron)
			{
			  gzerr << "Joint with name[" << controlJoint_portAileron << "] does not exist.\n";
			}
		}

		if (_sdf->HasElement("controlJoint_starboardAileron"))
		{
			std::string controlJoint_starboardAileron = _sdf->Get<std::string>("controlJoint_starboardAileron");
			this->controlJoint_starboardAileron = this->model->GetJoint(controlJoint_starboardAileron);
			if (!this->controlJoint_starboardAileron)
			{
			  gzerr << "Joint with name[" << controlJoint_starboardAileron << "] does not exist.\n";
			}
		}

		if (_sdf->HasElement("controlJoint_Rudder"))
		{
			std::string controlJoint_Rudder = _sdf->Get<std::string>("controlJoint_Rudder");
			this->controlJoint_Rudder = this->model->GetJoint(controlJoint_Rudder);
			if (!this->controlJoint_Rudder)
			{
			  gzerr << "Joint with name[" << controlJoint_Rudder << "] does not exist.\n";
			}
		}

	break;

	case VC_VTail:

		if (_sdf->HasElement("controlJoint_portRuddervator"))
		{
			std::string controlJoint_portRuddervator = _sdf->Get<std::string>("controlJoint_portRuddervator");
			this->controlJoint_portRuddervator = this->model->GetJoint(controlJoint_portRuddervator);
			if (!this->controlJoint_portRuddervator)
			{
			  gzerr << "Joint with name[" << controlJoint_portRuddervator << "] does not exist.\n";
			}
		}

		if (_sdf->HasElement("controlJoint_starboardRuddervator"))
		{
			std::string controlJoint_starboardRuddervator = _sdf->Get<std::string>("controlJoint_starboardRuddervator");
			this->controlJoint_starboardRuddervator = this->model->GetJoint(controlJoint_starboardRuddervator);
			if (!this->controlJoint_starboardRuddervator)
			{
			  gzerr << "Joint with name[" << controlJoint_starboardRuddervator << "] does not exist.\n";
			}
		}

	break;

	case VC_flyingWing:

		if (_sdf->HasElement("controlJoint_portElevon"))
		{
			std::string controlJoint_portElevon = _sdf->Get<std::string>("controlJoint_portElevon");
			this->controlJoint_portElevon = this->model->GetJoint(controlJoint_portElevon);
			if (!this->controlJoint_portElevon)
			{
			  gzerr << "Joint with name[" << controlJoint_portElevon << "] does not exist.\n";
			}
		}

		if (_sdf->HasElement("controlJoint_starboardElevon"))
		{
			std::string controlJoint_starboardElevon = _sdf->Get<std::string>("controlJoint_starboardElevon");
			this->controlJoint_starboardElevon = this->model->GetJoint(controlJoint_starboardElevon);
			if (!this->controlJoint_starboardElevon)
			{
			  gzerr << "Joint with name[" << controlJoint_starboardElevon << "] does not exist.\n";
			}
		}

	break;

	default: break;

	}

  }

}

/////////////////////////////////////////////////
void vehicleAerodynamicsPlugin::OnUpdate()
{
  GZ_ASSERT(this->link, "Link was NULL");

  // ------------------------------ Body Frame Airspeed Calculation ----------------------------- //

  // Get body linear velocity in the ROS standard ENU coordinate system
  ignition::math::Vector3d bodyLinearVelENU = this->link->RelativeLinearVel();

  // Convert this to the standard NED coordinate system and assign to standard variables (u, v, w)
  double v = bodyLinearVelENU.X();
  double u = bodyLinearVelENU.Y();
  double w = -bodyLinearVelENU.Z();

  // Get body angular velocity in the ROS standard ENU coordinate system
  ignition::math::Vector3d bodyAngularVelENU = this->link->RelativeAngularVel();

  // Convert this to the standard NED coordinate system and assign to standard variables (p, q, r)
  double q = bodyAngularVelENU.X();
  double p = bodyAngularVelENU.Y();
  double r = -bodyAngularVelENU.Z();

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

  // Calculate aircraft airspeed and associated angles (angle of attack and sideslip)

  ignition::math::Vector3d bodyAirspeedNED;
  bodyAirspeedNED.X(u - uw);
  bodyAirspeedNED.Y(v - vw);
  bodyAirspeedNED.Z(w - ww);

  double Va = bodyAirspeedNED.Length();
  double alpha = atan2(bodyAirspeedNED.Z(), bodyAirspeedNED.X());
  double beta = 0;

  if (Va > 0.01)
  {
	  beta = asin(bodyAirspeedNED[1]/Va);
  }

  // -------------------------------------------------------------------------------------------- //

  // ----------------------------- Aerodynamic Forces and Moments ------------------------------- //

  // Note that this doesn't consider propulsion forces (handled in a seperate plugin), nor does
  // it consider effects such as gravity (as this is handled in Gazebo already)

  // Control Inputs

  double de, da, dr = 0;

  switch(this->vehicleConfig)
  {

  case VC_standard:

	  de = this->controlJoint_Elevator->Position(0);
	  da = (this->controlJoint_portAileron->Position(0) - this->controlJoint_starboardAileron->Position(0))/2.0;
	  dr = this->controlJoint_Rudder->Position(0);

  break;

  case VC_VTail:

	  de = this->controlJoint_portRuddervator->Position(0) + this->controlJoint_starboardRuddervator->Position(0);
	  dr = this->controlJoint_portRuddervator->Position(0) - this->controlJoint_starboardRuddervator->Position(0);
	  da = (this->controlJoint_portAileron->Position(0) - this->controlJoint_starboardAileron->Position(0))/2.0;

  break;

  case VC_flyingWing:

	  de = this->controlJoint_portElevon->Position(0) + this->controlJoint_starboardElevon->Position(0);
	  da = this->controlJoint_portElevon->Position(0) - this->controlJoint_starboardElevon->Position(0);

  break;

  default: break;

  }

  // Aerodynamic Setup

  double aerodynamicMultiple = 0.5*rho*pow(Va, 2)*this->S; //Dynamic viscosity multipled by planform area

  double bOver2Va = 0;
  double cOver2Va = 0;

  if (Va > 0.01)
  {
	  bOver2Va = this->b/(2*Va);
	  cOver2Va = this->c/(2*Va);
  }

  // Calculating overall drag coefficient

  double CDCalc =  this->CD0 + this->CDalpha*alpha + this->CDalpha2*pow(alpha, 2) + this->CDde*pow(de, 2) + this->CDq*cOver2Va*q + this->CDbeta2*pow(beta, 2) + this->CDbeta*beta;

  // Calculating CLalpha

  double signAlpha = (alpha >= 0) - (alpha < 0);
  double CLAlpha = this->CL0 + this->CLalpha*alpha;

  // Applying flat plate stall model

  double sigma_alpha = (1 + exp(-this->M_trans*(alpha - this->alpha_0)) + exp(this->M_trans*(alpha + this->alpha_0)))/((1 + exp(-this->M_trans*(alpha - this->alpha_0)))*(1 + exp(this->M_trans*(alpha + this->alpha_0))));
  CLAlpha = (1 - sigma_alpha)*CLAlpha + sigma_alpha*(2*signAlpha*pow(sin(alpha), 2)*cos(alpha));

  // Calculating overall lift coefficient

  double CLCalc = CLAlpha + this->CLq*cOver2Va*q + this->CLde*de;

  // Rotating lift and drag into body frame

  double CX = -(cos(alpha)*CDCalc - sin(alpha)*CLCalc);
  double CZ = -(sin(alpha)*CDCalc + cos(alpha)*CLCalc);

  // Calculating overall sideforce coefficient

  double CY = this->CY0 + this->CYbeta*beta + this->CYp*bOver2Va*p + this->CYr*bOver2Va*r + this->CYda*da;

  // Calculating moment coefficients

  // Roll Coefficient

  double Cl = this->Cl0 + this->Clbeta*beta + this->Clp*bOver2Va*p + this->Clr*bOver2Va*r + this->Clda*da;

  // Yaw coefficient

  double Cn = this->Cn0 + this->Cnbeta*beta + this->Cnp*bOver2Va*p + this->Cnr*bOver2Va*r + this->Cnda*da;

  // Calculating pitch coefficient in alpha

  double CmAlpha = this->Cm0 + this->Cmalpha*alpha;

  // Applying flat plate stall model

  CmAlpha = (1 - sigma_alpha)*CmAlpha + sigma_alpha*(this->C_mfp*signAlpha*pow(sin(alpha), 2));

  // Calculating overall pitch coefficent

  double Cm = CmAlpha + this->Cmq*cOver2Va*q + this->Cmde*de;

  // Calculating aerodynamic forces and moments in body frame

  double Fx = aerodynamicMultiple*CX;
  double Fy = aerodynamicMultiple*CY;
  double Fz = aerodynamicMultiple*CZ;

  double l = aerodynamicMultiple*this->b*Cl;
  double m = aerodynamicMultiple*this->c*Cm;
  double n = aerodynamicMultiple*this->b*Cn;

  // Transitioning forces from NED to ENU as per ROS standard

  ignition::math::Vector3d aerodynamicForces(Fy, Fx, -Fz);
  ignition::math::Vector3d aerodynamicMoments(m, l, -n);

  // Correcting for NaN or Inf
  aerodynamicForces.Correct();
  aerodynamicMoments.Correct();

  this->link->AddRelativeForce(aerodynamicForces);
  this->link->AddRelativeTorque(aerodynamicMoments);

}


vehicle_config vehicleAerodynamicsPlugin::stringHash(std::string const& inputString)
{

	if (inputString == "standard") return VC_standard;
	if (inputString == "VTail") return VC_VTail;
	if (inputString == "FlyingWing") return VC_flyingWing;
	else return VC_error;

}
