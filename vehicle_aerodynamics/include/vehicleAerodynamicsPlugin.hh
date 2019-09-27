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
#ifndef GAZEBO_PLUGINS_VEHICLEAERODYNAMICSPLUGIN_HH_
#define GAZEBO_PLUGINS_VEHICLEAERODYNAMICSPLUGIN_HH_

#include <string>
#include <vector>

#include <ignition/math/Vector3.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"

enum vehicle_config {
	VC_standard,
	VC_VTail,
	VC_flyingWing,
	VC_error
};

namespace gazebo
{
  /// \brief A plugin that simulates vehicle aerodynamics
  class GAZEBO_VISIBLE vehicleAerodynamicsPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: vehicleAerodynamicsPlugin();

    /// \brief Destructor.
    public: ~vehicleAerodynamicsPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world.
    protected: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    protected: physics::PhysicsEnginePtr physics;

    /// \brief Pointer to model containing plugin.
    protected: physics::ModelPtr model;

    /// \brief Vehicle life coefficients
    /// Lift Force = C_L * q * S
    /// where q (dynamic pressure) = 0.5 * rho * Va^2
    /// C_L = CL0 + CLalpha*alpha + CLq*c*q/(2*Va) + CLde*de
    protected: double CL0, CLalpha, CLq, CLde;

    /// \brief Vehicle drag coefficients
    /// Drag Force = C_D * q * S
    /// where q (dynamic pressure) = 0.5 * rho * Va^2
    /// C_D = CD0 + CDalpha*alpha + CDalpha2*alpha^2 + CDq*c*q/(2*Va) + CDbeta2*beta^2 + CDbeta*beta + CDde*de
    protected: double CD0, CDalpha, CDalpha2, CDq, CDbeta2, CDbeta, CDde;

    /// \brief Vehicle side-force coefficients
    /// Side Force = C_Y * q * S
    /// where q (dynamic pressure) = 0.5 * rho * Va^2
    /// C_Y = CY0 + CYbeta*beta + CYp*b*p/(2*Va) + CYr*b*r/(2*Va) + CYda*da + CYdr*dr
    protected: double CY0, CYbeta, CYp, CYr, CYda, CYdr;

    /// \brief Vehicle roll coefficients
    /// Roll Moment = C_l * q * S * b
    /// where q (dynamic pressure) = 0.5 * rho * Va^2
    /// C_l = Cl0 + Clbeta*beta + Clp*b*p/(2*Va) + Clr*b*r/(2*Va) + Clda*da + Cldr*dr
    protected: double Cl0, Clbeta, Clp, Clr, Clda, Cldr;

    /// \brief Vehicle pitch coefficients
	/// Pitch Moment = C_m * q * S * c
	/// where q (dynamic pressure) = 0.5 * rho * Va^2
    /// C_m = Cm0 + Cmalpha*alpha + Cmq*c*q/(2*Va) + Cmde*de
    protected: double Cm0, Cmalpha, Cmq, Cmde;

    /// \brief Vehicle yaw coefficients
	/// Pitch Moment = C_n * q * S * b
	/// where q (dynamic pressure) = 0.5 * rho * Va^2
    /// C_n = Cn0 + Cnbeta*beta + Cnp*b*p/(2*Va) + Cnr*b*r/(2*Va) + Cnda*da + Cndr*dr
    protected: double Cn0, Cnbeta, Cnp, Cnr, Cnda, Cndr;

    /// \brief Geometric parameters
    /// Planform area, S (m^2)
    /// Mean chord of wing, c (m)
    /// Wingspan, b (m)
    protected: double b, c, S;

    /// \brief Stall parameters for flat plate model using sigmoid function
    /// Flat plate lift coefficient, C_L_{flat plate} = 2*sign(alpha)*sin^2(alpha)*cos(alpha)
    /// Flat plate pitch coefficient, C_m_{flat plate} = C_mfp*sign(alpha)*sin^2(alpha)
    /// Stall angle, alpha_0
    /// Transition rate of blending function, M_trans
    /// Flat plate pitch constant, C_mfp
    ///
    /// Overall lift coefficient, C_L_alpha = (1-sigmoid(alpha))*(CL_alpha) + sigmoid(alpha)*C_L_{flat plate}
    /// Overall pitch coefficient, C_m_alpha = (1 - sigmoid(alpha))*CmAlpha + sigmoid(alpha)*(C_mfp*sign(alpha)*sin^2(alpha));
    ///
    /// where sigmoid(alpha) = (1 + exp(-M*(alpha-alpha_0)) + exp(M*(alpha+alpha_0)))/((1 + exp(-M*(alpha-alpha_0)))*(1 + exp(M*(alpha+alpha_0))))
    /// is a smoothing function to handle the transition from the wing aerodynamics to flat plate stall

    protected: double alpha_0, M_trans, C_mfp;

    /// \brief air density
    /// To be pulled from the atmospheric model, dependent on system altitude
    protected: double rho;

    /// \brief Pointer to link describing vehicle body.
    protected: physics::LinkPtr link;

    /// \brief String describing the vehicle configuration
    protected: vehicle_config vehicleConfig;

    /// \brief Pointers to the actuator joints for a standard config aircraft (default config)
    protected: physics::JointPtr controlJoint_Elevator;
    protected: physics::JointPtr controlJoint_portAileron;
    protected: physics::JointPtr controlJoint_starboardAileron;
    protected: physics::JointPtr controlJoint_Rudder;

    /// \brief Pointers to the actuator joints for a VTail config aircraft
    protected: physics::JointPtr controlJoint_portRuddervator;
    protected: physics::JointPtr controlJoint_starboardRuddervator;

    /// \brief Pointers to the actuator joints for a flying wing aircraft
    protected: physics::JointPtr controlJoint_portElevon;
    protected: physics::JointPtr controlJoint_starboardElevon;

    /// \brief SDF for this plugin;
    protected: sdf::ElementPtr sdf;

    /// \brief Simple hash function for handling vehicle types
    protected: vehicle_config stringHash(std::string const& inputString);

  };
}
#endif
