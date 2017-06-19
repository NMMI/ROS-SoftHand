/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2016-2017, qbrobotics®
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 *  following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *    following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef QB_HAND_TRANSMISSION_H
#define QB_HAND_TRANSMISSION_H

#include <transmission_interface/transmission.h>
#include <control_toolbox/filters.h>
//TODO: switch to a more complete filter library

namespace qb_hand_transmission_interface {
/**
 * The qbrobotics \em qbhand Transmission interface implements the specific \p transmission_interface::Transmission to
 * convert from \em qbhand motors state to its equivalent joint state representation, and vice versa.
 * \sa qb_device_transmission_interface::qbDeviceTransmissionResources
 */
class qbHandVirtualTransmission : public transmission_interface::Transmission {
 public:
  /**
   * Build the \em qbhand transmission with default velocity and effort scale factors (respectively \p 0.2 and \p 0.001),
   * and retrieve the position scale factor from the parameter \p "~closure_ticks_limit" expressed in \em ticks in the
   * Parameter Server following the formula \f$f_{pos} = \frac{1}{closure\_limits}\f$ (if not found it uses the default
   * hand closure limit, which is \p 19000 \em ticks).
   */
  qbHandVirtualTransmission()
      : qbHandVirtualTransmission(1./ros::param::param<int>("~closure_ticks_limit", 19000)) {}

  /**
   * Build the \em qbhand transmission with the given position scale factor and the default velocity and effort scale
   * factors (respectively \p 0.2 and \p 0.001).
   * \param position_factor Motor position \em ticks to closure percent value [\p 0, \p 1] scale factor.
   */
  qbHandVirtualTransmission(const double &position_factor)
      : qbHandVirtualTransmission(position_factor, 0.2, 0.001) {}

  /**
   * Build the \em qbhand transmission with the given scale factors.
   * \param position_factor Motor position \em ticks to closure percent value [\p 0, \p 1] scale factor.
   * \param velocity_factor Exponential filter smoothing factor.
   * \param effort_factor Motor current \em mA to joint effort \em A scale factor.
   */
  qbHandVirtualTransmission(const double &position_factor, const double &velocity_factor, const double &effort_factor)
      : Transmission(),
        position_factor_(position_factor),
        velocity_factor_(velocity_factor),
        effort_factor_(effort_factor) {}

  /**
   * Transform \em effort variables from actuator to joint space.
   * \param actuator Actuator-space variables.
   * \param[out] joint Joint-space variables.
   * \pre Actuator and joint effort vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void actuatorToJointEffort(const transmission_interface::ActuatorData& actuator, transmission_interface::JointData& joint) {
    ROS_ASSERT(numActuators() == actuator.effort.size() && numJoints() == joint.effort.size());
    ROS_ASSERT(actuator.effort[0] && joint.effort[0]);

    *joint.effort[0] = *actuator.effort[0] * effort_factor_;
  }

  /**
   * Transform \em velocity variables from actuator to joint space.
   * \param actuator Actuator-space variables.
   * \param[out] joint Joint-space variables.
   * \pre Actuator and joint velocity vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void actuatorToJointVelocity(const transmission_interface::ActuatorData& actuator, transmission_interface::JointData& joint) {
    ROS_ASSERT(numActuators() == actuator.velocity.size() && numJoints() == joint.velocity.size());
    ROS_ASSERT(actuator.velocity[0] && joint.velocity[0]);

    // *actuator.velocity[0] is the current measured velocity in [ticks/s] while *joint.velocity[0] is the previous step velocity in [percent/s]
    *joint.velocity[0] = filters::exponentialSmoothing(*actuator.velocity[0] * position_factor_, *joint.velocity[0], velocity_factor_);
  }

  /**
   * Transform \em position variables from actuator to joint space.
   * \param actuator Actuator-space variables.
   * \param[out] joint Joint-space variables.
   * \pre Actuator and joint position vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void actuatorToJointPosition(const transmission_interface::ActuatorData& actuator, transmission_interface::JointData& joint) {
    ROS_ASSERT(numActuators() == actuator.position.size() && numJoints() == joint.position.size());
    ROS_ASSERT(actuator.position[0] && joint.position[0]);

    *joint.position[0] = *actuator.position[0] * position_factor_;
  }

  /**
   * \return The current position scale factor.
   */
  inline const double& getPositionFactor() const { return position_factor_; }

  /**
   * \return The current velocity scale factor.
   */
  inline const double& getVelocityFactor() const { return velocity_factor_; }

  /**
   * \return The current effort scale factor.
   */
  inline const double& getEffortFactor() const { return effort_factor_; }

  /**
   * Transform \em effort variables from joint to actuator space.
   * \param joint Joint-space variables.
   * \param[out] actuator Actuator-space variables.
   * \pre Actuator and joint effort vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void jointToActuatorEffort(const transmission_interface::JointData& joint, transmission_interface::ActuatorData& actuator) {
    ROS_ASSERT(numActuators() == actuator.effort.size() && numJoints() == joint.effort.size());
    ROS_ASSERT(actuator.effort[0] && joint.effort[0]);

    // the qbhand cannot be controlled in current, but this could help in the near future
    *actuator.effort[0] = *joint.effort[0] / effort_factor_;
  }

  /**
   * Transform \em velocity variables from joint to actuator space.
   * \param joint Joint-space variables.
   * \param[out] actuator Actuator-space variables.
   * \pre Actuator and joint velocity vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void jointToActuatorVelocity(const transmission_interface::JointData& joint, transmission_interface::ActuatorData& actuator) {
    ROS_ASSERT(numActuators() == actuator.velocity.size() && numJoints() == joint.velocity.size());
    ROS_ASSERT(actuator.velocity[0] && joint.velocity[0]);

    // the qbhand cannot be controlled in velocity
    *actuator.velocity[0] = 0.0;
  }

  /**
   * Transform \em position variables from joint to actuator space.
   * \param joint Joint-space variables.
   * \param[out] actuator Actuator-space variables.
   * \pre Actuator and joint position vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void jointToActuatorPosition(const transmission_interface::JointData& joint, transmission_interface::ActuatorData& actuator) {
    ROS_ASSERT(numActuators() == actuator.position.size() && numJoints() == joint.position.size());
    ROS_ASSERT(actuator.position[0] && joint.position[0]);

    *actuator.position[0] = *joint.position[0] / position_factor_;
  }

  /**
   * \return The number of actuators of this transmission, i.e always 1 for the \em qbhand.
   */
  inline std::size_t numActuators() const { return 1; }

  /**
   * \return The number of joints of this transmission, i.e always 1 for the \em qbhand.
   */
  inline std::size_t numJoints() const { return 1; }

 private:
  double position_factor_;
  double velocity_factor_;
  double effort_factor_;
};
}  // namespace qb_hand_transmission_interface

#endif // QB_HAND_TRANSMISSION_H