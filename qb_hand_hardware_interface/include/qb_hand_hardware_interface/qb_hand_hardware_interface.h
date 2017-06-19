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

#ifndef QB_HAND_HARDWARE_INTERFACE_H
#define QB_HAND_HARDWARE_INTERFACE_H

// internal libraries
#include <qb_device_hardware_interface/qb_device_hardware_interface.h>
#include <qb_hand_hardware_interface/qb_hand_transmission_interface.h>

namespace qb_hand_hardware_interface {
/**
 * The qbrobotics \em qbhand HardWare interface implements the specific structures to manage the communication with the
 * \em qbhand device. It exploits the features provided by the base device-independent hardware interface and the
 * specific transmission interface.
 * \sa qb_device_hardware_interface::qbDeviceHW, qb_hand_transmission_interface::qbHandVirtualTransmission
 */
class qbHandHW : public qb_device_hardware_interface::qbDeviceHW {
 public:
  /**
   * Initialize the \p qb_device_hardware_interface::qbDeviceHW with the specific transmission interface and actuator
   * and joint names.
   * \sa qb_device_hardware_interface::qbDeviceHW(), qb_hand_transmission_interface::qbHandVirtualTransmission
   */
  qbHandHW();

  /**
   * Do nothing.
   */
  virtual ~qbHandHW();

  /**
   * \return The vector of controller joint names.
   */
  std::vector<std::string> getJoints();

  /**
   * Call the base \p qb_device_hardware_interface::read().
   * \param time The current time.
   * \param period The time passed since the last call to this method, i.e. the control period.
   * \sa qb_device_hardware_interface::read()
   */
  void read(const ros::Time& time, const ros::Duration& period);

  /**
   * Call the base \p qb_device_hardware_interface::write().
   * \param time The current time.
   * \param period The time passed since the last call to this method, i.e. the control period.
   * \sa qb_device_hardware_interface::write()
   */
  void write(const ros::Time& time, const ros::Duration& period);
};
typedef std::shared_ptr<qbHandHW> qbHandHWPtr;
}  // namespace qb_hand_hardware_interface

#endif // QB_HAND_HARDWARE_INTERFACE_H