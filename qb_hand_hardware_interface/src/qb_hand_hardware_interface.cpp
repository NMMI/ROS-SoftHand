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

#include <qb_hand_hardware_interface/qb_hand_hardware_interface.h>

using namespace qb_hand_hardware_interface;

qbHandHW::qbHandHW()
    : qbDeviceHW(std::make_shared<qb_hand_transmission_interface::qbHandVirtualTransmission>(), {"synergy_joint"}, {"synergy_joint"}) {
  //TODO: check that closure_ticks_limit has the same value of device_.position_limits (device_ cannot be retrieved before calling the base constructor which needs the transmission already setup...)
}

qbHandHW::~qbHandHW() {

}

std::vector<std::string> qbHandHW::getJoints() {
  return joints_.names;
}

void qbHandHW::read(const ros::Time& time, const ros::Duration& period) {
  // read actuator state from the hardware (convert to proper measurement units)
  qb_device_hardware_interface::qbDeviceHW::read(time, period);
}

void qbHandHW::write(const ros::Time& time, const ros::Duration& period) {
  // send actuator command to the hardware (saturate and convert to proper measurement units)
  qb_device_hardware_interface::qbDeviceHW::write(time, period);
}