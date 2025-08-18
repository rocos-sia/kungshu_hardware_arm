/*
 * This software is dual-licensed under GPLv3 and a commercial
 * license. See the file LICENSE.md distributed with this software for
 * full license information.
 */

#include <spdlog/spdlog.h>
#include "kungshu_hardware_arm/fieldbus.h"

int main(int argc, char *argv[]) {
    spdlog::info("Hello, Arm Node!");

    KSH::Fieldbus fieldbus("enp114s0");

    fieldbus.Start();

    while (true) {

      for (int i = 0; i < 3; i++) {
        spdlog::info("status: {}; pos: {}; vel: {}; torque: {}; auxpos: {}; analog: {}",
             fieldbus.GetStatusWord(i),
             fieldbus.GetPosition(i),
             fieldbus.GetVelocity(i),
             fieldbus.GetTorque(i),
             fieldbus.GetAuxiliaryPosition(i),
             fieldbus.GetAnalogInput(i));
      }

      osal_usleep(1000000);
    }

    return (0);
}
