/*
 * This software is dual-licensed under GPLv3 and a commercial
 * license. See the file LICENSE.md distributed with this software for
 * full license information.
 */
#include <rclcpp/rclcpp.hpp>

#include "kungshu_hardware_arm/arm_node.h"

#include <spdlog/spdlog.h>

#include <sys/mman.h>
#include <sys/utsname.h>

/********************************************************************************/
/** Enable real-time environment
*
* Return: EC_E_NOERROR in case of success, EC_E_ERROR in case of failure.
*/
int EnableRealtimeEnvironment(void) {
    struct utsname SystemName;
    int nMaj, nMin, nSub;
    struct timespec ts;
    int nRetval;
    int dwResult = -1;
    bool bHighResTimerAvail;
    struct sched_param schedParam;

    /* master only tested on >= 2.6 kernel */
    nRetval = uname(&SystemName);
    if (nRetval != 0) {
        std::cerr << "ERROR calling uname(), required Linux kernel >= 2.6" << std::endl;
        dwResult = -1;
        goto Exit;
    }
    sscanf(SystemName.release, "%d.%d.%d", &nMaj, &nMin, &nSub);
    if (!(((nMaj == 2) && (nMin == 6)) || (nMaj >= 3))) {
        spdlog::critical("ERROR - detected kernel = {}.{}.{}, required Linux kernel >= 2.6", nMaj, nMin, nSub);
        dwResult = -1;
        goto Exit;
    }

    /* request realtime scheduling for the current process
    * This value is overwritten for each individual task
    */
    schedParam.sched_priority = 99; /* 1 lowest priority, 99 highest priority */
    nRetval = sched_setscheduler(0, SCHED_FIFO, &schedParam);
    if (nRetval == -1) {
        std::cerr << "ERROR - cannot change scheduling policy!\n"
                  << "root privilege is required or realtime group has to be joined!" << std::endl;
        goto Exit;
    }

    /* disable paging */
    nRetval = mlockall(MCL_CURRENT | MCL_FUTURE);
    if (nRetval == -1) {
        std::cerr << "ERROR - cannot disable paging!" << std::endl;
        dwResult = -1;
        goto Exit;
    }

    /* check if high resolution timers are available */
    if (clock_getres(CLOCK_MONOTONIC, &ts)) {
        bHighResTimerAvail = false;
    } else {
        bHighResTimerAvail = !(ts.tv_sec != 0 || ts.tv_nsec != 1);
    }
    if (!bHighResTimerAvail) {
        std::cerr << "WARNING: High resolution timers not available" << std::endl;
    }

    /* set type of OsSleep implementation  (eSLEEP_USLEEP, eSLEEP_NANOSLEEP or eSLEEP_CLOCK_NANOSLEEP) */
//    OsSleepSetType(eSLEEP_CLOCK_NANOSLEEP);

    dwResult = 0;
    Exit:
    return dwResult;
}

void SignalHandler(int nSignal) {
  //    bRun = false;
  exit(-1);
}

int main(int argc, char *argv[]) {
  //! Linux real-time settings
  EnableRealtimeEnvironment();
  {
    sigset_t SigSet;
    int nSigNum = SIGALRM;
    sigemptyset(&SigSet);
    sigaddset(&SigSet, nSigNum);
    sigprocmask(SIG_BLOCK, &SigSet, NULL);
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);
  }

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset); // 清空CPU集合

  // 设置进程可以运行在CPU核心0和1上
  CPU_SET(0, &cpuset);

  if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == -1) {
    spdlog::critical("sched_setaffinity");
    return 1;
  }

  spdlog::info("Cpu affinity set success\n");


  rclcpp::init(argc, argv);

  auto node = std::make_shared<KSH::ArmNode>();

  spdlog::info("Hello!! Kungshu Arm!");

  rclcpp::spin(node);

  rclcpp::shutdown();

  // while (true) {
  //
  //   for (int i = 0; i < 3; i++) {
  //     spdlog::info("status: {}; pos: {}; vel: {}; torque: {}; auxpos: {};
  //     analog: {}",
  //          fieldbus.GetStatusWord(i),
  //          fieldbus.GetPosition(i),
  //          fieldbus.GetVelocity(i),
  //          fieldbus.GetTorque(i),
  //          fieldbus.GetAuxiliaryPosition(i),
  //          fieldbus.GetAnalogInput(i));
  //   }
  //
  //   osal_usleep(1000000);
  // }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ByeBye, Kungshu Arm!");

  return (0);
}
