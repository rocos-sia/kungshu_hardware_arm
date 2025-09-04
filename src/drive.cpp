//
// Created by think on 8/20/25.
//

#include "kungshu_hardware_arm/drive.h"

using namespace KSH;

Drive::Drive(int id, Inputs* inputs, Outputs* outputs)
    : id_(id), inputs_(inputs), outputs_(outputs) {
  // Initialize the drive with default values
  controlword_.setAllFalse();
  current_drive_state_ = DriveState::NA;
  target_drive_state_ = DriveState::NA;

  is_guard_thread_running_ = true;

  AddToDriveGuard(this);
}

int8_t Drive::GetStatus() {
  // current_drive_state_ = getDriverState(0);
  switch (current_drive_state_) {
    case DriveState::SwitchOnDisabled:
      return 0;
    case DriveState::OperationEnabled:
      return 1;
    case DriveState::Fault:
      return -1;
    default:
      // spdlog::info("Unknown drive state: {}, statusword: {}", static_cast<int>(current_drive_state_), (int)inputs_->status_word);
      return 3;
  }
}

DriveState Drive::getDriverState(int id) {
  Statusword status;
  status.setFromRawStatusword(inputs_->status_word);

  return status.getDriveState();
}

Controlword Drive::getNextStateTransitionControlword(
    const DriveState& requestedDriveState,
    const DriveState& currentDriveState) {

  // spdlog::info("Drive: {}, requested state: {}, current state: {}",id_, static_cast<int>(requestedDriveState), static_cast<int>(currentDriveState));

  Controlword controlword;
  controlword.setAllFalse();
  switch (requestedDriveState) {
    case DriveState::SwitchOnDisabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          spdlog::info("SwitchOnDisabled is already reached for drive {}", id_);
          std::cerr << "[getNextStateTransitionControlword] "
                    << "drive state has already been reached" << std::endl;
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition7();
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition10();
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition9();
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          // std::cerr << "Driver " << id_
          //           << ": PDO state transition not implemented for '"
          //           << std::endl;
          break;
      }
      break;

    case DriveState::ReadyToSwitchOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          std::cerr << "Driver " << id_
                    << ": drive state has already been reached for '"
                    << std::endl;
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition6();
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition8();
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          // std::cerr << "Driver " << id_
          //           << ": PDO state transition not implemented for '"
          //           << std::endl;
          break;
      }
      break;

    case DriveState::SwitchedOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition3();
          break;
        case DriveState::SwitchedOn:
          std::cerr << "Driver " << id_
                    << ": drive state has already been reached for '"
                    << std::endl;
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition5();
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          // std::cerr << "Driver " << id_
          //           << ": PDO state transition not implemented for '"
          //           << std::endl;
          break;
      }
      break;

    case DriveState::OperationEnabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition3();
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition4();
          break;
        case DriveState::OperationEnabled:
          std::cerr << "Driver " << id_
                    << ": drive state has already been reached for '"
                    << std::endl;
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          // std::cerr << "Driver " << id_
          //           << ": PDO state transition not implemented for '"
          //           << std::endl;
          break;
      }
      break;

    case DriveState::QuickStopActive:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition3();
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition4();
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition11();
          break;
        case DriveState::QuickStopActive:
          std::cerr << "Driver " << id_
                    << ": drive state has already been reached for '"
                    << std::endl;
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          // std::cerr << "Driver " << id_
          //           << ": PDO state transition not implemented for '"
          //           << std::endl;
          break;
      }
      break;

    default:
      std::cerr << "Driver " << id_
                << ": PDO state cannot be reached for '" << std::endl;
  }

  return controlword;
}

void Drive::engageStateMachine() {
  // elapsed time since the last new controlword
  auto microsecondsSinceChange =
      (std::chrono::duration_cast<std::chrono::microseconds>(
           std::chrono::system_clock::now() - drive_state_change_time_point_))
          .count();

  // get the current state
  // since we wait until "hasRead" is true, this is guaranteed to be a newly
  // read value
  //        const DriveState currentDriveState = reading_.getDriveState();

  // check if the state change already was successful:
  if (current_drive_state_ == target_drive_state_) {
    num_of_successful_target_state_readings_++;
    //            if (numberOfSuccessfulTargetStateReadings_ >=
    //            configuration_.minNumberOfSuccessfulTargetStateReadings) {
    if (num_of_successful_target_state_readings_ >= 3) {
      // disable the state machine
      conduct_state_change_ = false;
      num_of_successful_target_state_readings_ = 0;
      state_change_successful_ = true;

      return;
    }
  } else if (microsecondsSinceChange > 20000) {
    // get the next controlword from the state machine
    controlword_ = getNextStateTransitionControlword(target_drive_state_,
                                                     current_drive_state_);
    drive_state_change_time_point_ = std::chrono::system_clock::now();
    outputs_->control_word =
        controlword_.getRawControlword();  // set control word
  }
}

bool Drive::setDriverState(const DriveState& driveState, bool waitForState) {
  bool success = false;
  /*
   ** locking the mutex_
   ** This is not done with a lock_guard here because during the waiting time
   *the
   ** mutex_ must be unlocked periodically such that PDO writing (and thus state
   ** changes) may occur at all!
   */
  //        mutex_.lock();

  // reset the "stateChangeSuccessful_" flag to false such that a new successful
  // state change can be detected
  state_change_successful_ = false;

  // make the state machine realize that a state change will have to happen
  conduct_state_change_ = true;

  // overwrite the target drive state
  target_drive_state_ = driveState;

  // set the hasRead flag to false such that at least one new reading will be
  // available when starting the state change
  //        hasRead_ = false;

  // set the time point of the last pdo change to now
  drive_state_change_time_point_ = std::chrono::system_clock::now();

  // set a temporary time point to prevent getting caught in an infinite loop
  auto driveStateChangeStartTimePoint = std::chrono::system_clock::now();

  // return if no waiting is requested
  if (!waitForState) {
    // unlock the mutex
    //            mutex_.unlock();
    // return true if no waiting is requested
    return true;
  }

  // Wait for the state change to be successful
  // during the waiting time the mutex MUST be unlocked!

  while (true) {
    // break loop as soon as the state change was successful
    if (state_change_successful_) {
      success = true;
      break;
    }

    // break the loop if the state change takes too long
    // this prevents a freezing of the end user's program if the hardware is not
    // able to change it's state.
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now() - driveStateChangeStartTimePoint);
    if (duration_us.count() >
        300000) {  // wait for 100ms  TODO:
                   // configuration_.driveStateChangeMaxTimeout
      // std::cout << "Drive " << id_ << " takes too long (" << duration_us.count() / 1000.0
      //           << " ms) to switch state! current: " << (int)current_drive_state_ << " , target: " << (int)target_drive_state_ << std::endl;
      break;
    }
    // unlock the mutex during sleep time
    //            mutex_.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
    // lock the mutex to be able to check the success flag
    //            mutex_.lock();
  }
  // unlock the mutex one last time
  //        mutex_.unlock();
  return success;
}

void Drive::AddToDriveGuard(Drive* drive) {
  driver_list_.push_back(drive);  // Add driver to driver_list
}

std::atomic<bool> Drive::is_guard_thread_running_ = true;

std::vector<Drive*> Drive::driver_list_ = {};

std::thread Drive::guard_thread_ =
    std::thread([]() {
      spdlog::info("Drive Guard is running on thread.");

      while (is_guard_thread_running_) {
        for (auto driver : driver_list_) {
          driver->current_drive_state_ = driver->getDriverState(0);

          if (driver->current_drive_state_ == DriveState::Fault) {
          }

          if (driver->conduct_state_change_) driver->engageStateMachine();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(40));
      }

      spdlog::info("Drive Guard thread is terminated.");
    });
