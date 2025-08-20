//
// Created by think on 8/20/25.
//

#include "kungshu_hardware_arm/drive.h"

namespace KSH {

Drive::Drive(Inputs* inputs, Outputs* outputs)
    : inputs_(inputs), outputs_(outputs) {
  // Initialize the drive with default values
  id_ = 0;  // Default ID, can be set later

  controlword_.setAllFalse();
  current_drive_state_ = DriveState::NotReadyToSwitchOn;
  target_drive_state_ = DriveState::NotReadyToSwitchOn;

  if (guard_thread_ && guard_thread_->joinable()) {
    spdlog::warn("Guard thread is already running, stopping it first.");
    is_guard_thread_running_ = false;
    guard_thread_->join();
  }

  is_guard_thread_running_ = true;

  guard_thread_ = std::make_shared<std::thread>([=]() {
    std::cout << "Drive Guard is running on thread "
              << std::this_thread::get_id() << std::endl;
    while (is_guard_thread_running_) {
      current_drive_state_ = getDriverState(0);

      if (current_drive_state_ == DriveState::Fault) {

        //        is_guard_thread_running_ = false;
        //        return;
      }

      if (conduct_state_change_) engageStateMachine();

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "Drive Guard thread is terminated." << std::endl;
  });


}

DriveState Drive::getDriverState(int id) {
  Statusword status;
  status.setFromRawStatusword(inputs_->status_word);
}

Controlword Drive::getNextStateTransitionControlword(
    const DriveState& requestedDriveState,
    const DriveState& currentDriveState) {
  Controlword controlword;
  controlword.setAllFalse();
  switch (requestedDriveState) {
    case DriveState::SwitchOnDisabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
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
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "PDO state transition not implemented for '"
                    << std::endl;
      }
      break;

    case DriveState::ReadyToSwitchOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "drive state has already been reached for '"
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
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "PDO state transition not implemented for '"
                    << std::endl;
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
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "drive state has already been reached for '"
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
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "PDO state transition not implemented for '"
                    << std::endl;
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
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "drive state has already been reached for '"
                    << std::endl;
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "PDO state transition not implemented for '"
                    << std::endl;
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
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "drive state has already been reached for '"
                    << std::endl;
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                    << "PDO state transition not implemented for '"
                    << std::endl;
      }
      break;

    default:
      std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                << "PDO state cannot be reached for '" << std::endl;
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
        150000) {  // wait for 100ms  TODO:
                   // configuration_.driveStateChangeMaxTimeout
      std::cout << "It takes too long (" << duration_us.count() / 1000.0
                << " ms) to switch state!" << std::endl;
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

}  // namespace KSH