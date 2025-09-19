// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#pragma once

#include "servo_controller.h"

//*** CLASS *****************************************************************************

/**
 * This class implements the homing procedure for a single actuator. To allow for parallel 
 * homing the class is stateful and has an update() function, that can be called togeather
 * with the updates of other homing controllers inside a loop.
 * 
 * Homing procedure:
 *   1. move axis in negative direction until a physical hard stop is reached
 *   2. reset encoder period
 *   3. back off slightly from the hard stop
 */
class HomingController {
  public:
    HomingController();

    // Starts the homing cycle, motor_velocity can be negative and defines the homing direction.
    // WARNING: Servo loop updates (including encoder reads) must be completely disabled during homing.
    bool run_blocking(ServoController* servo_controller, float motor_velocity, float search_range, float current);
    void start(ServoController* servo_controller, float motor_velocity, float search_range, float current);
    void update();
    void finalize();

    bool is_finished() const;
    bool is_successful() const;
    float get_home_encoder_angle() const;

  private:
    void on_endstop_detected();
    float compute_eval_pos_delta(float pos, float field_angle_delta);

  private:
    enum class State {
      Idle,
      Initializing,
      Homing,
      Done
    };

    ServoController* servo_ctrl;

    // Configuration params
    float field_velocity = 0.0f; // defines homing direction
    float field_angle_search_range = 0.0f;
    float homing_current = 0.0f;
    float initial_current = 0.0f;
    float retract_field_velocity = 0.0f;

    // State machine
    State state = State::Idle;
    bool search_failed = false;

    // Timing
    uint64_t last_time = 0;

    // Offsets and tracking
    float start_field_angle = 0.0f;
    float field_angle_offset = 0.0f;
    float last_eval_field_angle_offset = 0.0f;
    float last_eval_encoder_angle = 0.0f;
    float eval_field_angle_delta = 0.0f;
    float expected_encoder_delta = 0.0f;

    float home_encoder_angle = 0.0f;
};
