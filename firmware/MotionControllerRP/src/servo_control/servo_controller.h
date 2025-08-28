#pragma once

#include "hardware/MT6835_encoder.h"
#include "hardware/TB6612_motor_driver.h"
#include "encoder_lut.h"
#include "pid.h"

class ServoController {
  public:
    // use defines instead of virtual functions for speed
    // TODO: check if this makes any difference and change accordingly
    typedef TB6612MotorDriver MOTOR_DRIVER_TYPE;
    typedef MT6835Encoder ENCODER_TYPE;

  public:
    ServoController(MOTOR_DRIVER_TYPE& motor_driver, ENCODER_TYPE& encoder, int32_t motor_pole_pairs);

    void init(float max_motor_amplitude);

    void set_encoder_lut(LookupTable& enc_to_pos_lut);

    void update(float target_motor_pos,
                float dt,
                float one_over_dt);

    bool at_position(float motor_pos_eps);

    float read_position();

    float get_position();

    float get_position_error();

    bool move_to(float target_motor_angle, 
                 float at_pos_motor_angle_eps, 
                 float settle_time_ms,
                 float timeout_us);

    void move_to_open_loop(float target_motor_angle, 
                           float angular_velocity);

    void home(float motor_velocity, float search_range, float current=0.2f);
    
    ENCODER_TYPE& get_encoder();
    MOTOR_DRIVER_TYPE& get_motor_driver();
    float output;

  private:
    float encoder_angle_to_motor_pos(int32_t encoder_angle_raw);
    float motor_pos_to_field_angle(float motor_pos);
    float motor_velocity_to_field_velocity(float v);

  private:
    ENCODER_TYPE& encoder;
    MOTOR_DRIVER_TYPE& motor_driver;
    LookupTable enc_to_pos_lut;

    LowpassFilter velocity_lowpass;
    PIDController pos_controller;
    PIDController velocity_controller;

    float motor_pos = 0;                  // current motor position
    float motor_pos_prev = 0;             // previous motor position
    float pos_error = 0;                  // current position error as computed by upate()
    float velocity = 0;                   // current velocity estimate

    float motorpos_to_field_angle = 0;    // conversion factor derived from pole pair count
    float motor_current_amplitude = 0.5f;
};

//*** FUNCTIONS **************************************************************/

bool build_motor_to_enc_angle_lut(
  LookupTable& lut,
  ServoController& servo_controller,
  float min_motor_angle,
  float max_motor_angle,
  size_t size);