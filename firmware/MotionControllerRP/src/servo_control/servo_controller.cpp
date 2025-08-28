#include "hardware/timer.h"
#include "Arduino.h"

#include "servo_controller.h"
#include "utilities/logging.h"
#include "utilities/math_constants.h"

#include <algorithm>

ServoController::ServoController(
  MOTOR_DRIVER_TYPE& motor_driver, 
  ENCODER_TYPE& encoder, 
  int32_t motor_pole_pair_count) :
    motor_driver(motor_driver),
    encoder(encoder), 
    motorpos_to_field_angle(motor_pole_pair_count) 
{
  motor_pos = 0.0f;
  pos_error = 0.0f;
}

void ServoController::init(float max_motor_amplitude) {
  ServoController::motor_current_amplitude = max_motor_amplitude;

  // setup motor driver
  motor_driver.begin();
  motor_driver.set_amplitude(0.0f, true); 
  motor_driver.enable();
  motor_driver.set_field_angle(0.0f);

  // soft start
  for(int i=0; i<100; i++) {
    motor_driver.set_amplitude(motor_current_amplitude*float(i)/(100-1), true);
    sleep_ms(1);
  }

  velocity_lowpass.set_time_constant(0.004f);
  pos_controller.set_parameter(75.0f, 50000.0f, 0.0f, Constants::PI_F*2.0F, Constants::PI_F*0.5F);
  velocity_controller.set_parameter(0.2f, 150.0f, 0.0f, Constants::PI_F*0.45f, Constants::PI_F*0.45f);
}

void ServoController::set_encoder_lut(LookupTable& enc_to_pos_lut) {
  ServoController::enc_to_pos_lut = enc_to_pos_lut;
}

void ServoController::update(float target_motor_pos, float dt, float one_over_dt) { 
  // read encoder
  int32_t encoder_angle_raw = encoder.read_abs_angle_raw();

  // convert encoder angle to motor pos using LUT and compute field angle
  motor_pos = encoder_angle_to_motor_pos(encoder_angle_raw);
  float field_angle = motor_pos_to_field_angle(motor_pos);

  // position controll loop
  pos_error = target_motor_pos-motor_pos;
  float velocity_target = pos_controller.compute(pos_error, dt, one_over_dt);

  // velocity controll loop
  float velocity_unfiltered = (motor_pos - motor_pos_prev)*one_over_dt;
  velocity = velocity_lowpass.update(velocity_unfiltered, dt);
  float torque_target = velocity_controller.compute(velocity_target-velocity, dt, one_over_dt);

  // torque controll loop
  output = torque_target;

  // set new field direction
  // motor_driver.set_amplitude(std::clamp(abs(output*10.0f), 0.1f, 0.5f), false);
  motor_driver.set_field_angle(field_angle + output); 

  // store values for next update
  motor_pos_prev = motor_pos;
}

bool ServoController::at_position(float motor_pos_eps) {
  return fabs(pos_error) < motor_pos_eps;
}

float ServoController::read_position() {
  return encoder_angle_to_motor_pos(encoder.read_abs_angle_raw());
}

float ServoController::get_position() {
  return motor_pos;
}

float ServoController::get_position_error() {
  return pos_error;
}

bool ServoController::move_to(float target_motor_pos, float at_pos_eps, float settle_time_s, float timeout_s) {
  uint64_t start_time_us = time_us_64();
  uint64_t time_us = start_time_us;
  uint64_t pos_reached_time_us = 0;
  uint64_t last_time = time_us;
  uint32_t settle_time_us = settle_time_s*1e6f;
  uint32_t timeout_us = timeout_s*1e6f;

  do {
    // get time and detla time
    time_us = time_us_64();
    float dt = float(time_us - last_time)*1e-6f;
    last_time = time_us;

    float pos_error;
    update(target_motor_pos, dt, 1.0f/dt);

    // check if traget position reached
    if(pos_reached_time_us == 0) {
      if(at_position(at_pos_eps))
        pos_reached_time_us = time_us;
    } else {
      if(time_us-pos_reached_time_us > settle_time_us)
        return true;
    }

  } while(time_us-start_time_us < timeout_us);

  return false;
}

void ServoController::move_to_open_loop(float target_motor_pos, float motor_angular_velocity) {
  // Determine direction of movement at the start
  const bool moving_forward = target_motor_pos > motor_pos;

  uint64_t last_time = time_us_64();
  while ((moving_forward && motor_pos < target_motor_pos) ||
         (!moving_forward && motor_pos > target_motor_pos))
  {
    uint64_t time_us = time_us_64();
    float dt = float(time_us - last_time) * 1e-6f;
    last_time = time_us;

    // update encoder regularly
    encoder.read_abs_angle_raw();

    // update motor position
    motor_pos += moving_forward ? motor_angular_velocity * dt : -motor_angular_velocity * dt;

    // set field ange to new position
    float clamped_motor_pos = moving_forward ? std::min(motor_pos, target_motor_pos) : 
                                               std::max(motor_pos, target_motor_pos);
    motor_driver.set_field_angle(motor_pos_to_field_angle(clamped_motor_pos));
    sleep_us(100);
  }
  
  motor_pos = target_motor_pos;
}

void ServoController::home(float motor_velocity, float search_range, float current) {
  bool search_failed = false;
  float pos_offset = 0.0f;
  motor_driver.set_amplitude(current, true);
  float eval_pos_delta = (Constants::TWO_PI_F*0.1)/motorpos_to_field_angle;

  // determine expected encoder angle delta for motion of eval_pos_delta
  motor_driver.set_field_angle(motor_pos_to_field_angle(motor_pos+eval_pos_delta));
  sleep_ms(200);
  float angle1 = encoder.read_abs_angle();

  motor_driver.set_field_angle(motor_pos_to_field_angle(motor_pos));
  sleep_ms(200);
  float angle2 = encoder.read_abs_angle();
  float expected_encoder_delta = (angle2-angle1);
  
  // start homing search
  uint64_t last_time = time_us_64();
  float encoder_angle_prev = encoder.read_abs_angle();
  float last_eval_offset = 0.0f;

  while(true) {
    // compute time delta
    uint64_t time_us = time_us_64();
    float dt = float(time_us - last_time) * 1e-6f;
    last_time = time_us;

    // move motor and read encoder
    pos_offset += motor_velocity * dt;
    motor_driver.set_field_angle(motor_pos_to_field_angle(motor_pos+pos_offset));
    float encoder_angle = encoder.read_abs_angle();

    // check ratio of measured encoder delta to expected delta to determine motor stop
    if(fabs(last_eval_offset-pos_offset) > eval_pos_delta) {
      float encoder_delta = (encoder_angle - encoder_angle_prev);
      float encoder_velocity_ratio = encoder_delta/expected_encoder_delta;
      // Serial.printf(">encoder_velocity_ratio: %f\n", encoder_velocity_ratio);
      // Serial.printf(">encoder_velocity: %f\n", encoder_velocity);
      if(encoder_velocity_ratio < 0.05f)
        break;
      encoder_angle_prev = encoder_angle;
      last_eval_offset = pos_offset;
    }

    // check if search range exeeded
    if(fabs(pos_offset) > search_range) {
      search_failed = true;
      break;
    }
  }

  // reset positions
  motor_pos = 0;
  motor_driver.set_field_angle(0);
  sleep_ms(200);
  encoder.reset_abs_angle();
 
  // set normal motor current
  motor_driver.set_amplitude(motor_current_amplitude, true);
}

ServoController::ENCODER_TYPE& ServoController::get_encoder() {
  return encoder;
}

ServoController::MOTOR_DRIVER_TYPE& ServoController::get_motor_driver() {
  return motor_driver;
}

float ServoController::encoder_angle_to_motor_pos(int32_t encoder_angle_raw) {
  // TODO: use lut here
  if(enc_to_pos_lut.size() == 0) {
    int32_t encoder_cpr = encoder.get_rawcounts_per_rev();
    return encoder_angle_raw*Constants::TWO_PI_F/encoder_cpr/(7.5f*4);
  } else {
    return enc_to_pos_lut.evaluate(encoder_angle_raw);
  }
}

float ServoController::motor_pos_to_field_angle(float motor_pos) {
  return motor_pos*motorpos_to_field_angle;
}

float ServoController::motor_velocity_to_field_velocity(float v) {
  return v*motorpos_to_field_angle;
}

//*** FUNCTION ***********************************************************************************/

bool build_motor_to_enc_angle_lut(
            LookupTable& lut,
            ServoController& servo_controller,
            float min_motor_angle,
            float max_motor_angle,
            size_t size)
{
  LOG_INFO("Measuring motor to encoder angle lookup table...");
  float speed = 1.0f;
  float input_min = min_motor_angle;
  float input_max = max_motor_angle;

  lut.init(size, input_min, input_max);
  // float initial_pos = servo_controller.read_position();
  // move to starting position
  servo_controller.move_to_open_loop(min_motor_angle, 2.0f);
  servo_controller.get_encoder().reset_abs_angle(0); // Reset encoder to 0 at min_motor_angle

  float step = float(input_max - input_min) / (size - 1);

  // Measure in increasing direction
  for (size_t i = 0; i < size; ++i) {
    float target_motor_angle = input_min + step * i;
    servo_controller.move_to_open_loop(target_motor_angle, speed);
    // sleep_ms(0);
    float encoder_angle_raw = servo_controller.get_encoder().read_abs_angle_raw();
    lut.set_entry(i, encoder_angle_raw);
  }

  // Measure in decreasing direction (average with increasing direction)
  for (size_t i = 0; i < size; ++i) {
    float target_motor_angle = input_max - step * i; // Start from max and go down
    servo_controller.move_to_open_loop(target_motor_angle, speed);
    // sleep_ms(0);
    float encoder_angle_raw = servo_controller.get_encoder().read_abs_angle_raw();
    // Average with the previously recorded value
    int idx = size-1-i;
    lut.set_entry(idx, (lut.get_entry(idx) + encoder_angle_raw) / 2.0f);
  }

  // move to starting position
  servo_controller.move_to_open_loop(min_motor_angle, 2.0f);

  LOG_INFO(">finished");
  return true;
}
