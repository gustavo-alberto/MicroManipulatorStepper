// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#include "main.h"

#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/vreg.h"

#include <NeoPixelConnect.h>
#include <Wire.h>
#include <algorithm>

#include "robot.h"
#include "utilities/logging.h"
#include "utilities/frequency_counter.h"
#include "kinemtaic_models/kinematic_model_delta3d.h"

#include "hw_config.h"

//*** GLOBALS ***************************************************************************

NeoPixelConnect strip(PIN_BUILTIN_LED, 1);
Robot robot(0.01f);

/*
MT6835Encoder encoder1(spi0, PIN_ENCODER1_CS);
MT6835Encoder encoder2(spi0, PIN_ENCODER2_CS);
MT6835Encoder encoder3(spi0, PIN_ENCODER3_CS);

TB6612MotorDriver motor_driver1(
  PIN_MOTOR_EN, PIN_M1_PWM_A_POS, PIN_M1_PWM_A_NEG, PIN_MOTOR_PWMAB,
  PIN_MOTOR_EN, PIN_M1_PWM_B_POS, PIN_M1_PWM_B_NEG, PIN_MOTOR_PWMAB
);
TB6612MotorDriver motor_driver2(
  PIN_MOTOR_EN, PIN_M2_PWM_A_POS, PIN_M2_PWM_A_NEG, PIN_MOTOR_PWMAB,
  PIN_MOTOR_EN, PIN_M2_PWM_B_POS, PIN_M2_PWM_B_NEG, PIN_MOTOR_PWMAB
);
TB6612MotorDriver motor_driver3(
  PIN_MOTOR_EN, PIN_M3_PWM_A_POS, PIN_M3_PWM_A_NEG, PIN_MOTOR_PWMAB,
  PIN_MOTOR_EN, PIN_M3_PWM_B_POS, PIN_M3_PWM_B_NEG, PIN_MOTOR_PWMAB
);

ServoController servo_controller1(motor_driver1, encoder1, 400/4);
ServoController servo_controller2(motor_driver2, encoder2, 400/4);
ServoController servo_controller3(motor_driver3, encoder3, 400/4);
FrequencyCounter loop_freq_counter(1000);

PathPlanner planner(0.01f);
MotionController motion_controller(&planner);
Pose6DF current_pose;

CommandParser command_parser; */

//*** FUNCTIONS *************************************************************************

// Run before setup()
//__attribute__((constructor))
void overclock() {
  vreg_set_voltage(VREG_VOLTAGE_1_20);         // For >133 MHz
  busy_wait_us(10 * 1000);  // 10 ms delay
  set_sys_clock_khz(250000, true);             // Set to 250 MHz
}

void set_led_color(uint8_t r, uint8_t g, uint8_t b) {
  strip.neoPixelSetValue(0, r, g, b, false);
  delayMicroseconds(1000);
  strip.neoPixelShow();
}

void led_blink(uint8_t r, uint8_t g, uint8_t b, int count, int period_time_ms) {
  for(int i=0; i<count; i++) {
    set_led_color(r, g, b);
    sleep_ms(period_time_ms/2);
    set_led_color(0, 0, 0);
    sleep_ms(period_time_ms/2);
  }
}

void main_core0() {
  uint64_t last_time = time_us_64();

  while(true) {
    // update motion controller
    robot.update_command_parser();
    robot.update_path_planner();
  }
}

void main_core1() {
  LOG_INFO("starting servo controll loops on core 1...");

  uint64_t last_time = time_us_64();
  while(true) {
    // get time and detla time
    uint64_t time_us = time_us_64();
    float dt = float(time_us - last_time)*1e-6f;
    last_time = time_us;

    robot.update_servo_controllers(dt);
  }
}

void setup() {
  led_blink(0, 20, 0, 1, 4000/3);
  // stdio_init_all();  // Initializes USB or UART stdio
  overclock();
  // Serial.begin(921600);
  Logger::instance().begin(921600, false);
  while(!Serial);

  set_led_color(50, 10, 0);
  // auto* test = new KinematicModel_Delta3D(); test->test(); delete test;

  delay(100);  // Allow time for serial monitor to connect
  Serial.printf("System clock: %i Mhz\n", int32_t(clock_get_hz(clk_sys))/1000/1000);

  LOG_INFO("initializing robot...");
  robot.init();

  LOG_INFO("homing axes...");
  robot.home();

  multicore_launch_core1(&main_core1);
  
  set_led_color(0, 10, 0);
  LOG_INFO("initialization finished...");

  return;

  /*
  rotencoder_wire.setSDA(PIN_ENCODER_SDA);
  rotencoder_wire.setSCL(PIN_ENCODER_SCL);
  rotencoder_wire.begin();
  rotencoder_wire.setClock(1000000);
  encoder.init();
  encoder.set_hysteresis(0x4); //0x6);
  */
 /* pinMode(PIN_USER_BUTTON, INPUT_PULLUP);

  // init encoders
  MT6835Encoder::setup_spi(spi0, PIN_ENCODER_SCK, PIN_ENCODER_MOSI, PIN_ENCODER_MISO, 8000000);
  encoder1.init(0x5, 0x4);
  encoder2.init(0x5, 0x4);
  encoder3.init(0x5, 0x4);

  servo_controller1.init(0.5);
  servo_controller2.init(0.5);
  servo_controller3.init(0.5);

  delay(1000); 

  set_led_color(50, 10, 0);

  servo_controller1.home(-1.0f, 100.0f*DEG_TO_RAD, 0.1f);
  calibrate_actuator(servo_controller1);

  servo_controller2.home(-1.0f, 100.0f*DEG_TO_RAD, 0.1f);
  calibrate_actuator(servo_controller2);

  servo_controller3.home(-1.0f, 100.0f*DEG_TO_RAD, 0.1f);
  calibrate_actuator(servo_controller3);

  set_led_color(0, 10, 0);
  */
}

void loop() {
  main_core0();
}

/*
int it=0;
float target_angle1 = 50.0f/180.0f*PI;
float target_angle2 = 50.0f/180.0f*PI;
float target_angle3 = 50.0f/180.0f*PI;
int k=0;
uint64_t last_time = time_us_64();
uint64_t last_print_time = time_us_64();


void loop_encoder_test() {
  float encoder_angle = encoder3.read_abs_angle();
  Serial.printf(">angle: %f\n", encoder_angle*360/TWO_PI);
  delay(10);
}

void loop_motor_test() {
  uint64_t time_us = time_us_64();
  //target_angle = (30.0f+0.07f*((time_us>>19)%2))*DEG_TO_RAD;
  float target_angle = (1000.05f*sin(float(time_us)*3e-5f))/180*PI;
  motor_driver1.set_field_angle(target_angle);

  target_angle = (1000.05f*sin(float(time_us)*4e-5f))/180*PI;
  motor_driver2.set_field_angle(target_angle);

  target_angle = (1000.05f*sin(float(time_us)*5e-5f))/180*PI;
  motor_driver3.set_field_angle(target_angle);
}

void loop_old() {
 // return;
 //loop_motor_test(); return;
 // loop_encoder_test(); return;

  // get time and detla time
  uint64_t time_us = time_us_64();
  float dt = float(time_us - last_time)*1e-6f;
  float one_over_dt = 1.0f/dt;
  last_time = time_us;

  servo_controller1.update(shared_data.joint_positions[0], dt, one_over_dt);
  servo_controller2.update(shared_data.joint_positions[1], dt, one_over_dt);
  servo_controller3.update(shared_data.joint_positions[2], dt, one_over_dt);
  loop_freq_counter.update(dt);

  // motor_servo_update(dt_ms);

  
  // print info
  if(time_us-last_print_time > 10000 && true) {
    //Serial.printf(">angle: %f\n", encoder_angle*360/TWO_PI);
    //Serial.printf(">field: %f\n", field_angle*360/TWO_PI);
    Serial.printf(">pos_error [µrad]: %f\n", servo_controller3.get_position_error()*1e6f);
    Serial.printf(">output [deg]: %f\n", servo_controller3.output*float(RAD_TO_DEG));
    Serial.printf(">motor_pos [deg]: %f\n", servo_controller3.get_position()*float(RAD_TO_DEG));
    //Serial.printf(">motor_pos: %f µm\n", servo_controller.get_position()*15.0e6f);
    //Serial.printf(">update_khz: %f\n", float(loop_freq_counter.get())*0.001);
    
    //Serial.printf(">e: %f\n",e*360/TWO_PI);

    last_print_time = time_us;
  }

  //target_angle2 = (50.0f+0.012f*((time_us>>20)%2))*DEG_TO_RAD;
  //target_angle3 = (50.0f+0.012f*(1-(time_us>>20)%2))*DEG_TO_RAD;

  //target_angle2 = (50.0f+0.0002f*sin(float(time_us)*5e-6f))/180*PI;
  //target_angle3 = (50.0f+0.0002f*cos(float(time_us)*5e-6f))/180*PI;
  
  // target_angle1 = (50.0f+20.001f*trapezoidal_wave(float(time_us)*5e-6f-PI*0.33f))/180*PI;
  // target_angle2 = (50.0f+20.001f*trapezoidal_wave(float(time_us)*5e-6f))/180*PI;
  // target_angle3 = (50.0f+20.001f*trapezoidal_wave(float(time_us)*5e-6f+PI*0.33f))/180*PI;
  
  //target_angle = (50.0f+20.005f*triangle_wave(float(time_us)*1.0e-5f))/180*PI;
  

  return;
/*
  it++;

  float ki = 0.0f;
  if(k<5000 || digitalRead(PIN_USER_BUTTON) == 0)
    ki = 0.3f;
  k++;

  // test
  float max_integral = 0.15f;
  float e = (target_angle - encoder_angle);
  //float gain_scale = e<0.25f/360*TWO_PI ? 3.0 : 1.0f;
  //float d = (e-prev_e) * 10.0f;
  //d_filtered = d_filtered*0.5f + d*0.5f;

  float velocity_setpoint = 0.2f * e;
  float velocity = encoder_angle-prev_angle;
  float velocity_error = velocity_setpoint - velocity;

  integral += std::clamp(1.25f * velocity_error, -max_integral, max_integral);  // velocity PI

 // float ig = max(min(e*ki, max_integral), -max_integral);
 // integral += ig - v*0.4f;

  out = integral;// + std::clamp(e*0.0f, -3.415926f*0.05f, 3.415926f*0.05f);// - d_filtered;
  //float p = max(min(e*3, 1), -1);
  motor.set_field_angle(target_angle*e2m_scale+out);
  //motor.set_field_angle(out);
  prev_e = e;
  prev_angle = encoder_angle;

  if(it>100) {
    // Serial.print(rawAngle);
    Serial.print(">angle: ");
  //  float angleDegrees = angle_tracker. * 360.0f / 16384.0f;
    Serial.println(encoder_angle*360/TWO_PI, 9);
    Serial.print(">out: ");
    Serial.println(out, 9);

    Serial.printf(">e: %f\n",e*360/TWO_PI);
   // Serial.printf(">v: %f\n",velocity);

    if(fabs(target_angle-encoder_angle)*360/TWO_PI > 0.05)
        strip.neoPixelSetValue(0, 10, 0, 0, false);
    else
        strip.neoPixelSetValue(0, 0, 10, 0, false);

    delayMicroseconds(10);
    strip.neoPixelShow();
  }

 // target_angle = (60.0f+20.00f*((k/1000)%2))/180*PI;
  //  target_angle = (60.0f+0.1f*sin(float(k)*0.01))/180*PI;
 // k++;

 // int32_t output = pos_controller.compute(pos_controller.to_fixpoint(target_angle), 
 //                                         pos_controller.to_fixpoint(angle));

 // a = pos_controller.from_fixpoint(output);
  //a += 0.001f;
 // motor.set_field_angle(a);

  // float ma = (angle)*e2m_scale + e2m_offset;
  // motor.set_field_angle(ma);

/*  static uint16_t hue = 0; // 0-255 for full RGB cycle
  uint8_t r, g, b;
  //hsv2rgb(hue, 255, 1, r, g, b); // 50 = brightness (0-255)
  r = 0; g=0; b=0;
  g = raw_angle%2 == 0 ? 10 : 0;

  // Set the single pixel to the current color
  strip.neoPixelSetValue(0, r, g, b, false);
  strip.neoPixelShow();

  hue = (hue + 1) % 256; // Adjust increment for speed
  delay(20);             // Adjust delay for smoothness
  */
 //  strip.neoPixelSetValue(0, 2, 190, 3, false);
// }