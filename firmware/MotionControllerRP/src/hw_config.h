// motor pole pair count
//  * 100 for 0.9deg stepper motors
//  * 50  for 1.8deg stepper motors
#define MOTOR1_POLE_PAIRS 100
#define MOTOR2_POLE_PAIRS 100
#define MOTOR3_POLE_PAIRS 100

#define CALIBRATION_RANGE 95        // degrees from home position
#define CALIBRATION_FIELD_VELOCITY 40.0f

#define HOMING_VELOCITY   1.0f        // rad per s
#define HOMING_CURRENT    0.15f       // range 0..1
#define HOMING_FINISH_POS 0.5f        // in rad


#define SINGLE_AXIS_BOARD
#ifndef SINGLE_AXIS_BOARD
  // Pins for 3Axis Board
  #define PIN_BUILTIN_LED 23
  #define PIN_USER_BUTTON 24

  #define PIN_M1_PWM_A_POS  13
  #define PIN_M1_PWM_A_NEG  12
  #define PIN_M1_PWM_B_POS  14
  #define PIN_M1_PWM_B_NEG  15

  #define PIN_M2_PWM_A_POS  9
  #define PIN_M2_PWM_A_NEG  8
  #define PIN_M2_PWM_B_POS  10
  #define PIN_M2_PWM_B_NEG  11

  #define PIN_M3_PWM_A_POS  5
  #define PIN_M3_PWM_A_NEG  4
  #define PIN_M3_PWM_B_POS  6
  #define PIN_M3_PWM_B_NEG  7


  #define PIN_MOTOR_EN      18
  #define PIN_MOTOR_PWMAB   19

  #define PIN_ENCODER1_CS 20
  #define PIN_ENCODER2_CS 21
  #define PIN_ENCODER3_CS 22
  #define PIN_ENCODER_SCK 2
  #define PIN_ENCODER_MISO 0
  #define PIN_ENCODER_MOSI 3

#else
  // Single Axis Board
  #define PIN_BUILTIN_LED 16
  #define PIN_USER_BUTTON 24

  #define PIN_M1_PWM_A_POS  13
  #define PIN_M1_PWM_A_NEG  12
  #define PIN_M1_PWM_B_POS  14
  #define PIN_M1_PWM_B_NEG  15

  #define PIN_M2_PWM_A_POS  9
  #define PIN_M2_PWM_A_NEG  8
  #define PIN_M2_PWM_B_POS  10
  #define PIN_M2_PWM_B_NEG  11

  #define PIN_M3_PWM_A_POS  5
  #define PIN_M3_PWM_A_NEG  4
  #define PIN_M3_PWM_B_POS  6
  #define PIN_M3_PWM_B_NEG  7


  #define PIN_MOTOR_EN      18
  #define PIN_MOTOR_PWMAB   19

  #define PIN_ENCODER1_CS 20
  #define PIN_ENCODER2_CS 21
  #define PIN_ENCODER3_CS 22
  #define PIN_ENCODER_SCK 2
  #define PIN_ENCODER_MISO 0
  #define PIN_ENCODER_MOSI 3
#endif

// test setup
/*
#define PIN_PWM_A_POS  2
#define PIN_PWM_A_NEG  3
#define PIN_PWM_B_POS  1
#define PIN_PWM_B_NEG  0
#define PIN_PWM_A_EN   5
#define PIN_PWM_B_EN   5
#define PIN_PWMAB      4

// I2C Encoder
//#define PIN_ENCODER_SDA 28
//#define PIN_ENCODER_SCL 29

#define PIN_ENCODER_CS 17
#define PIN_ENCODER_SCK 18
#define PIN_ENCODER_MISO 16
#define PIN_ENCODER_MOSI 19
*/
