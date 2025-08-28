#include "robot.h"
#include "hw_config.h"
#include "utilities/logging.h"
#include "kinemtaic_models/kinematic_model_delta3d.h"

//*** FUNCTION **************************************************************************

bool startswith(const std::string& str, const std::string& prefix) {
    return str.size() >= prefix.size() &&
           std::equal(prefix.begin(), prefix.end(), str.begin());
}

//*** CLASS *****************************************************************************

//--- RobotAxis -------------------------------------------------------------------------

RobotJoint::RobotJoint(MT6835Encoder* encoder, 
                     TB6612MotorDriver* motor_driver,
                     int pole_pairs) 
{
  RobotJoint::encoder = encoder;
  RobotJoint::motor_driver = motor_driver;
  servo_controller = new ServoController(*motor_driver, *encoder, pole_pairs);
  position = 0.0f;
  velocity = 0.0f;
}

RobotJoint::~RobotJoint() {
  delete servo_controller;
  delete motor_driver;
  delete encoder;
  servo_controller = nullptr;
  motor_driver = nullptr;
  encoder = nullptr;
}

void RobotJoint::init() {
  encoder->init(0x5, 0x4);
  servo_controller->init(0.5);
}

void RobotJoint::home() {
  servo_controller->home(-1.0f, 100.0f*DEG_TO_RAD, 0.1f);
  position = servo_controller->get_position();
  velocity = 0.0f;
}

void RobotJoint::calibrate() {
  LookupTable lut;
  build_motor_to_enc_angle_lut(lut, *servo_controller, 1.0f*DEG_TO_RAD, 92.0f*DEG_TO_RAD, 256);
  // lut.print_to_log();
  LOG_DEBUG("Inverting lookup table...");
  bool ok = lut.invert(256);
  if(!ok) {
    servo_controller->get_motor_driver().disable();
    lut.print_to_log();
    while(true);
  }
  LOG_DEBUG(">finished");
  // lut.print_to_log();
  delay(200);

  servo_controller->set_encoder_lut(lut);
}

void RobotJoint::update(float dt, float one_over_dt) {
  servo_controller->update(position, dt, one_over_dt);
}

void RobotJoint::update_target(float p, float v) {
  position = p;
  velocity = v;
}

//--- Robot -----------------------------------------------------------------------------

Robot::Robot(float path_segment_time_step) : 
  path_planner(nullptr, path_segment_time_step), 
  motion_controller(&path_planner),
  servo_loop_frequency_counter(10000),
  motion_controller_frequency_counter(1000)
{
  kinematic_model = new KinematicModel_Delta3D();
  path_planner.set_kinematic_model(kinematic_model);

  for(int i=0; i<3; i++)
    joints[i] = nullptr;

  command_parser.set_command_processor(this);

  current_feedrate = LinearAngular(10.0f, 1.0f);
  max_acceleration = LinearAngular(500.0f, 50.0f);
  path_buffering_time_us = 50*1e3;

  state = ERobotState::IDLE;
}

Robot::~Robot() {
  if(kinematic_model != nullptr)
    delete kinematic_model;

  for(int i=0; i<3; i++) {
    if(joints[i] != nullptr)
      delete joints[i];
    joints[i] = nullptr;
  }
}

void Robot::init() {
  MT6835Encoder::setup_spi(spi0, PIN_ENCODER_SCK, PIN_ENCODER_MOSI, PIN_ENCODER_MISO, 8000000);

  // axis 1
  {
    auto* encoder = new MT6835Encoder(spi0, PIN_ENCODER1_CS);
    auto* motor_driver = new TB6612MotorDriver(
      PIN_MOTOR_EN, PIN_M1_PWM_A_POS, PIN_M1_PWM_A_NEG, PIN_MOTOR_PWMAB,
      PIN_MOTOR_EN, PIN_M1_PWM_B_POS, PIN_M1_PWM_B_NEG, PIN_MOTOR_PWMAB
    );
    joints[0] = new RobotJoint(encoder, motor_driver, 400/4);
  }

  // axis 2
  {
    auto* encoder = new MT6835Encoder(spi0, PIN_ENCODER2_CS);
    auto* motor_driver = new TB6612MotorDriver(
      PIN_MOTOR_EN, PIN_M2_PWM_A_POS, PIN_M2_PWM_A_NEG, PIN_MOTOR_PWMAB,
      PIN_MOTOR_EN, PIN_M2_PWM_B_POS, PIN_M2_PWM_B_NEG, PIN_MOTOR_PWMAB
    );
    joints[1] = new RobotJoint(encoder, motor_driver, 400/4);
  }

  // axis 3
  {
    auto* encoder = new MT6835Encoder(spi0, PIN_ENCODER3_CS);
    auto* motor_driver = new TB6612MotorDriver(
      PIN_MOTOR_EN, PIN_M3_PWM_A_POS, PIN_M3_PWM_A_NEG, PIN_MOTOR_PWMAB,
      PIN_MOTOR_EN, PIN_M3_PWM_B_POS, PIN_M3_PWM_B_NEG, PIN_MOTOR_PWMAB
    );
    joints[2] = new RobotJoint(encoder, motor_driver, 400/4);
  }

  // initialize axes
  for(int i=0; i<3; i++) {
    joints[i]->init();
  }

  // setup timer for updating the motion controller (which evaluates joint space path
  // segments and produces the current target position for the servo loops)
  float motion_controller_update_time_us = 500;
  add_repeating_timer_us(-motion_controller_update_time_us, 
                         Robot::update_motion_controller_isr, 
                         (void*)this, 
                         &motion_controller_update_timer);
}

void Robot::calibrate() {
  for(int i=0; i<3; i++) {
    joints[i]->calibrate();
  }
}

void Robot::home() {
  for(int i=0; i<3; i++) {
    joints[i]->home();
    joints[i]->calibrate();

    // set start angle
    float start_angle = 20*Constants::DEG2RAD;
    joints[i]->servo_controller->move_to_open_loop(start_angle, 1.0f);
    if (spin_try_lock_unsafe(shared_data.lock)) {
      shared_data.joint_positions[i] = start_angle;
      spin_unlock_unsafe(shared_data.lock);
    }
  }
}

void Robot::update_command_parser() {
  // process serial input
  if (Serial.available()) {
    char c = Serial.read();
    command_parser.add_input_character(c);
    // Serial.write(c);
  }

  // update command parse which will queue command to the path planner
  command_parser.update();

  // TESTING:
  //sleep_ms(10);
  //float pos_error = joints[1]->servo_controller->get_position_error();
  //LOG_INFO(">pos_error [µrad]: %f\n", pos_error*1e6);
  //Serial.printf(">pos_error [µrad]: %f\n", pos_error*1e6);#
  //LOG_INFO(">pos_x [mm]: %f", joints[1]->position);
}

/**
 * Updates the path planner, that chops up kartesian path segments into joint space
 * path segments using the inverse kinematic model. It then enqueues these joint space path
 * segments for the motion controller.
 */
void Robot::update_path_planner() {
  // check if buffering starts
  uint64_t time = time_us_64();
  if(state == ERobotState::IDLE && path_planner.input_queue_size() > 0) {
    state = ERobotState::BUFFERING_PATH;
    path_buffering_start_time = time;
  }

  // check if execution starts
  uint64_t buffering_time = time-path_buffering_start_time;
  if(state == ERobotState::BUFFERING_PATH && buffering_time > path_buffering_time_us) {
    state = ERobotState::EXECUTING_PATH;
  }

  // execute path
  if(state == ERobotState::EXECUTING_PATH) {
    // update planner and generate joint space path segments
    path_planner.process(true);

    if(path_planner.all_finished())
      state = ERobotState::IDLE;
  }
}

/**
 * Updates the motion controller with a timer interrupt in regular intervals.
 * The function evaluates joint space path segments and produces the current 
 * target position for the servo loops.
 */
bool Robot::update_motion_controller_isr(repeating_timer_t* timer) {
  float joint_positions[NUM_JOINTS];
  float joint_velocities[NUM_JOINTS];

  // get robot pointer
  Robot* robot = (Robot*)timer->user_data;

  // get time and delta time
  uint64_t time_us = time_us_64();
  float dt = float(time_us - robot->last_mc_update_time)*1e-6f;
  robot->last_mc_update_time = time_us;

  // get current joint position/velocity
  bool update_ok = robot->motion_controller.update(dt, joint_positions, joint_velocities);

  // Attempt to acquire spinlock non-blocking and set new target data for the servo loops
  if (update_ok && spin_try_lock_unsafe(robot->shared_data.lock)) {
    for (int i = 0; i < NUM_JOINTS; i++) {
      robot->shared_data.joint_positions[i] = joint_positions[i];
      robot->shared_data.joint_velocities[i] = joint_velocities[i];
    }
    spin_unlock_unsafe(robot->shared_data.lock);
  }

  // update frequency counter
  robot->motion_controller_frequency_counter.update(dt);

  return true; // keep repeating
}

/**
 * update servo loops, this is called from a second cpu core
 */
void Robot::update_servo_controllers(float dt) {
  float one_over_dt = 1.0f/dt;

  // update axis target position and velocity from shared data
  spin_lock_unsafe_blocking(shared_data.lock);
  for(int i=0; i<3; i++)
    joints[i]->update_target(shared_data.joint_positions[i], shared_data.joint_velocities[i]);
  spin_unlock_unsafe(shared_data.lock);

  // update servo loop for each axis
  for(int i=0; i<3; i++) {
    joints[i]->update(dt, one_over_dt);
  }

  // update frequency counter
  servo_loop_frequency_counter.update(dt);
}

bool Robot::can_process_command(const GCodeCommand& cmd) {
  if(cmd.get_command() == "G0" || 
     cmd.get_command() == "G4")
  {
    return path_planner.input_queue_full() == false;
  }

  return true;
}

void Robot::send_reply(const char* str) {
  Serial.write(str);
}

void Robot::process_command(const GCodeCommand& cmd, std::string& reply) {
  if(cmd.get_command() == "G0") process_motion_command(cmd, reply);
  else if(cmd.get_command() == "G1") process_motion_command(cmd, reply);
  else if(cmd.get_command() == "G4") process_dwell_command(cmd, reply);
  else if(startswith(cmd.get_command(), "M")) process_machine_command(cmd, reply);
  else reply="error: unknown command\n";
}

void Robot::process_motion_command(const GCodeCommand& cmd, std::string& reply) {
  Pose6DF end_pose;

  if(path_planner.input_queue_full()) {
    reply = "error: input queue full\n";
    return;
  }

  // read feed rate
  current_feedrate.linear = cmd.get_value('F', current_feedrate.linear);
  current_feedrate.angular = cmd.get_value('R', current_feedrate.angular);

  if(cmd.has_word('I'))
    state = ERobotState::EXECUTING_PATH;

  // read translation
  end_pose.translation.x = cmd.get_value('X', current_pose.translation.x);
  end_pose.translation.y = cmd.get_value('Y', current_pose.translation.y);
  end_pose.translation.z = cmd.get_value('Z', current_pose.translation.z);

  // read rotation (all elements must be present)
  if(cmd.has_word('A') && cmd.has_word('B') && cmd.has_word('C')) {
    Vec3F rot_vec(cmd.get_value('A'), cmd.get_value('B'), cmd.get_value('C'));
    end_pose.rotation = QuaternionF::from_rot_vec(rot_vec);
  } else {
    end_pose.rotation = current_pose.rotation;
  }

  // create path segment
  CartesianPathSegment path_segment(current_pose, end_pose, 
                                    current_feedrate, 
                                    max_acceleration);

  bool ok = path_planner.add_cartesian_path_segment(path_segment);
  if(ok) {
    path_planner.run_look_ahead_planning();
    current_pose = end_pose;
    reply = "ok\n";
  } else {
    reply = "error\n";
  }
}

void Robot::process_machine_command(const GCodeCommand& cmd, std::string& reply) {
  reply = "";
  
  if(cmd.get_command() == "M50") {
    reply += "Current Position: ";
    reply += std::string(" X") + std::to_string(current_pose.translation.x);
    reply += std::string(" Y") + std::to_string(current_pose.translation.y);
    reply += std::string(" Z") + std::to_string(current_pose.translation.z);
    reply += "\n";
    reply = "ok\n";
  }
  if(cmd.get_command() == "M51") {
    uint32_t servo_loop_freq = servo_loop_frequency_counter.get();
    uint32_t mcontroler_freq = motion_controller_frequency_counter.get();
    reply += std::string("Servo Loop: ") + std::to_string(servo_loop_freq/1000) + "kHz\n";
    reply += std::string("Motion Controler: ") + std::to_string(mcontroler_freq/1000) + "kHz\n";
    reply += "ok\n";
  }
  if(cmd.get_command() == "M52") {
    int s = path_planner.input_queue_size();
    reply += std::string("Queue Size: ") + std::to_string(s) + "\n";
    reply += "ok\n";
  }
  if(cmd.get_command() == "M53") {
    bool f = path_planner.all_finished();
    reply += f ? "1\n" : "0\n";
    reply += "ok\n";
  }
  if(cmd.get_command() == "M55") {
    process_set_servo_parameter_command(cmd, reply);
  }
  if(cmd.get_command() == "M204") {
    if(cmd.has_word('L')) max_acceleration.linear = cmd.get_value('L');
    if(cmd.has_word('A')) max_acceleration.angular = cmd.get_value('A');
    reply += "ok\n";
  }
}

void Robot::process_dwell_command(const GCodeCommand& cmd, std::string& reply) {
  // get dwell time
  float dwell_time = 1.0f;
  if(cmd.has_word('S')) dwell_time = cmd.get_value('S');          // time given in seconds
  if(cmd.has_word('P')) dwell_time = cmd.get_value('P')*0.001f;   // time given in milliseconds

  // create path segment
  CartesianPathSegment path_segment(current_pose, dwell_time);
  path_planner.add_cartesian_path_segment(path_segment);
  path_planner.run_look_ahead_planning();

  reply = "ok\n";
}

void Robot::process_set_servo_parameter_command(const GCodeCommand& cmd, std::string& reply) {
  // example: M55 A150 B50000 C0.2 D100 E F0.0025
  bool has_all = cmd.has_word('A') && cmd.has_word('B') && cmd.has_word('C') && 
                 cmd.has_word('D') && cmd.has_word('F');

  if(has_all == false)
    reply = "error: not all parameters given (A,B,C,D,F expected)\n";

  for(int i=0; i<NUM_JOINTS; i++) {

    joints[i]->servo_controller->velocity_lowpass.set_time_constant(cmd.get_value('F'));
    joints[i]->servo_controller->pos_controller.set_parameter(cmd.get_value('A'), cmd.get_value('B'), 0.0f, Constants::PI_F*2.0F, Constants::PI_F*0.5F);
    joints[i]->servo_controller->velocity_controller.set_parameter(cmd.get_value('C'), cmd.get_value('D'), 0.0f, Constants::PI_F*0.45f, Constants::PI_F*0.45f);
  }

  reply = "ok\n";
}

