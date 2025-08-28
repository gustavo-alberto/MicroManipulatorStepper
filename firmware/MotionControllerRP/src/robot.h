#pragma once

#include "utilities/logging.h"
#include "utilities/frequency_counter.h"
#include "hardware/MT6701_encoder.h"
#include "hardware/MT6835_encoder.h"
#include "hardware/TB6612_motor_driver.h"
#include "servo_control/servo_controller.h"
#include "servo_control/encoder_lut.h"
#include "utilities/math_constants.h"

#include "motion_control/path_planner.h"
#include "motion_control/motion_controller.h"
#include "command_parser/command_parser.h"

//*** CALSS *****************************************************************************

class Robot;

//--- SharedData ------------------------------------------------------------------------

enum class ERobotState {
  IDLE = 0,
  BUFFERING_PATH = 1,
  EXECUTING_PATH = 2,
  ERROR = 3
};

//--- SharedData ------------------------------------------------------------------------

struct SharedData {
  SharedData(int hw_spinlock_id=0){
    lock = spin_lock_instance(hw_spinlock_id);
  };

  volatile float joint_positions[NUM_JOINTS];
  volatile float joint_velocities[NUM_JOINTS];
  spin_lock_t* lock = nullptr;
};

//--- RobotJoint ------------------------------------------------------------------------

class RobotJoint {
  public:
    RobotJoint(MT6835Encoder* encoder, TB6612MotorDriver* motor_driver, int pole_pairs);
    ~RobotJoint();

    void init();
    void home();
    void calibrate();
    void update(float dt, float one_over_dt);

    void update_target(float p, float v);

  public:
    float position;
    float velocity;

    MT6835Encoder* encoder;
    TB6612MotorDriver* motor_driver;
    ServoController* servo_controller;
};

//--- Robot -----------------------------------------------------------------------------

class Robot : public ICommandProcessor {
  public:
    Robot(float path_segment_time_step);
    ~Robot();

    void init();
    void calibrate();
    void home();

    void update_command_parser();            // called from main loop
    void update_path_planner();              // called from main loop
    void update_servo_controllers(float dt); // called from seperate cpu-core

  public:
    void send_reply(const char* str) override;
    bool can_process_command(const GCodeCommand& cmd) override;
    void process_command(const GCodeCommand& cmd, std::string& reply) override;

    void process_motion_command(const GCodeCommand& cmd, std::string& reply);
    void process_dwell_command(const GCodeCommand& cmd, std::string& reply);
    void process_machine_command(const GCodeCommand& cmd, std::string& reply);
    void process_set_pose_command(const GCodeCommand& cmd, std::string& reply);
    void process_set_servo_parameter_command(const GCodeCommand& cmd, std::string& reply);

  protected:
    static bool update_motion_controller_isr(repeating_timer_t* timer); // called from update timer

  private:
    ERobotState state;
    uint32_t path_buffering_time_us;
    uint64_t path_buffering_start_time;

    RobotJoint* volatile joints[NUM_JOINTS];

    IKinemtaicModel* kinematic_model;
    PathPlanner path_planner;
    MotionController motion_controller;
    CommandParser command_parser;

    LinearAngular max_acceleration;
    Pose6DF current_pose;
    LinearAngular current_feedrate;

    SharedData shared_data;

    struct repeating_timer motion_controller_update_timer;
    uint64_t last_mc_update_time;

    FrequencyCounter servo_loop_frequency_counter;
    FrequencyCounter motion_controller_frequency_counter;
};