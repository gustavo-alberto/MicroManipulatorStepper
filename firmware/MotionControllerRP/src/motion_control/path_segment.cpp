#include "path_segment.h"
#include "utilities/logging.h"
#include "kinemtaic_models/kinematic_model_base.h"

//--- MotionProfileConstAcc -------------------------------------------------------------

MotionProfileConstAcc::MotionProfileConstAcc(float dwell_time) {
  MotionProfileConstAcc::t1 = 0.0f;
  MotionProfileConstAcc::t2 = dwell_time;
  MotionProfileConstAcc::t3 = dwell_time;
  MotionProfileConstAcc::d1 = 0.0f;
  MotionProfileConstAcc::d2 = 1.0f;
  MotionProfileConstAcc::v_peak = 0.0f;
  MotionProfileConstAcc::acceleration = 0.0f;
}

MotionProfileConstAcc::MotionProfileConstAcc(
    float distance,
    float v_start,
    float v_end,
    float max_velocity,
    float max_acceleration)
{
  MotionProfileConstAcc::acceleration = max_acceleration;
  MotionProfileConstAcc::v_start = v_start;
  MotionProfileConstAcc::v_end = v_end;

  if (distance <= 1e-7f) {
    MotionProfileConstAcc::t1 = 0.0f;
    MotionProfileConstAcc::t2 = 0.0f;
    MotionProfileConstAcc::t3 = 0.0f;
    MotionProfileConstAcc::d1 = 0.0f;
    MotionProfileConstAcc::d2 = 1.0f;
    MotionProfileConstAcc::v_peak = 0.0f;
    MotionProfileConstAcc::acceleration = 0.0f;
  } else {
    const float inv_max_acceleration = 1.0f / max_acceleration;

    // Time to accelerate/decelerate, using multiplication by inverse accel
    float t_accel = (max_velocity - v_start) * inv_max_acceleration;
    float t_decel = (max_velocity - v_end) * inv_max_acceleration;

    // Distances covered during accel/decel
    float d_accel = 0.5f * (v_start + max_velocity) * t_accel;
    float d_decel = 0.5f * (max_velocity + v_end) * t_decel;

    float d_cruise = distance - (d_accel + d_decel);

    if (d_cruise >= 0.0f) {
      // Trapezoidal velocity profile
      MotionProfileConstAcc::t1 = t_accel;
      MotionProfileConstAcc::t2 = t1 + d_cruise / max_velocity;
      MotionProfileConstAcc::t3 = t2 + t_decel;
      MotionProfileConstAcc::v_peak = max_velocity;
    } else {
      // Triangular velocity profile: recompute peak velocity v_peak
      float v_peak_sq = max_acceleration * distance + 0.5f * (v_start * v_start + v_end * v_end);
      float v_peak = std::sqrt(std::max(0.0f, v_peak_sq));

      MotionProfileConstAcc::t1 = (v_peak - v_start) * inv_max_acceleration;
      MotionProfileConstAcc::t2 = t1 + 0.0f;
      MotionProfileConstAcc::t3 = t2 + (v_peak - v_end) * inv_max_acceleration;
      MotionProfileConstAcc::v_peak = v_peak;
    }
  }

  // normalize velocity and acceleration to interpolator range (0..1)
  const float inv_distance = 1.0f/distance;
  v_start *= inv_distance;
  v_end *= inv_distance;
  v_peak *= inv_distance;
  acceleration *= inv_distance;

  // precompute some values for faster evaluation
  MotionProfileConstAcc::d1 = 0.5f * (v_start + v_peak) * t1;
  MotionProfileConstAcc::d2 = d1 + v_peak * (t2-t1);

  //LOG_INFO("d1=%f, d2=%f, d3=%f", d1, d2, distance);
  //LOG_INFO("t1=%f, t2=%f, t3=%f", t1, t2, t3);
}

float MotionProfileConstAcc::evaluate(float time) const {
  if (time <= 0.0f) {
    return 0.0f;
  } else if (time < t1) {
    // Acceleration phase
    return v_start * time + 0.5f * acceleration * time * time;
  } else if (time < t2) {
    // Cruise phase
    float dt = time - t1;
    return d1 + v_peak * dt;
  } else if (time < t3) {
    // Deceleration phase
    float dt = time - t2;
    return d2 + v_peak * dt - 0.5f * acceleration * dt * dt;
  } else {
    // Finished
    return 1.0f;
  }
}

//--- CartesianPathSegment --------------------------------------------------------------

CartesianPathSegment::CartesianPathSegment() {
  dwell_time = 0.0f;
}

CartesianPathSegment::CartesianPathSegment(const Pose6DF& start_pose,
                                           const Pose6DF& end_pose,
                                           const LinearAngular& target_velocity,
                                           const LinearAngular& max_acceleration)
{
  CartesianPathSegment::dwell_time = 0.0f;
  CartesianPathSegment::start_pose = start_pose;
  CartesianPathSegment::end_pose = end_pose;

  CartesianPathSegment::target_velocity = target_velocity;
  CartesianPathSegment::start_velocity = LinearAngular(0.0f, 0.0f);
  CartesianPathSegment::end_velocity = LinearAngular(0.0f, 0.0f);
  CartesianPathSegment::max_acceleration = max_acceleration;

  travel_distance.linear = (end_pose.translation - start_pose.translation).length();
  travel_distance.angular = (start_pose.rotation.normalized_inverse() * end_pose.rotation).angle();
}

CartesianPathSegment::CartesianPathSegment(const Pose6DF& pose, float dwell_time)
{
  CartesianPathSegment::dwell_time = dwell_time;
  CartesianPathSegment::start_pose = pose;
  CartesianPathSegment::end_pose = pose;

  CartesianPathSegment::target_velocity = 0.0f;
  CartesianPathSegment::start_velocity = LinearAngular(0.0f, 0.0f);
  CartesianPathSegment::end_velocity = LinearAngular(0.0f, 0.0f);
  CartesianPathSegment::max_acceleration = 0.0f;

  travel_distance.linear = 0.0f;
  travel_distance.angular = 0.0f;
}


void CartesianPathSegment::compute_motion_profile() {
  // LOG_INFO("compute_motion_profile...");
  if(dwell_time > 0.0f) {
    motion_profile = MotionProfileConstAcc(dwell_time);
  } else {
    MotionProfileConstAcc linear_profile(travel_distance.linear,start_velocity.linear, 
                                        end_velocity.linear, target_velocity.linear, 
                                        max_acceleration.linear);

    MotionProfileConstAcc angular_profile(travel_distance.angular, start_velocity.angular, 
                                          end_velocity.angular, target_velocity.angular, 
                                          max_acceleration.angular);

    // select profile that requires the longest time
    if(linear_profile.t3 > angular_profile.t3) {
      motion_profile = linear_profile;
    } else {
      motion_profile = angular_profile;
    }
  }
}

void CartesianPathSegment::evaluate(float time, Pose6DF& pose) const {
  // evaluate motion profile
  float t = motion_profile.evaluate(time);

  // interpolate pose
  pose = Pose6DF::lerp(start_pose, end_pose, t);
}

float CartesianPathSegment::get_duration() const {
  return motion_profile.t3;
}

//--- JointSpacePathSegment -------------------------------------------------------------

JointSpacePathSegment::JointSpacePathSegment() {
  for(int i=0; i<NUM_JOINTS; i++) {
    JointSpacePathSegment::start_pos[i] = 0.0f;
    JointSpacePathSegment::end_pos[i] = 0.0f;
  }
  duration = 0.0f;
  inv_duration = 0.0f;
  initialized = false;
}

JointSpacePathSegment::JointSpacePathSegment(
  const float start_pos[NUM_JOINTS],
  const float end_pos[NUM_JOINTS],
  float duration)
{
  JointSpacePathSegment::duration = duration;
  JointSpacePathSegment::inv_duration = 1.0f/std::max(duration, 1e-7f);

  for(int i=0; i<NUM_JOINTS; i++) {
    JointSpacePathSegment::start_pos[i] = start_pos[i];
    JointSpacePathSegment::end_pos[i] = end_pos[i];
  }

  initialized = true;
}

void JointSpacePathSegment::evaluate(
  float time, 
  float joint_positions[NUM_JOINTS], 
  float joint_velocity[NUM_JOINTS]) const
{
  float t = time*inv_duration;
  float s = 1.0f-t;

  for(int i=0; i<NUM_JOINTS; i++) {
    joint_positions[i] = start_pos[i]*s + end_pos[i]*t;
    joint_velocity[i] = 0;
  }
}

float JointSpacePathSegment::get_duration() {
  return duration;
}

bool JointSpacePathSegment::is_initialized() {
  return initialized;
}

//--- JointSpacePathSegmentGenerator ----------------------------------------------------

JointSpacePathSegmentGenerator::JointSpacePathSegmentGenerator(
  const CartesianPathSegment* path_segment,
  IKinemtaicModel* kinematic_model,
  float time_step)
{
  JointSpacePathSegmentGenerator::path_segment = path_segment;
  JointSpacePathSegmentGenerator::kinematic_model = kinematic_model;
  current_time = 0.0f;
  delta_time = time_step;
  end_time = path_segment->get_duration();
  end_time_with_eps = end_time-0.2f*delta_time;

  // check kinematic model
  if(kinematic_model->get_joint_count() != NUM_JOINTS) {
    LOG_ERROR("NUM_JOINTS (%i) differs from value required by Kinematic model (%i)",
      NUM_JOINTS, kinematic_model->get_joint_count());
    error_trap("Fatal Error");
  }

  // evaluate inverse kinematic model to et start joint positions
  kinematic_model->inverse(path_segment->start_pose, current_joint_pos);
}

bool JointSpacePathSegmentGenerator::generate_next(JointSpacePathSegment& js_path_segment) {
  bool end_reached = false;

  // increment evaluation position
  float initial_time = current_time;
  current_time += delta_time;

  // check if end of path is reached, check against end_t which includes an epsilon
  // to prevent tiny segments at path end (snaps to t=1.0 within tolerance).
  if(current_time >= end_time_with_eps) {
    end_reached = true;
    current_time = end_time;  // snap to 1.0
  }

  // evaluate path to get new end position
  Pose6DF seg_end_pose;
  path_segment->evaluate(current_time, seg_end_pose);

  // evaluate inverse kinematic model here
  float next_joint_pos[NUM_JOINTS];
  kinematic_model->inverse(seg_end_pose, next_joint_pos);
  
  // create joint space path segment
  float duration = current_time-initial_time;
  js_path_segment = JointSpacePathSegment(current_joint_pos, next_joint_pos, duration);

  // update current joint pos
  for(int i=0; i<NUM_JOINTS; i++)
    current_joint_pos[i] = next_joint_pos[i];

  return end_reached;
}