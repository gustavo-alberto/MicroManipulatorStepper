
#include "hardware/sync.h"

#include "path_planner.h"
#include "utilities/logging.h"

PathPlanner::PathPlanner(IKinemtaicModel* kinematic_model, float time_step) {
  segment_time_step = time_step;
  kinematic_model = kinematic_model;
}

PathPlanner::~PathPlanner() {
}

void PathPlanner::set_kinematic_model(IKinemtaicModel* kinematic_model) {
  PathPlanner::kinematic_model = kinematic_model;
}

bool PathPlanner::add_cartesian_path_segment(const CartesianPathSegment& path_segment) {
  auto* new_segment = ct_path_segment_queue.push(path_segment);
  if(new_segment == nullptr) {
    // queue full
    return false;
  }

  // TODO: do look ahead planning of queue
  new_segment->compute_motion_profile(); // for testing

  return true;
}

void PathPlanner::process(bool disable_interrupts_for_queue_update) {
  // create new segment generator for next cartesian path segment
  // the segment stays in the queue until it is completed
  if(segment_generator == nullptr && ct_path_segment_queue.empty() == false) {
    auto* current_segment = ct_path_segment_queue.peek();
    segment_generator = new JointSpacePathSegmentGenerator(current_segment, 
                                                           kinematic_model,
                                                           segment_time_step);
    /*LOG_INFO("Starting segment: duration=%fs, (%f, %f, %f)->(%f, %f, %f) | queue size: %i", 
      current_segment->get_duration(),
      current_segment->start_pose.translation.x,
      current_segment->start_pose.translation.y,
      current_segment->start_pose.translation.z,
      current_segment->end_pose.translation.x,
      current_segment->end_pose.translation.y,
      current_segment->end_pose.translation.z,
      ct_path_segment_queue.size()); */
  }

  // generate joint space segment
  if(segment_generator != nullptr && js_path_segment_queue.full() == false) {
    JointSpacePathSegment segment;
    bool end_reached = segment_generator->generate_next(segment);

    // update output queue
    if(disable_interrupts_for_queue_update) {
      uint32_t status = save_and_disable_interrupts();
      js_path_segment_queue.push(segment);
      restore_interrupts(status); 
    } else {
      js_path_segment_queue.push(segment);
    }

   // LOG_INFO("Adding joint space segment: [%f, %f, %f] -> [%f, %f, %f]", 
   //   segment.start_pos[0], segment.start_pos[1],  segment.start_pos[2],
   //   segment.end_pos[0], segment.end_pos[1],  segment.end_pos[2]);

    // check if current cartesian path segmetn is completed
    if(end_reached) {
      // remove current cartesian path segment from ringbuffer
      ct_path_segment_queue.pop();
      // destroy segment generator
      delete segment_generator;
      segment_generator = nullptr;
    }
  }
}

/**
 * retrieve 
 */
bool PathPlanner::pop_js_path_segment(JointSpacePathSegment& segment) {
  return js_path_segment_queue.pop(segment);
}

bool PathPlanner::all_finished() {
  return js_path_segment_queue.empty() && ct_path_segment_queue.empty() && segment_generator == nullptr;
}

int PathPlanner::input_queue_full() {
  return ct_path_segment_queue.full();
}

int PathPlanner::input_queue_size() {
  return ct_path_segment_queue.size();
}

void PathPlanner::run_look_ahead_planning() {

}
