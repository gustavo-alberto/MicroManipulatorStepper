// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

//*** INCLUDE ***************************************************************************

#include "motion_controller.h"
#include "path_planner.h"
#include "utilities/logging.h"

//*** CLASS *****************************************************************************

MotionController::MotionController(PathPlanner* path_planner) {
  MotionController::path_planner = path_planner;
  current_time = 0.0f;
}

bool MotionController::update(float dt, float* joint_positions, float* joint_velocities) {
  // increment time counter
  current_time += dt;

  // check if end of current path segment exceeded and if so, fetch next one
  float segment_duration = current_path_segment.get_duration();
  while(current_time > segment_duration) {
    // get next path segment from queue
    bool queue_empty = !path_planner->pop_js_path_segment(current_path_segment);
    if(queue_empty) {
      current_time = segment_duration;
      break;
    }
    
    // update current time and segment duration
    current_time -= segment_duration;
    segment_duration = current_path_segment.get_duration();
  }

  if(!current_path_segment.is_initialized())
    return false;

  // evaluate path segment
  current_path_segment.evaluate(current_time, joint_positions, joint_velocities);
  return true;
}
