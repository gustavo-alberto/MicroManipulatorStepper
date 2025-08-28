// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#include "encoder_lut.h"
#include "pico/stdlib.h"
#include "utilities/logging.h"
#include "utilities/math_constants.h"

void LookupTable::init(int32_t size, float input_min, float input_max) {
  lookup_table.clear();
  lookup_table.resize(size, 0.0f);
  LookupTable::input_min = input_min;
  LookupTable::input_max = input_max;
  LookupTable::one_over_input_range = 1.0f/(input_max-input_min);
}

void LookupTable::clear() {
  lookup_table.clear();
}

// returns the size of the lookup table
uint32_t LookupTable::size() {
  return (uint32_t)lookup_table.size();
}

void LookupTable::set_entry(int32_t idx, float v) {
  lookup_table[idx] = v;
}

// set an entry of the lookup table
float LookupTable::get_entry(int32_t idx) {
  return lookup_table[idx];
}

float LookupTable::evaluate(float x) const {
  if (lookup_table.empty() || lookup_table.size() < 2)
    return 0.0f;

  int32_t table_size = lookup_table.size();
  float t = (x - input_min) * one_over_input_range;
  float pos = t * (table_size - 1);
  float frac;
  size_t index;

  if (t < 0.0f) {
    return lookup_table.front();
    // Extrapolate to the left using first two points
    index = 0;
    frac = pos;  // pos is negative
  } else if (t >= 1.0f) {
    return lookup_table.back();
    // Extrapolate to the right using last two points
    index = table_size - 2;
    frac = pos - (table_size - 2);
  } else {
    // Interpolate normally
    index = static_cast<size_t>(std::floor(pos));
    frac = pos - index;
  }

  float a = lookup_table[index];
  float b = lookup_table[index + 1];

  return a + frac * (b - a);  // Linear interpolation or extrapolation
}


bool LookupTable::is_monotonic() const {
  if (lookup_table.size() < 2)
    return true;

  bool increasing = true, decreasing = true;
  for (size_t i = 1; i < lookup_table.size(); ++i) {
    float b = lookup_table[i - 1];
    float a = lookup_table[i];

    if (a < b) increasing = false;
    if (a > b) decreasing = false;
  }

  return increasing || decreasing;
}

bool almost_equal(float a, float b, float rel_tol = 1e-6f, float abs_tol = 1e-6f) {
    return std::fabs(a - b) <= std::max(rel_tol * std::max(std::fabs(a), std::fabs(b)), abs_tol);
}

float LookupTable::evaluate_inverse(float y) const {
  int size = static_cast<int>(lookup_table.size());
  if (size < 2) return input_min;

  int low = 0;
  int high = size - 1;
  bool increasing = lookup_table.front() < lookup_table.back();

  // Clamp y outside the range
// Clamp y outside the range
if ((increasing && y <= lookup_table.front()) || 
    (!increasing && y >= lookup_table.front()))
  return input_min;
if ((increasing && y >= lookup_table.back()) || 
    (!increasing && y <= lookup_table.back()))
  return input_max;

  // Binary search to find the interval
  while (high - low > 1) {
    int mid = (low + high) / 2;
    float val = lookup_table[mid];

    if ((increasing && val < y) || (!increasing && val > y))
      low = mid;
    else
      high = mid;
  }

  // Interpolate between low and high
  float y0 = lookup_table[low];
  float y1 = lookup_table[high];

  if (std::fabs(y1 - y0) < std::numeric_limits<float>::epsilon()) {
    // Avoid division by zero if both entries are equal
    float t = float(low) / (size - 1);
    return input_min + t * (input_max - input_min);
  }

  float t = (y - y0) / (y1 - y0);
  float pos = (float(low) + t) / (size - 1);

  return input_min + pos * (input_max - input_min);
}

// inverts the lookup table so it represents the funcion x = fi(y) given y = f(x) 
bool LookupTable::invert(int new_size) {
  if (lookup_table.empty() || new_size <= 0) {
    LOG_ERROR("invert_lut(): lut size is zero");
    return false;
  }

  if (!is_monotonic()) {
    LOG_ERROR("invert_lut(): lut is not monotonic");
    return false;
  }

  // Find the output (y) range of the current LUT
  float output_min = lookup_table.front();
  float output_max = lookup_table.back();
  if (output_max < output_min) {
      std::swap(output_min, output_max);
  }

  // Prepare new LUT data
  std::vector<float> new_lut(new_size);
  float delta_y = (output_max - output_min) / (new_size - 1);

  for (int i = 0; i < new_size; ++i) {
      float y = output_min + i * delta_y;
      new_lut[i] = evaluate_inverse(y);  // find x for given y
  }

  // Replace old LUT with the inverted LUT
  lookup_table = std::move(new_lut);
  input_min = output_min;
  input_max = output_max;
  one_over_input_range = 1.0f/(input_max-input_min);

  return true;
}

void LookupTable::print_to_log() const {
  int size = lookup_table.size();
  if (size == 0) return;

  float step = (input_max - input_min) / (size - 1);
  for (int i = 0; i < size; ++i) {
    float x = input_min + i * step;
    float y = lookup_table[i];
    LOG_INFO("%.6f;%.6f", x, y);
  }
}