#pragma once

#include <vector>
#include <cstdint>
#include <cmath>


class LookupTable {
  public:
    LookupTable() {}

    // initializes the lookup table to a given size and input range
    void  init(int32_t size, float input_min, float input_max);

    // clear the lookup table, use init to use it again
    void  clear();

    // returns the size of the lookup table
    uint32_t size();

    // set an entry of the lookup table
    void  set_entry(int32_t idx, float v);

    // set an entry of the lookup table
    float  get_entry(int32_t idx);

    // evaluate the lookup table at a given position with linear interpolation
    float evaluate(float x) const;

    // evaluate the inverse of the lookup table function (very slow), the LUT must be monotonic
    float evaluate_inverse(float y) const;

    // inverts the lookup table so it represents the funcion x = fi(y) given y = f(x) 
    bool invert(int new_size);

    // check if the lookup table is monotonic
    bool is_monotonic() const;

    // prints the lookup table using the logger
    void print_to_log() const;

  private:
    float input_min = 0.0f;
    float input_max = 0.0f;
    float one_over_input_range = 1.0f;
    std::vector<float> lookup_table;
};

//*** FUNCTION ***********************************************************************************/

