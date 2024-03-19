/**
 *
 *
 *
 */

#ifndef RC_CONTROLLER_H_
#define RC_CONTROLLER_H_

#include "moving_average_filter.h"

// Define reference value names
enum REFERENCE_VALUE_NAMES {
  REF_THETA,
  REF_PHI,
  REF_PSI,
  REF_Z,
  NUM_REF_VALUES
};

// Arrays to hold reference pins and values
const int reference_pins[] = {5, 4, 3, 2};
moving_average_filter_t reference_values[NUM_REF_VALUES];

void rc_controller_init(void)
{
  for (int i = 0; i < NUM_REF_VALUES; ++i) {
    filter_init(&reference_values[i]);
  }
}

void rc_controller_update(void)
{
  for (int i = 0; i < NUM_REF_VALUES; ++i) {
    filter_update(&reference_values[i], pulseIn(reference_pins[i], HIGH));
  }
}

float rc_controller_get_value(enum REFERENCE_VALUE_NAMES ref_val)
{
  float ret = 0;

  // Map averaged value to pseudo-log function; 
  if ((reference_values[ref_val].averaged_value > 1470) && (reference_values[ref_val].averaged_value < 1510)) {
    ret = 0;  // Joysticks in neutral position
  }
  else if ((reference_values[ref_val].averaged_value > 1510) && (reference_values[ref_val].averaged_value < 1800)) {
    ret = (reference_values[ref_val].averaged_value - 1510) * 0.0003;
  }
  else if (reference_values[ref_val].averaged_value > 1800) {
    ret = ((reference_values[ref_val].averaged_value - 1800) * 0.00175) + 0.15;
  }
  else if ((reference_values[ref_val].averaged_value > 1200) && (reference_values[ref_val].averaged_value < 1470)) {
    ret = (reference_values[ref_val].averaged_value - 1470) * 0.0003;
  }
  else if (reference_values[ref_val].averaged_value < 1200) {
    ret = ((reference_values[ref_val].averaged_value - 1200) * 0.002) - 0.15;
  }

  return ret;
}

#endif /* RC_CONTROLLER_H_ */