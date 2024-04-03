#include "rc_receiver.h"
#include "moving_average_filter.h"
#include <PPMReader.h>
#include <stdint.h>

// Global variables
static const uint8_t RC_PPM_INTERRUPT_PIN = 3;
static PPMReader rc_receiver_ppm(RC_PPM_INTERRUPT_PIN, NUM_RC_RECEIVER_CHANNELS);
static moving_average_filter_t channel_values[NUM_RC_RECEIVER_CHANNELS];

void rc_receiver_init(void)
{
  for (int i = 0; i < NUM_RC_RECEIVER_CHANNELS; ++i) {
    moving_average_filter_init(&channel_values[i]);
  }
}

void rc_receiver_update(void)
{
  for (int i = 0; i < NUM_RC_RECEIVER_CHANNELS; ++i) {
    unsigned value = rc_receiver_ppm.latestValidChannelValue(i, 0);
    moving_average_filter_update(&channel_values[i], value);
  }
}

int rc_receiver_get_value(int channel_index)
{
  return channel_values[channel_index].averaged_value;
}















// #include "moving_average_filter.h"
// #include <Arduino.h>

// // Arrays to hold reference pins and values
// const int reference_pins[] = {5, 4, 3, 2};
// moving_average_filter_t reference_values[NUM_REF_VALUES];

// void rc_controller_init(void)
// {
//   for (int i = 0; i < NUM_REF_VALUES; ++i) {
//     filter_init(&reference_values[i]);
//   }
// }

// void rc_controller_update(void)
// {
//   for (int i = 0; i < NUM_REF_VALUES; ++i) {
//     filter_update(&reference_values[i], pulseIn(reference_pins[i], HIGH));
//   }
// }

// float rc_controller_get_value(int ref_idx)
// {
//   float ret = 0;

//   // Map averaged value to pseudo-log function; 
//   if ((reference_values[ref_idx].averaged_value > 1470) && (reference_values[ref_idx].averaged_value < 1510)) {
//     ret = 0;  // Joysticks in neutral position
//   }
//   else if ((reference_values[ref_idx].averaged_value > 1510) && (reference_values[ref_idx].averaged_value < 1800)) {
//     ret = (reference_values[ref_idx].averaged_value - 1510) * 0.0003;
//   }
//   else if (reference_values[ref_idx].averaged_value > 1800) {
//     ret = ((reference_values[ref_idx].averaged_value - 1800) * 0.00175) + 0.15;
//   }
//   else if ((reference_values[ref_idx].averaged_value > 1200) && (reference_values[ref_idx].averaged_value < 1470)) {
//     ret = (reference_values[ref_idx].averaged_value - 1470) * 0.0003;
//   }
//   else if (reference_values[ref_idx].averaged_value < 1200) {
//     ret = ((reference_values[ref_idx].averaged_value - 1200) * 0.002) - 0.15;
//   }

//   return ret;
// }// 