/**
 *
 *
 *
 */

#ifndef RC_CONTROLLER_H_
#define RC_CONTROLLER_H_

// Define reference value names
enum REFERENCE_VALUE_NAMES {
  REF_THETA,
  REF_PHI,
  REF_PSI,
  REF_Z,
  NUM_REF_VALUES
};

void rc_controller_init(void);

void rc_controller_update(void);

float rc_controller_get_value(int ref_idx);

#endif /* RC_CONTROLLER_H_ */