/**
 *
 *
 *
 */

#ifndef RC_RECEIVER_H_
#define RC_RECEIVER_H_

// Define reference value names
enum RC_RECEIVER_CHANNELS {
  RC_RECEIVER_CHANNEL_1,  // Roll
  RC_RECEIVER_CHANNEL_2,  // Pitch
  RC_RECEIVER_CHANNEL_3,  // Throttle
  RC_RECEIVER_CHANNEL_4,  // Yaw
  RC_RECEIVER_CHANNEL_5,  // Right Trigger Switch
  RC_RECEIVER_CHANNEL_6,  // Right Bumper
  RC_RECEIVER_CHANNEL_7,  // left Trigger Switch
  RC_RECEIVER_CHANNEL_8,  // Left Trimpot - doesnt seem to work?
  NUM_RC_RECEIVER_CHANNELS
};

void rc_receiver_init(void);

void rc_receiver_update(void);

int rc_receiver_get_value(int channel_index);

#endif /* RC_RECEIVER_H_ */