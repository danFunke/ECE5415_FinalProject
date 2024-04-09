/**
 *
 *
 *
 */

#ifndef RC_RECEIVER_H_
#define RC_RECEIVER_H_

// Define reference value names
enum RC_RECEIVER_CHANNELS {
  RC_RECEIVER_CHANNEL_0,  // nothing?
  RC_RECEIVER_CHANNEL_1,  // ROLL
  RC_RECEIVER_CHANNEL_2,  // PITCH
  RC_RECEIVER_CHANNEL_3,  // THROTTLE
  RC_RECEIVER_CHANNEL_4,  // YAW
  RC_RECEIVER_CHANNEL_5,  // RIGHT TRIGGER SWITCH
  RC_RECEIVER_CHANNEL_6,  // RIGHT BUMPER BUTTON
  RC_RECEIVER_CHANNEL_7,  // LEFT TRIGGER SWITCH
  RC_RECEIVER_CHANNEL_8,  // Left trimpot
  NUM_RC_RECEIVER_CHANNELS
};

void rc_receiver_init(void);

void rc_receiver_update(void);

int rc_receiver_get_value(int channel_index);

#endif /* RC_RECEIVER_H_ */