#include "rc_receiver.h"

void setup(void){
  Serial.begin(115200);
  rc_receiver_init();
}

void loop(){
  rc_receiver_update();

  Serial.print("ROLL:");
  Serial.print(rc_receiver_get_value(RC_RECEIVER_CHANNEL_1));
  Serial.print(",");
  Serial.print("     PITCH:");
  Serial.print(rc_receiver_get_value(RC_RECEIVER_CHANNEL_2));
  Serial.print(",");
  Serial.print("     YAW:");
  Serial.print(rc_receiver_get_value(RC_RECEIVER_CHANNEL_4));
  Serial.print(",");
  Serial.print("     THROTTLE:");
  Serial.print(rc_receiver_get_value(RC_RECEIVER_CHANNEL_3));
  Serial.print(",");
  Serial.print("    CHANNEL 5:");
  Serial.print(rc_receiver_get_value(RC_RECEIVER_CHANNEL_5));
  Serial.print(",");
  Serial.print("     CHANNEL 6:");
  Serial.print(rc_receiver_get_value(RC_RECEIVER_CHANNEL_6));
  Serial.print(",");
  Serial.print("     CHANNEL 7:");
  Serial.print(rc_receiver_get_value(RC_RECEIVER_CHANNEL_7));
  Serial.print(",");
  Serial.print("     CHANNEL 8:");
  Serial.print(rc_receiver_get_value(RC_RECEIVER_CHANNEL_8)); 
  Serial.println();
}