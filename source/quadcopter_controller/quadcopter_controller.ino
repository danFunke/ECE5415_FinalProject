/**
 *
 *
 *
 */

#include "rc_controller.h"
#include "imu.h"

/*** Global variables ***/
uint16_t TIMER_COMPARE_VALUE = 249;  // (16 * 10^6) / (250 * 256) - 1
boolean toggle1 = 0;

/*** Static functions ***/
static void configure_interrupts(void)
{
  cli();  // Disable interrupts

  // Set timer1 interrupt at 250Hz
  TCCR1A = 0;                     // Clear TCCR1A register
  TCCR1B = 0;                     // Clear TCCR1B register
  TCNT1 = 0;                      // Initialize counter value to 0
  OCR1A = TIMER_COMPARE_VALUE;    // 250 Hz w/prescaler set to 256
  TCCR1B |= (1 << WGM12);         // Turn on CTC mode
  TCCR1B |= (1 << CS12);          // Set CS12 bit for 256 prescaler
  TIMSK1 |= (1 << OCIE1A);        // Enable timer compare interrupt

  sei();  // Enable interrupts
}

/*** Interrupt Handlers ***/
ISR(TIMER1_COMPA_vect)
{
  
  
}

uint32_t loop_timer;

void setup() {
  configure_interrupts();
  imu_init();
  rc_controller_init();
  

  // Dev/Debug
  pinMode(8, OUTPUT);
  Serial.begin(57600);

  // Initialize loop timer
  loop_timer = micros();
}

void loop() 
{
  // rc_controller_update();
  imu_update();

  Serial.print("Roll Angle [deg] ");
  Serial.print(imu_get_roll_angle());
  Serial.print(" Pitch Angle [deg] ");
  Serial.println(imu_get_pitch_angle());

  // pause for 4 ms - 250 Hz update frequency
  while(micros() - loop_timer < 4000) {}
  loop_timer = micros();
}
