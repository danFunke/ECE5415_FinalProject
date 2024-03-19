/**
 *
 *
 *
 */

#include "rc_controller.h"

/*** Global variables ***/
uint16_t TIMER_COMPARE_VALUE = 249;  // (16 * 10^6) / (250 * 256) - 1
boolean toggle1 = 0;

/*** Static functions ***/
static void configure_interrupts(void)
{
  cli();  // Disable interrupts

  // Set timer1 interrupt at 1Hz
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

  // Dev/Debug
  if (toggle1) {
    digitalWrite(8, HIGH);
    toggle1 = 0;
  } else {
    digitalWrite(8, LOW);
    toggle1 = 1;
  }
}

void setup() {
  // configure_interrupts();
  rc_controller_init();

  // Dev/Debug
  pinMode(8, OUTPUT);
  Serial.begin(9600);
}

void loop() 
{
  rc_controller_update();

  // Dev/Debug
  for (int i = 0; i < NUM_REF_VALUES; ++i) {
    Serial.print(rc_controller_get_value(i));
    Serial.print(',');
  }
  Serial.println();

}
