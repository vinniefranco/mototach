/*
 * Timers:
 *
 * Like all digital systems a timer requires a clock to run.
 * As each clock pulse increments the timer's counter by one,
 * The time measures the intervals in perious of one on the input frequency.
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile uint16_t clock    = 0;
volatile uint8_t  tick     = 0;
volatile uint8_t  running  = 0;

/*
 * LED Pins used as tach.
 */
#define LED_G     OCR1A
#define LED_R     OCR1B

/*
 * Max value for set_intensity()
 */
#define INTENSITY 255

/*
 * Idle    1,000 RPM =  16.6667Hz
 * Redline 8,500 RPM = 141.6667Hz
*/
#define IDLE      16
#define REDLINE   141

/*
 * Our clock count at no-prescale with register overflow
 * 31250 = 8MHz / 256
 */
#define MAX_CLOCK 31250

ISR( TIMER0_OVF_vect )
{
  clock++;
}

ISR ( INT0_vect )
{
  running = 1;
  tick++;
}

/*
 * Setup timer counter register 1 as PWM
 * drives red and green anodes of RGB LED.
 */
void counter1_pwm_init ( void )
{
  TCCR1 |= (1<<PWM1A) | (1<<COM1A0) | (1<<CS10); // Enable PWM, no prescaler.
  GTCCR |= (1<<PWM1B) | (2<<COM1B0);             // Enable use of OC1B, clear OC1B on compare-match, set at BOTTOM
}

/*
 * Enables falled edge pin change interrupt for ignition pulse counting.
 * from low voltage side of ignition coil.
 */
void int0_pin_change_init ( void )
{
  MCUCR |= (1<<ISC01); // Interrupt on falling edge of INT0.
  GIMSK |= (1<<INT0);  // INT0 Interrupt request 0 enable.
}

/*
 * Enables clock cycle counting to derive RPM from clock/ignition ticks.
 */
void timer0_init ( void )
{
  TIMSK  |= (1<<TOIE0);  // Overflow Interrupt enable.
  TCCR0B |= (1<<CS00);   // Set clk/I/O/ No prescaling.
}

void set_intensity ( uint8_t range )
{
  LED_G = INTENSITY - range;
  LED_R = range;
}

void set_output_pins ( void )
{
  DDRB |= (1<<DDB4) | (1<<DDB2) | (1<<DDB1); // Set PB4, PB2 PB1 as output.
}

void startup_routine ( void )
{

  uint8_t start     = 0;
  uint8_t range     = 1;
  uint8_t direction = 1;


  for (; start < 255; ++start)
  {
    LED_G = start;
    _delay_ms(5);
  }

  while (range > 0)
  {

    if ( range >= INTENSITY )
    {
      direction = -1;
    }

    range += direction;
    set_intensity(range);
    _delay_ms(3);
  }

  LED_G = 0;
  _delay_ms(80);
  LED_G = 255;
  _delay_ms(80);
  LED_G = 0;
  _delay_ms(80);
  LED_G = 255;
  _delay_ms(80);
  LED_G = 0;
}

int main ( void )
{

  float rps       = 0.0;
  float intensity = 0.0;

  set_output_pins();
  timer0_init();
  counter1_pwm_init();                       // Enable PWM.
  int0_pin_change_init();                    // Enable INT0.
  sei();                                     // Enable interrupts.

  startup_routine();

  for (;;)
  {
    if (running && tick) {
      cli();                                  // Stop interrupts.

      if ( clock == MAX_CLOCK && tick == 0 )
      {
        running = 0;
        LED_G   = 0;
        LED_R   = 0;
      }
      else
      {

        rps = MAX_CLOCK / (clock / tick); // Munge out RPS.

        if ( rps >= REDLINE )
        {
          rps = 141; // Prevent overflows with over revving.
        }

        intensity = INTENSITY * (rps / 141.0);  // Munge out intensity.
        set_intensity((uint8_t) intensity);     // Set visual RPM.
      }

      clock     = 0;                          // Reset clock.
      tick      = 0;                          // Reset tick.

      sei();                                  // Enable interrupts.
      _delay_us(10);
    }

  }

  return 0;
}
