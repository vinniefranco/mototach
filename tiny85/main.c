#include <stdio.h>
#include <math.h>

// Include all AVR headers
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>

#include <util/delay.h>

#define PWM_R OCR0A
#define PWM_G OCR0B

// Mocking voltage input
#define POWER_MIN 1
#define POWER_MAX 1024
#define SHIFT_AT  5

#define MAX_LUM             255
#define TOTAL_RANGE MAX_LUM * 3
#define ASC                   1
#define DESC                 -1

typedef struct
{
  unsigned int red;
  unsigned int green;
  unsigned int blue;
} ColorRGB; // Defines color space

void     power_to_luminance   ( uint16_t power , ColorRGB * _rgb );
uint16_t power_for            ( uint16_t power );
void     run_startup_sequence ( ColorRGB * _initRGB );
void     set_flash            ( int toggle );
void     set_pwm_intensities  ( ColorRGB * __rgb );

int main (void)
{

  /* - Attiny85 Setup ------------------------------------------------------ */

  DDRB = (1<<PB1) | (1<<PB0); // Set pins 5,6 as output.

  // PWM -----------------------------------------------------------------------


  TCCR0A = (1<<COM0A1) | (1<<COM0A0); // Non inverting mode (OC0A and OC0B)
  TCCR0A |= (1<<WGM01) | (1<<WGM00); // Fast PWM
  TCCR0B = (1<<CS00); // No prescaling

  // ADC -----------------------------------------------------------------------
  /*  ADCSRA |=  (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);    //Prescaler at 128 so we have an 125Khz clock source
  ADMUX  |=  (1<<REFS0);                              //
  ADMUX  &= ~(1<<REFS1);                              //Avcc(+5v) as voltage reference
  ADCSRB &= ~((1<<ADTS2) | (1<<ADTS1) | (1<<ADTS0));  //ADC in free-running mode
  ADCSRA |= (1<<ADATE);                               //Signal source, in this case is the free-running
  ADCSRA |= (1<<ADEN);                                //Power up the ADC
  ADCSRA |= (1<<ADSC);                                //Start converting
  *//* ------------------------------------------------------------------------ */

  ColorRGB rgb = {.red = 0, .green = 0, .blue = 0}; // Holds intensity values.

  int toggle = 1;
  int adc_value; // Stores ADC conversion.

  run_startup_sequence(&rgb);

  /*
   * Program loop
   */
  for (;;)
  {
    //read_fake_voltage(&test);

    adc_value = ADCW;

    if (adc_value < POWER_MAX - SHIFT_AT) {
      power_to_luminance(adc_value, &rgb); // Get intensities
      set_pwm_intensities(&rgb);           // Set intensities

      _delay_us(1024);
    }
    else
    {
      toggle = !toggle;     // Invert toggle
      set_flash(toggle); // Set flash

      _delay_ms(32);
    }

  }

  return 0;
}

void set_flash ( int toggle )
{
  PWM_R  = toggle ? MAX_LUM : 0;
  PWM_G  = 0;
}

void set_pwm_intensities ( ColorRGB * __rgb )
{
  // Assign tach RGB intensities.
  PWM_R = __rgb->red;
  PWM_G = __rgb->green;
}

void run_startup_sequence ( ColorRGB * _initRGB )
{
  int      direction = ASC;
  uint16_t sweep     = 1;

  for (;;)
  {
    if ( sweep >= POWER_MAX )
    {
      direction = DESC;
    }

    sweep += direction;

    if ( sweep < POWER_MIN )
    {
      break;
    }

    power_to_luminance(sweep, _initRGB);

    PWM_R = _initRGB->red;
    PWM_G = _initRGB->green;

    _delay_us(1024);
  }
}

uint16_t power_for ( uint16_t power )
{
  float    power_float;
  uint16_t voltage;

  power_float = (float) power;
  voltage     = floor(TOTAL_RANGE * ( power_float / POWER_MAX ));

  return voltage;
}

void power_to_luminance ( uint16_t power , ColorRGB * _rgb )
{
  int           remaining = -1;
  uint16_t      revs      = power_for(power);
  unsigned int  values[3] = {0,0,0};

  while (remaining != 0)
  {
    remaining = revs / MAX_LUM;

    if (remaining > 0)
    {
      values[0] = 255;
    }

    if (remaining > 1)
    {
      values[1] = 255;
    }

    revs = revs % MAX_LUM;
  }

  if (values[0] && values[1] && !values[2])
  {
    values[2] = revs;
  }

  if (values[0] && !values[1] && !values[2])
  {
    values[1] = revs;
    values[2] = 0;
  }

  if (!values[0] && !values[1] && !values[2])
  {
    values[0] = revs;
    values[1] = values[2] = 0;
  }

  if (values[1] > 0)
  {
    values[0] -= values[1];
  }

  if (values[2] > 0) {
    values[1] -= values[2];
  }

  _rgb->green = values[1];
  _rgb->red   = values[2];
}
