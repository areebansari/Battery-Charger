#include <18f4520.h>
#device ICD=TRUE ADC=10
#fuses HS,NOLVP,NOWDT,PUT,NOBROWNOUT
#use delay (clock=20000000)
#define STROBE_PIN PIN_A1
#define CLOCK_PIN PIN_A2
#define DATA_PIN PIN_A3
#define SW1 PIN_B0
#define SW2 PIN_B1
#define SW3 PIN_B2
#define SW4 PIN_B3
#define LED1 PIN_C0
#define LED2 PIN_C1
#define LED3 PIN_D0
#define LED4 PIN_D1
#define LED5 PIN_D2
#define LED6 PIN_D3
#define LED7 PIN_D4
#define LED8 PIN_D5
#define LED9 PIN_D6
#define LED10 PIN_D7

