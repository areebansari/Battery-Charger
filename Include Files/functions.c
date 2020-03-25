/*============================================================================*/
/***
   * Function      : void lcd_print(int data)
   * Parameters    : data 
   * Return value  : none
   * Purpose       : Prints 8 bit data on LCD serially
***/
/*============================================================================*/
void lcd_print(int data)
{
   int b=0b10000000;
    do
    {     
       output_bit(DATA_PIN,data & b);
       output_low(CLOCK_PIN);
       output_high(CLOCK_PIN);
       rotate_right(&b , 1);
    } while(b!=0x80);
    
   output_low(STROBE_PIN);
   delay_cycles(2);
   output_high(STROBE_PIN);
}

/*============================================================================*/
/***
   * Function      : void lcd_out(int n, long val)
   * Parameters    : n, val 
   * Return value  : none
   * Purpose       : Manipulates and converts data to a form that could be
   *                 printed on LCD. 
   *                 first arguement is for selection of LCD. 
   *                 1 for upper LCD and 2 for lower LCD
***/
/*============================================================================*/
void lcd_out(int n, long val)
{              // 
   int hundreds=0, tens=0, units=0, tenths=0;
   switch(n)
   {
   case 1:  // for LCD 1 the segment addresses are 0x40, 0x50, 0x60 and 0x70
   {
      while   (val>=1000)      // separate hundreds from the number
      {
      val-=1000;
      hundreds++;
      }
   
      while   (val>=100)      // separate tens from the number
      {
      val-=100;
      tens++;
      }
   
      while   (val>=10)       // separate units from the number                   
      {
      val-=10;
      units++;
      }
      
      while   (val>=1)       // separate tenths from the number                  
      {
      val-=1;
      tenths++;
      }
      
   if(hundreds==0) lcd_print(0x70 | 0x0A);
   // blank third segment if number is less than 100
   
   else lcd_print(0x70 | hundreds);
   // else print address of segment ORed with  respective data
   
   if (hundreds==0 && tens==0) lcd_print(0x60 | 0x0A);
   // blank third and fourth segment if number is less than 10
   
   else lcd_print(0x60 | tens);
   // else print address of segment ORed with  respective data
   
   lcd_print(0x50 | units);
   // print address of segment ORed with respective data
   
   lcd_print(0x40 | tenths);
   // print address of segment ORed with respective data
   } break;
   
   case 2: // for LCD 2 the segment addresses are 0x00, 0x10, 0x20 and 0x30
   {
      while   (val>=1000)
      {
      val-=1000;
      hundreds++;
      }
   
      while   (val>=100)
      {
      val-=100;
      tens++;
      }
   
      while   (val>=10)
      {
      val-=10;
      units++;
      }
      
      while   (val>=1)
      {
      val-=1;
      tenths++;
      }
      
   if(hundreds==0) lcd_print(0x30 | 0x0A);
   else lcd_print(0x30 | hundreds);
   
   if (hundreds==0 && tens==0) lcd_print(0x20 | 0x0A);
   else lcd_print(0x20 | tens);
   
   lcd_print(0x10 | units);
   lcd_print(0x00 | tenths); 
 
   } break;
   default: break;
   }                              
}

/*============================================================================*/
/***
   * Function      : void lcd_flash(short k, long lcd_value)
   * Parameters    : k, lcd_value 
   * Return value  : none
   * Purpose       : Function for flashing LCD when in edit mode.
   *                 First arguement selects to blank the LCD or to show normal
   *                 data. Second arguement holds data which needs to be flashed
***/
/*============================================================================*/
void lcd_flash(short k, long lcd_value) 
{         

   switch(k)      
   {
      case 0:    // LCD2 is blanked when switch variable is 0
      {
         lcd_print(0x30 | 0x0A);
         lcd_print(0x20 | 0x0A);
         lcd_print(0x10 | 0x0A);
         lcd_print(0x00 | 0x0A); break;  
      }
      
      case 1:   // normal data is printed on LCD2 when switch variable is 1 
      {
         lcd_out(2, lcd_value); break;
      }
   }
}

/*============================================================================*/
/***
   * Function      : void led_on(int led)
   * Parameters    : led 
   * Return value  : none
   * Purpose       : Function for switching on respective LED.
***/
/*============================================================================*/
void led_on(int led)
{
   switch(led)
   {
   case 1: output_high(PIN_C0); break;
   case 2: output_high(PIN_C1); break;
   case 3: output_high(PIN_D0); break;
   case 4: output_high(PIN_D1); break;
   case 5: output_high(PIN_D2); break;
   case 6: output_high(PIN_D3); break;
   case 7: output_high(PIN_D4); break;
   case 8: output_high(PIN_D5); break;
   case 9: output_high(PIN_D6); break;
   case 10: output_high(PIN_D7); break;
   }
}

/*============================================================================*/
/***
   * Function      : void led_off(int led)
   * Parameters    : led 
   * Return value  : none
   * Purpose       : Function for switching off respective LED.
***/
/*============================================================================*/
void led_off(int led)      
{
   switch(led)
   {
   case 1: output_low(PIN_C0); break;
   case 2: output_low(PIN_C1); break;
   case 3: output_low(PIN_D0); break;
   case 4: output_low(PIN_D1); break;
   case 5: output_low(PIN_D2); break;
   case 6: output_low(PIN_D3); break;
   case 7: output_low(PIN_D4); break;
   case 8: output_low(PIN_D5); break;
   case 9: output_low(PIN_D6); break;
   case 10: output_low(PIN_D7); break;
   }
}

/*============================================================================*/
/***
   * Function      : void all_leds_off()
   * Parameters    : none 
   * Return value  : none
   * Purpose       : Function for switching off all LEDs.
***/
/*============================================================================*/
void all_leds_off()      
{
   output_low(LED1);
   output_low(LED2);
   output_d(0);
}

/****************************** END OF FILE  **********************************/


