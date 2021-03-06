/*******************************************************************************
**    Version : 0.4.22                                                        **
**    Date : 10-Jan-2015                                                      **
**    Author : Muhib Raza (muhibraza@hotmail.com)                             **
**    Project : Programmable Thyristor Controlled Battery Charger             **
**    Designed for : Advance Electronics International Co.                    **
**    Brief : Code for Slave CPU PIC18F4520 (HMI Circuit)                     **
**                                                                            **
** ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ **
**   Pin assignment :                                                         **
**       PIN_A1 ->  Strobe Pin for LCD (OUT)                                  **
**       PIN_A2 ->  Clock Pin for LCD (OUT)                                   **
**       PIN_A3 ->  Data Pin for LCD (OUT)                                    **
**       PIN_B0 ->  Switch1 (IN)                                              **
**       PIN_B1 ->  Switch2 (IN)                                              **
**       PIN_B2 ->  Switch3 (IN)                                              **
**       PIN_B3 ->  Switch4 (IN)                                              **
**       PIN_C0 ->  LED1 (OUT)                                                **
**       PIN_C1 ->  LED2 (OUT)                                                **
**       PIN_C3 ->  Used as SCL for I2C communication                         **
**       PIN_C4 ->  Used as SDA for I2C communication                         **
**       PIN_C5 ->  Driver Enable [DE] (OUT)                                  **
**       PIN_C6 ->  MODBBUS/RS232 Tx                                          **
**       PIN_C7 ->  MODBBUS/RS232 Rx                                          **
**       PIN_D0 ->  LED3 (OUT)                                                **
**       PIN_D1 ->  LED4 (OUT)                                                **
**       PIN_D2 ->  LED5 (OUT)                                                **
**       PIN_D3 ->  LED6 (OUT)                                                **
**       PIN_D4 ->  LED7 (OUT)                                                **
**       PIN_D5 ->  LED8 (OUT)                                                **
**       PIN_D6 ->  LED9 (OUT)                                                **
**       PIN_D7 ->  LED10 (OUT)                                               **
**                                                                            **
**   EEPROM Location Map :                                                    **
**       Location 0 : e_flag (for checking first run after being programmed)  **
**       Location 1 : voltage setpoint (low byte)                             **
**       Location 2 : voltage setpoint (high byte)                            **
**       Location 3 : current setpoint (low byte)                             **
**       Location 4 : current setpoint (high byte)                            **
**       Location 5 : Parameter 3 (low byte)                                  **
**       Location 6 : Parameter 3 (high byte)                                 **
**       Location 7 : Parameter 4 (low byte)                                  **
**       Location 8 : Parameter 4 (high byte)                                 **
**       Location 9 : Parameter 5 (low byte)                                  **
**       Location 10 : Parameter 5 (high byte)                                **
**                                                                            **
**   Interrupt -> Associated Function :                                       **
**       INT_TIMER3 -> void timer_3_isr(void)                                 **
**       INT_SSP    -> void SSP_isr(void)                                     **
**                                                                            **
**   Function Description :                                                   **
**       [+] int f(int k) - Function that returns location for EEPROM         **
**                          read/write                                        **
**       [+] void edit_value(long lcdval) - Function for editing parameters   **
**       [+] void lcd_print(int data) - Function for printing data on LCD     **
**       [+] void lcd_out(int n,long val) - Function for LCD data arrangement **
**       [+] void lcd_flash(short k,long lcd_value) - Function for flashing LCD*
**       [+] void led_on(int led) - Function for turning on specific LED      **
**       [+] void led_off(int led) - Function for turning off specific LED    **
**       [+] void all_leds_off() - Function for turning off all LEDs          **
**                                                                            **
**       [#] SSP_isr() - ISR for I2C communication                            **
**       [#] timer_3_isr() - ISR for timer3, used for generating fixed        **
**                           interrupts at a rate of 90ms                     **
*******************************************************************************/

/* INCLUDES & OTHER PREPROCESSORS --------------------------------------------*/
#include "hardwaredefinitions.h"   // File containing hardware definitions
#include "functions.c"         // File containing functions for LCD and LED
#USE I2C(SLAVE,SDA=PIN_C4, SCL=PIN_C3,ADDRESS=0xA0)

/* DEFINES -------------------------------------------------------------------*/
/* Maximum voltage to which editing of voltage set point is limited */
#define max_voltage   30

/* Minimum voltage to which editing of voltage set point is limited */
#define min_voltage   0

/* maximum value of current to which editing of voltage set point is limited */
#define max_current_in_mA   960 

/* Minimum value of current to which editing of voltage set point is limited */
#define min_current_in_mA   0

/* Initial voltage setpoint that is effective when program executed first time*/
#define initial_setpoint_for_voltage 13.2

/* Initial current limit that is effective when program executed first time */
#define initial_current_limit_in_mA 500

/* GLOBAL VARIABLES ----------------------------------------------------------*/
int1 show_parameters_mode=0, edit_parameters_mode=FALSE, flash=FALSE;
int state=0 , e_flag=0, address, a, low_byte, high_byte, delay_time=125;
int edit_counter=0, flash_counter=0, menu_timeout_monitor=0;
int data[500], reading[128];
long loop_execution_counter=0;
long value[5];
long max_setpoint_vo=max_voltage*10;
long min_setpoint_vo=min_voltage*10;
long max_current_limit=max_current_in_mA*10;
long min_current_limit=min_current_in_mA*10;

/*============================================================================*/
/***
   * Function      : void SSP_isr(void)
   * Parameters    : None  
   * Return value  : None
   * Purpose       : Sends/Receives Data to/from master CPU
***/
/*============================================================================*/
#INT_SSP
void SSP_isr(void) 
{
   state = i2c_isr_state();
      if (state < 128)
      {
         /* If device address matched, then receive data from master */
         if(state==0) address=i2c_read();

         else if (state >= 1 && state < 128)    
         reading[state-1] = i2c_read();  // Store Data in array one by one
      }  
      
      else if (state >= 128)  // If Master is requesting Data
      i2c_write (data[state - 127]);       // Send Data to Master
}

/*============================================================================*/
/***
   * Function      : void timer_3_isr(void)
   * Parameters    : None  
   * Return value  : None
   * Purpose       : Used for timeouts and flashing purposes
***/
/*============================================================================*/
#INT_TIMER3
void timer_3_isr(void)
{   
   if (show_parameters_mode==TRUE && edit_parameters_mode==TRUE) edit_counter++;
   // edit_counter increments every 90 ms when in edit mode
   
   else if (show_parameters_mode==TRUE) menu_timeout_monitor++;
   // menu timeout counter increments every 90 ms when in menu stage 0
   
   flash_counter++;     // flash_counter increments every 90 ms
   
   if (flash_counter==3) // if 90ms*3= 270 ms have passed, then complement flag
   { flash=~flash; flash_counter=0;}  
   
   set_timer3(9286);      // set timer 3 to generate interrupt every 90 ms
}

/*============================================================================*/
/***
   * Function      : int f(int k)
   * Parameters    : int k  
   * Return value  : int
   * Purpose       : Returns location for EEPROM read and write
***/
/*============================================================================*/
int f(int k)
{
   return (k*2)+1;
}

/*============================================================================*/
/***
   * Function      : void edit_value(long lcdval)
   * Parameters    : long lcdval 
   * Return value  : none
   * Purpose       : Edits parameter values
***/
/*============================================================================*/
void edit_value(long lcdval)
{  
   led_on(3);       // LED 3 is on for indication of menu stage 1 (or edit mode)
   edit_parameters_mode=TRUE;   // Flag to indicate edit mode

   do
   {  
      /* call function for flashing LCD with lcdval every 270 ms */
      lcd_flash(flash,lcdval);
      
      while(!input(SW2))
         {
            delay_ms(delay_time);  /* delay for debouncing. Also used for fast
                                    *  increment/decrement */
            edit_counter=0; /* reset edit counter on button press so that 
                             * timeout does not occur */
            loop_execution_counter++;   // increment on every execution
            
            /* a is the pointer for parameter value. 
             * if a is 0 then parameter 1 is selected */
            if (a==0)
            {
               /* Parameter 1 is setpoint voltage. 
                * No increment if max value reached */
               if(lcdval==max_setpoint_vo) break;
               
               lcdval++;   /* Increment one value if within limits.
                            * decimal point in LCD shows 0.1 Volt increment */
            }
            
            else if (a==1)   // if a is 1 then parameter 2 is selected
            {
               /* parameter 2 is max current limit. 
                * No increment if maximum value reached */
               if(lcdval==max_current_limit) break;

               if(loop_execution_counter>=32) delay_time=10; 
               /* change debouncing delay from 125 to 10 if button has been
                * pressed for 4 sec. 
                * 125 ms * 32 = 4 sec  */
 
               lcdval+=10; /* Increment 10 values if within limits. 
                            * decimal point in LCD will show 1 mA increment */
            }
            lcd_out(2,lcdval);   // display the final value on LCD2
            
            /* value array used for storing 5 parameters against respective 
             * pointer a */
            value[a]=lcdval;
            low_byte=value[a];    // separate low byte of int16 data
            high_byte=value[a]>>8;   // shift right and separate high byte
              
            // store low byte in data array on location 1 when pointer a=0
            data[f(a)]=low_byte;
              
            // store high byte in data array on location 2 when pointer a=0
            data[f(a)+1]=high_byte;
              
              /* Data array will be accessed by Master PC through ISR. The
               * master controller reads edited parameter values after every
               * second. The data array needs to be updated every time there is
               * a change in parameter */
         }
         while(!input(SW3)) // this loop is for decrement.
         {
            delay_ms(delay_time);
            edit_counter=0;         
            loop_execution_counter++;
            if (a==0)
            {
               if(lcdval==min_setpoint_vo) break;
               lcdval--;
            }
            else if (a==1)
            {
               if(lcdval==min_current_limit) break;
               if(loop_execution_counter>=32) delay_time=10;
               lcdval-=10;
            }
            lcd_out(2,lcdval);
            value[a]=lcdval;
            low_byte=value[a];
            high_byte=value[a]>>8;
            data[f(a)]=low_byte;
            data[f(a)+1]=high_byte;
         }
      loop_execution_counter=0;
      delay_time=125;
   
   // if no editing is done for 56 * 90 ms = 5.04 second, then break
   if (edit_counter==56) break;
   }while(input(SW1));      // exit from loop when switch 1 is pressed
   
   delay_ms(50);            // Delay for debouncing
   while(!input(SW1));      // halt here if bounce still exists
   
   edit_counter=0;           // reset edit counter
   
   /* clear flag to indicate an exit from "edit parameters mode" */
   edit_parameters_mode=FALSE;
   led_off(3);               // turn off indication LED3
}

/*============================================================================*/
/***
   * Function     : void main(void)
   * Parameters   : None  
   * Return value : None
   * Purpose      : Serves for the following purposes:
   *                [+] Displays voltage and current values on LCDs
   *                [+] Scans menu button SW4 and calls edit function if pressed
   *                [+] Reads/writes values to EEPROM
   *                [+] Loops forever
***/
/*============================================================================*/
void main(void)
{ 
   output_high(STROBE_PIN);   // De-activate the strobe pin
   output_high(CLOCK_PIN);      // De-activate the clock pin
   all_leds_off();            // Make all LEDs off initially
   
   enable_interrupts(INT_SSP);   // Enable Interrupt for SPI or I2C activity
   setup_timer_3 (T3_INTERNAL | T3_DIV_BY_8); //16-Bit Timer;1.6 usec for 1 tick
   enable_interrupts(GLOBAL);      // Enable global interrupts
   
   // long variable to hold values to be shown on LCD 1 and LCD 2
   long lcd_1_val, lcd_2_val; 
   
   /* long variables that hold values of voltage and current received from
    * master controller */
   long voltage, current;
   long temp_val;     // temporary LCD value that is going to be edited by user
   
   /* variables that hold high and low values of parameters */
   long high_val, low_val;
   
   // menu array with five menu numbers from 1 till 5, decimal point is in LCD
   int menu[5]={10,20,30,40,50};   
   reading[0]=0xB8;
   reading[2]=0xB8;
   reading[1]=0x22;
   reading[3]=0x22;
   /* These four lines are for displaying 8888 on both LCDs when no data has
    * been received from Master controller */
   
   
   e_flag=read_eeprom(0);
   /* Flag to hold value of first location of EEPROM after being programmed
    * the internal EEPROM hold 0xFF value in all locations initially */

   /* Write arbitrary values into EEPROM if Started for the first time
    * if e_flag holds a non-zero value then execute the following commands */
   if (e_flag)
   {
      e_flag=0;   // make the e_flag=0
      write_eeprom(0,e_flag); // store 0 in location 0 of EEPROM
      
      // voltage set_point multiplied by 10 and stored as parameter 1
      value[0]=initial_setpoint_for_voltage*10;
      
      // max current limit multiplied by 10 and stored as parameter 2
      value[1]=initial_current_limit_in_mA*10;   
      
      value[2]=2500;      // arbitrary value for parameter 3
      value[3]=3500;      // arbitrary value for parameter 4
      value[4]=6500;      // arbitrary value for parameter 5

      // copy the initial values into EEPROM on respective locations
      for (a=0 ; a<=4 ; a++)
      { 
         // copy low byte of value array for respective pointer 
         low_byte=value[a];
         
         // copy high byte of value array for respective pointer
         high_byte=value[a]>>8;
         
         //store low byte in EEPROM on location determined by f(a)
         write_eeprom(f(a),low_byte);
         
         //store high byte in EEPROM just one location above f(a)
         write_eeprom(f(a)+1,high_byte);
       }
   }

   /* Read the 5 parameter values from EEPROM into array */
   // copy the values from EEPROM into data array that would be sent to master
   for (a=0 ; a<5 ; a++)
      {  
         // copy low byte from EEPROM for every respective pointer
         data[f(a)]=read_eeprom(f(a));
         
         // copy high byte from EEPROM for every respective pointer
         data[f(a)+1]=read_eeprom(f(a)+1);
         
         low_val=read_eeprom(f(a));      // copy low byte into 16 bit variable
         high_val=read_eeprom(f(a)+1); // copy high byte into 16 bit variable
         high_val=high_val<<8;   // shift high value to left 8 times
         value[a]=high_val | low_val;
         // concatenate high and low bytes and store 16 bit variable in array
      }
      
   a=0; // variable a is used as pointer for array elements in the whole program
   
//********************** Main Loop that repeats forever **********************//
   do
   { 
      /* Following 4 lines read and display the voltage and current reading 
      that is received from master CPU */
   // concatenate lower and higher bytes of voltage value
   voltage= make16(reading[1],reading[0]);
   
   // concatenate lower and higher bytes of current value
   current= make16(reading[3],reading[2]);
   
   lcd_1_val=voltage;   // copy voltage to lcd 1 variable
   lcd_2_val=current;   // copy current to lcd 2 variable
   while(!input(SW4))         // Press once to enter menu
   {  delay_ms(50);            // debouncing
      while(!input(SW4));      //debouncing
      led_on(4); // indication for menu stage 0 ("show parameter" screen)
      show_parameters_mode=TRUE;  //set flag when show parameter screen is shown
      
      // Clear Timer3 flag if interrupt has occured prior to being enabled
      clear_interrupt(INT_TIMER3);
      enable_interrupts(INT_TIMER3);
      // enable timer for timeout monitoring and screen flash in edit screen
      
      set_timer3(9286);    // set timer 3 to generate interrupt on every 90 ms
      lcd_1_val=menu[a];      // hold parameter number for LCD1
      lcd_2_val=value[a];      // hold corresponding parameter value for LCD2
      lcd_out(1,lcd_1_val);   // display lcd 1 value on LCD1   
      lcd_out(2,lcd_2_val);   // display lcd 2 value on LCD2
      menu_timeout_monitor=0;  // reset timeout monitor
      
      /* Select the parameter number to be shown through SW3 and SW2 */
      do
      {                      
         while(!input(SW3))      // Press once to decrement parameter number
         {
            while(!input(SW3));   // halt here until button is released
            delay_ms(50);         // delay to cater for bounce
            menu_timeout_monitor=0;      // reset tiemout monitor
            if (a==0) a=4;      // underflow if a is already 0
            else a--;         // otherwise decrement once in a
         }
         
         while(!input(SW2))      // Press once to increment parameter number
         {
            while(!input(SW2));   
            delay_ms(50);
            menu_timeout_monitor=0;
            if (a==4) a=0;      // overflow if a is already 0
            else a++;         // otherwise increment once in a
         }
      
      lcd_1_val=menu[a];      
      lcd_2_val=value[a];
      lcd_out(1,lcd_1_val);      // Show updated parameter number
      lcd_out(2,lcd_2_val);      // Show corresponding value on other LCD
     
/* Press SW4 to edit the parameter Value; store in EEPROM; send to Master CPU */
      while(!input(SW4))         
         {  
         while(!input(SW4));
            menu_timeout_monitor=0;   // reset timeout monitor
            temp_val=lcd_2_val;      // copy LCD 2 value in temporary variable.
            edit_value(temp_val);
               // pass copy of temporary variable in edit value function
            
            write_eeprom(f(a),low_byte);
               /* store low byte on respective location in EEPROM after
                * returning from edit function */
            
            write_eeprom(f(a)+1,high_byte);
               /* store high byte on respective location in EEPROM after
                * returning from edit function */
         } // while(!input(SW4))   

      if(menu_timeout_monitor==56) break;
       // if no button is pressed for 5.04 seconds then menu timeout will occur
      
      } while(input(SW1));      // Press once to exit from menu
/******************************************************************************/
      disable_interrupts(INT_TIMER3);
      // disable timer 3. there is no need of timeout or flashing on main screen
      
      menu_timeout_monitor=0;      // reset timeout monitor
      show_parameters_mode=FALSE;   // clear flag for show parameters mode    
      led_off(4);
      // led off to indicate an exit from menu stage 0 (show parameters screen)
   }
/******************************************************************************/
   /* get back to main screen and display voltage and current values
    * that have been received in reading[] array from master CPU */
   lcd_out(1,lcd_1_val);
   lcd_out(2,lcd_2_val);
   
   }while(TRUE);
}
/****************************** END OF FILE  **********************************/
