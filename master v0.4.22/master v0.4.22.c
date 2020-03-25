/*******************************************************************************
**    Version : 0.4.22                                                        **
**    Date : 13-Oct-2014                                                      **
**    Author : Muhib Raza (muhibraza@hotmail.com)                             **
**    Project : Programmable Thyristor Controlled Battery Charger             **
**    Designed for : Advance Electronics International Co.                    **
**    Brief : Code for Master CPU PIC18F26K22 (Control Circuit)               **
**                                                                            **
** ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ **
**   Pin assignment :                                                         **
**       PIN_A0 ->  ADC Channel 0 for Voltage Measurement (Analog IN)         **
**       PIN_A1 ->  ADC Channel 1 for Current Measurement (Analog IN)         **
**       PIN_B0 ->  Zero Crossing Interrupt (IN)   [10 ms interrupt]          **
**       PIN_B1 ->  Push Button 1 (IN) [only in calibration Mode]             **
**       PIN_B2 ->  Push Button 2 (IN) [only in calibration Mode]             **
**       PIN_B4 ->  Used for outputting pulse for SCR gate triggering (OUT)   **
**       PIN_C3 ->  Used as SCL for I2C communication                         **
**       PIN_C4 ->  Used as SDA for I2C communication                         **
**                                                                            **
**   Interrupt -> Associated Function :                                       **
**       INT_TIMER5 -> void int_timer_5(void)                                 **
**       INT_EXT    -> void int_isr(void)                                     **
**       INT_EXT1   -> void int_ext_1(void)  [only in calibration Mode]       **
**       INT_EXT2   -> void int_ext_2(void)  [only in calibration Mode]       **
**                                                                            **
**   Function Description :                                                   **
**       [+] acquire_setpoint() - Reads setpoint values from Slave CPU        **
**                                through I2C bus.                            **
**       [+] read_channels() - Reads ADC channels for measurement of          **
**                             voltage and current.                           **
**       [+] maintain_output() - Loads values in timer5 so as to produce      **
**                               desired firing angle.                        **
**       [+] show_parameters_on_lcd() - Sends values of voltage and           **
**                                      current to slave CPU for display      **
**       [#] int_timer_5() - ISR for Timer5 , Generates firing pulse.         **
**       [#] int_isr() - ISR for external interrupt , counts zero crossing    **
**                        and loads timer5 with timer_value.                  **
**       [#] int_ext_1() - ISR for Push button 1 , used for cycling up        **
**                         through preset timer values [only in calibration]  **
**       [#] int_ext_2() - ISR for Push button 2 , used for cycling down      **
**                         through preset timer values [only in calibration]  **
*******************************************************************************/

/* INCLUDES, FUSES AND OTHER PREPROCESSORS -----------------------------------*/
#include <18f26k22.h>
#DEVICE ICD=true ADC=10 HIGH_INTS=TRUE
#fuses MCLR,HSH,NOWDT,NOLVP,PUT,NOBROWNOUT
#use delay(xtal=20000000)
#use i2c(MASTER,SDA=PIN_C4,SCL=PIN_C3,FORCE_HW,fast=450000)
#use fast_io(B)
#bit TMR5ON=getenv("BIT:TMR5ON")

/* DEFINES -------------------------------------------------------------------*/
#define firing_pulse_on    output_low(PIN_B4)
#define firing_pulse_off   output_high(PIN_B4)

/* Factor by which the system output is attenuated and fed back to channel 0  */
#define attenuation_factor   0.1

/* Factor by which millivolts on shunt resistor are amplified and fed back in
 * to channel 1 */
#define ch1_gain 40

/* Value of shunt resistance in the high side power circuit */
#define shunt_resistance 0.075

/* Slope for adc conversion (5/1024) */
#define slope   0.0048828125

/* Number of samples to be taken in every sample train */
#define adcsamples 40

/* Maximum value of timer5 to restrict firing angles below 45 degrees */
#define max_timer_value 63973

/* Minimum value of timer5 to restrict firing angles above 169 degrees */
#define min_timer_value 59694

/* Uncommenting the following #define will make the system enter calibration 
 * mode. Right click the line and click "Toggle Comments" in CCS IDE. */
//!#define calibration_mode

/* GLOBAL VARIABLES ----------------------------------------------------------*/
/* Zero_Crossing flag - Indicates interrupt edge selection.
 * Uses [..]
 * 0 : External Interrupt Edge : low to high
 * 1 : External Interrupt Edge : high to low */
int1 zc_flag=0;

/* <poll> flag used in I2C to check whether address matched or not.
 * Uses [..]
 * 0 : Slave address matched, Communication Successful
 * 1 : communication failed */
int1 poll=1;

/* Flag used to indicate present state of firing pulse (on or off)
 * Uses [..]
 * 0 : Firing Pulse deactivated
 * 1 : Firing Pulse activated */
int1 pulse=0;

/* Flag used to indicate a change in task variable 
 * Uses [..]
 * 0 : Task is not changed during execution of any one
 * 1 : Another task is waiting in queue */
int1 task_changed=0;

/* Flag used to indicate whether setpoints have been acquired from the HMI 
 * Uses [..]
 * 0 : First setpoint of the system has not been acquired yet
 * 1 : Setpoint acquired from HMI */
int1 setpoint_acquired=FALSE;

/* Variable used for selecting which task needs to be performed next 
 * Uses [..]
 * 0 : No task , idle state
 * 1 : Task 1 -> Acquire Setpoints from HMI CPU
 * 2 : Task 2 -> Read ADC channels and maintain output
 * 3 : Task 3 -> Send Parameter Values to HMI CPU for display */
int task=0;

/* Variables used as pointers for array locations  */
int p, r;

/* Union allocates shared space for members. In this code the voltage and
 * current need to be sent through I2C bus 1 byte at a time, but the calculation
 * is being done in long type (which is int16). Now when 16 bit value is needed
 * then the word_value is being accessed, and when 2 values of 8 bit are needed
 * then byte_value[0] and byte_value[1] are being accessed.
 * Variable accessing example :
 * voltage_rx.byte_value[0] for accessing lower byte of received voltage 
 * voltage_rx.byte_value[1] for upper byte of received voltage 
 * voltage_rx.word_value for 16 bit value of received voltage as a whole. */
union i2c
{
   int byte_value[2];
   long word_value;
}voltage_rx, current_rx, voltage_tx, current_tx;

/* Following #defines are for better naming of variables of the union. */
#define voltage_low_byte_rx    voltage_rx.byte_value[0]
#define voltage_high_byte_rx   voltage_rx.byte_value[1]
#define voltage_16_bit_rx      voltage_rx.word_value

#define current_low_byte_rx    current_rx.byte_value[0]
#define current_high_byte_rx   current_rx.byte_value[1]
#define current_16_bit_rx      current_rx.word_value

#define voltage_low_byte_tx    voltage_tx.byte_value[0]
#define voltage_high_byte_tx   voltage_tx.byte_value[1]
#define voltage_16_bit_tx      voltage_tx.word_value

#define current_low_byte_tx    current_tx.byte_value[0]
#define current_high_byte_tx   current_tx.byte_value[1]
#define current_16_bit_tx      current_tx.word_value

/* 16 bit variables used in setpoint function */
long v_set_point_old=0, v_set_point_new, i_set_point_old=0, i_set_point_new;

/* 16 Bit variable used in read_channels() function in minimum value algorithm*/
long min;

/* 16 bit variable used in loading timer 5 */
long timer_value=60067;

/* Commands used for calibration using preset firing angles */
#ifdef calibration_mode
int b=0;
long test_array[]={59733,60067,60501,60935,61369,61803,62237,62671,63105,63539,
                   63973};
#endif

/* Float variables being used to store absolute value of error  between actual
 * and desired output */
float absolute_difference_vo, absolute_difference_io;

/* Float variables used in acquire_setpoint() function */
float set_point_vo, set_point_io;

/* Variables used to store manipulated/meaningful value of Load Voltage (VL)
 * and Load Current (IL) */
float VL, IL;

/* Variables used in read_channels() function for storing average values */
float AD0, AD1;

/* Variables used in sending actual values to HMI for displaying */
float v0=0, i=0;

/* Arrays used to store adc channel 1 and channel 0 values */
long v_1[adcsamples], v_0[adcsamples];

/* Variables used to count how many 10 ms interrupts have occured. Each of the
 * following three variables is used in selection of a specific task */
int count1=0, count2=0, count3=0;

/*============================================================================*/
/***
   * Function     : void acquire_setpoint(void)
   * Parameters   : None  
   * Return value : None
   * Purpose      : Acquires setpoint values for Charging Voltage and 
   *                Max Current Limit from HMI through I2C communication.
***/
/*============================================================================*/
void acquire_setpoint(void)
{      
   i2c_start();       // Start I2C Communication

   /* i2c_write returns zero if device address matched succesfully. The LSB of
    * arguement determines direction of communication.
    * 0 for master to slave, 1 for slave to master */
   poll=i2c_write(0xa1);
      
   /* If device address matched, then do the following */
   if(poll==0)         
   {   
      voltage_low_byte_rx = i2c_read();    //rcv low byte of setpoint voltage
      voltage_high_byte_rx = i2c_read();   //rcv high byte of setpoint voltage
      current_low_byte_rx = i2c_read();    // rcv low byte of setpoint current
      current_high_byte_rx = i2c_read(0);  // rcv high byte of setpoint current                                                         
                                           /* 0 is to indicate last read.*/
        
      v_set_point_new = voltage_16_bit_rx;
      i_set_point_new = current_16_bit_rx;
         
      /* Make flag high to indicate that setpoint has been acquired */
      setpoint_acquired=TRUE;
   }
   i2c_stop();      // stop communication and release I2C bus
      
   /* Check if setpoint has changed with reference to previous. If changed,
    * divide long value by 10 to restore decimal point and store the value */
   if((v_set_point_new-v_set_point_old)!=0)
   {
   v_set_point_old=v_set_point_new;
   set_point_vo=v_set_point_new;
   set_point_vo/=10; 
   }
      
   /* Check if setpoint has changed with reference to previous. If changed,
    * divide long value by 10 to restore decimal point and store the value */
   if((i_set_point_new-i_set_point_old)!=0)
   {
   i_set_point_old=i_set_point_new;
   set_point_io=i_set_point_new;
   set_point_io/=10;
   }
   
   /* Make flag high if there is any change in task variable */
   task_changed=(task!=1) ? TRUE:FALSE;
}

/*============================================================================*/
/***
   * Function      : void read_channels(void)
   * Parameters    : None  
   * Return value  : None
   * Purpose       : Reads ADC channel 0 (voltage measurement) and channel 1
   *                 (current measurement), manipulates and stores in AD0 and 
   *                 AD1 variables.
***/
/*============================================================================*/
void read_channels(void)
{          
   AD0=0;  // reset the variable being used to store 40 samples of channel 0
   AD1=0;  // reset variable used to store channel 1 value         
   
   /* take 40 samples of channel 0 and channel 1 in 10 ms duration. Delay has 
    * been calculated as 10ms/40 = 250 usec. */
   for(p=0; p<adcsamples; p++)
   {                         
      v_0[p]=read_adc();
      SET_ADC_CHANNEL(1);
      delay_us(10);            
      
      v_1[p]=read_adc();
      SET_ADC_CHANNEL(0);
      delay_us(240);
   }     
           
   min=1023;                
   for (r=0; r<adcsamples; r++)
   {
   AD0+=v_0[r];  // add values of all samples whose average is to be calculated
   
   /* Minimum Value Algorithm. This is being implemented because ripple on
    * shunt resistor gets amplified by the op amp. The actual voltage is thus 
    * minimum voltage in the waveform. */
   if(v_1[r]<min) min=v_1[r];
   }
   
   AD0/=adcsamples;   // adc value averaging for channel 0
   AD1=min;   // Store the adc value of shunt voltage in AD1 variable

//!      AD0=AD0-(AD1/ch1_gain);
}

/*============================================================================*/
/***
   * Function     : void maintain_output(void)
   * Parameters   : None  
   * Return value : None
   * Purpose      : Calculates and sets the desired timer_value to make the
   *                output equal to the setpoints.
***/
/*============================================================================*/
void maintain_output(void)
{   
   /* Manipulation of adc value and cancellation of attenuation for
    * finding V Load */
   VL=AD0*slope/attenuation_factor;                                  

   /* Manipulation of adc value and cancellation of op amp gain for
    * finding I Load */   
   IL=(AD1*slope/ch1_gain)*1000/shunt_resistance;
   
   /* Maintain output voltage if current is less then current limit minus 1% */
   if  (IL < (set_point_io*(1-0.01)))                                                                                      
   {
      /* Check if voltage is exceeding setpoint tolerances */      
      if ((VL<(set_point_vo+0.01)) || (VL>(set_point_vo+0.03))) 
      {     
      
      /* Find the absolute value of error between V Load and Setpoint */
      absolute_difference_vo=abs(VL-set_point_vo);

         /* Load large timer_value if the error is large. timer_value is
          * inversely proportional to firing angle and directly proportional
          * to V Load */
         if(absolute_difference_vo>5)                                  
         {
            if((VL-set_point_vo)<0)   timer_value+=25;
            else   timer_value-=25;
         }
           
         else if(absolute_difference_vo>4)   // fine tuning
         {      
            if((VL-set_point_vo)<0)   timer_value+=20;
            else   timer_value-=20;
         }
           
         else if(absolute_difference_vo>3)
         {      
            if((VL-set_point_vo)<0)   timer_value+=15;
            else   timer_value-=15;
         }
           
         else if(absolute_difference_vo>2)
         {      
            if((VL-set_point_vo)<0)   timer_value+=10;
            else   timer_value-=10;
         }
         
         else if(absolute_difference_vo>1)
         {
            if((VL-set_point_vo)<0)   timer_value+=5;
            else   timer_value-=5;
         }
         
         else
         {
            if((VL-set_point_vo)<0)   timer_value+=1;
            else   timer_value-=1;
         }
      }
   }
   
   /* if I Load exceeds the given limit, bring I Load to within 1% of 
    * current limit and maintain it there. */
   else if (IL>(set_point_io*(1+0.01)))      
   {
      absolute_difference_io=abs(IL-set_point_io);  // calculate absolute error

      /* Decrement the timer value (and thus the voltage) if current exceeds */
      if(absolute_difference_io>10)   timer_value-=10;   
      else   timer_value-=1;
   }
   
   /* Make flag high if there is any change in task variable */
   task_changed=(task!=2) ? TRUE:FALSE;        
}

/*============================================================================*/
/***
   * Function      : void show_parameters_on_lcd(void)
   * Parameters    : None  
   * Return value  : None
   * Purpose       : Sends values of measured voltage and current to the 
   *                 HMI CPU for displaying.
***/
/*============================================================================*/
void show_parameters_on_lcd(void)
{  
  /** If in calibration mode, user should see the voltage on the HMI as a 
    * value with 2 decimal places. This is to achieve a greater accuracy in 
    * calibration. Once calibration mode disabled, the voltage on HMI will 
    * display normally, and will contain only one decimal place. */
   
   #ifdef calibration_mode
   v0=AD0*slope/attenuation_factor;
   i=(AD1*slope/ch1_gain)*1000/shunt_resistance;
   v0=v0*100;
   i=i*10;
   
   #else    // Normal Operation
   
   /* Shift decimal point to right by one digit to avoid floating point
    * values. The decimal will be restored in HMI. */
   v0=VL*10; 
   
   /* Shift decimal point to right by one digit to avoid floating point 
    * values. The decimal will be restored in HMI. */
   i=IL*10;
   
   #endif   
      
   /* Store the values in int16 variable instead of float value of 32 bit */
   voltage_16_bit_tx=v0;
   current_16_bit_tx=i;
   
   i2c_start();               // start i2c communication
   poll=i2c_write(0xa0);      // LSB is zero so direction is Master to Slave
   if (poll==0)
   {
      i2c_write(voltage_low_byte_tx);      // Send low byte of voltage
      i2c_write(voltage_high_byte_tx);   // Send high byte of voltage          
      i2c_write(current_low_byte_tx);      // Send low byte of current                                        
      i2c_write(current_high_byte_tx);   // Send high byte of current                                           
   }
   i2c_stop();                // stop i2c communication
   
   /* Make flag high if there is any change in task variable */
   task_changed=(task!=3) ? TRUE:FALSE;
}

/*============================================================================*/
/***
   * Function      : void int_timer_5(void)
   * Parameters    : None  
   * Return value  : None
   * Purpose       : Activates firing pulse when certain time has passed 
   *                 after zero crossing and deactivates the pulse 
   *                 after 500 usec
***/
/*============================================================================*/
#INT_TIMER5
void int_timer_5(void)
{  
   /* Checks pulse status and does the following....
    * if pulse is off : activate pulse, set pulse flag and set timer5 to 
    *                   overflow after 500 usec.
    * if pulse is on  : deactivate pulse , clear pulse flag and stop timer5 */
   
   if(!pulse) { firing_pulse_on; pulse=TRUE; set_timer5(65223); }
   else  { firing_pulse_off; pulse=FALSE;  TMR5ON=0;}
}

/*============================================================================*/
/***
   * Function     : void int_isr(void)
   * Parameters   : None  
   * Return value : None
   * Purpose      : Serves for the following purposes:
   *                [+] Changes interrupt edge on every interrupt.
   *                [+] Monitors timer_value and prevents it from getting out
   *                    of limits.
   *                [+] Loads timer 5 with the timer_value already calculated
   *                    in maintain_output() function.
   *                [+] Increments count1, count2, and count3 variables.
   *                [+] Sets a value in task variable based on interrupt counts
***/
/*============================================================================*/
#INT_EXT   HIGH
void int_isr(void) // Interrupt routine for zero crossing; occurs every 10 msec.
{
   /* count1 variable used for calling acquire_setpoint() function. 
    * count2 variable used for calling read_channels() 
    * and maintain_output() function.
    * count3 variable used for calling show_parameters_on_lcd() function. */
   count1++;
   count2++;          
   count3++;           
   
   /* task=1 when 1 sec passed (100 * 10ms = 1 sec) 
    * task=3 when 0.5 sec passed (50 * 10ms = 0.5 sec)
    * task=2 when 20 msec passed (2 * 10ms = 20 msec) */
   
   /* In case of collision, task priority would be highest for
    * acquire_setpoint(), normal for show_parameters_on_lcd and lowest for 
    * read_channels() and maintain_output().
    * The "if else" and "greater than or equal to" sign make sure that
    * none of the task would be missed. */
   if (count1>=100)  { task=1; count1=0; }
   else if (count3>=50) { task=3; count3=0; } 
   else if (count2>=2)  { task=2; count2=0; }
  
   /* if zero crossing flag is 0 then change interrupt to rising edge
    * if zero crossing flag is 1 then change interrupt to falling edge */
   if (!zc_flag) { ext_int_edge(0,H_TO_L); zc_flag=1; }
   else { ext_int_edge(0,L_TO_H); zc_flag=0; }
   
   /* Restrict timer value to defined minimum if it is less than lower limit
    *  else, restrict it to defined maximum if it is greater than upper limit */
   if (timer_value<min_timer_value) timer_value=min_timer_value;
   else if (timer_value>max_timer_value) timer_value=max_timer_value; 

   set_timer5(timer_value);   // set timer5 with predetermined value
   TMR5ON=1;      // Start Timer 5
}

/*============================================================================*/
/***
   * Function     : void int_ext_1(void) ; void int_ext_2(void)
   * Parameters   : None  
   * Return value : None
   * Purpose      : The following two functions are only compiled when
   *                calibration_mode is defined. The push buttons in the 
   *                control circuit are used to select a fixed timer_value by
   *                cycling through an array of 11 values. Once a fixed firing 
   *                angle is selected, the variable resistors in the circuit 
   *                could be used to match HMI values with multimeter values.
***/
/*============================================================================*/
#ifdef calibration_mode
#INT_EXT1
void int_ext_1(void)
{
   if (b==10) b=0;
   else b++;  //increment once in b variable when push button 1 is pressed
   
   /* Assign one of the predefined timer values from the array */
   timer_value=test_array[b];
}

#INT_EXT2
void int_ext_2(void)
{
   if (b==0) b=10;
   else b--;  //decrement once in b variable when push button 2 is pressed
   
   /* Assign one of the predefined timer values from the array */
   timer_value=test_array[b];
}
#endif

/*============================================================================*/
/***
   * Function     : void main(void)
   * Parameters   : None  
   * Return value : None
   * Purpose      : Serves for the following purposes:
   *                [+] Initializes Interrupts, Timer5 and ADC channel 0 and 1
   *                [+] Calls one of the 3 subroutines based on their timing.
   *                [+] Loops forever
***/
/*============================================================================*/
void main(void)
{  
   /* Initialization of Port B. PortB is being configured in FAST_IO mode 
    * therefore specifying data direction is must. PIN_B4 is being configured as 
    * output using output_drive() built-in CCS function. Directions of other 
    * pins of PORTB do not need to be defined manually since they are already
    * inputs on Power-On Reset. All other ports are configured in STANDARD_IO 
    * mode by default. (meaning that compiler configures data direction on
    * every IO operation). Refer to list file (.lst) for better understanding.*/
   firing_pulse_off;
   output_drive(PIN_B4);
   
   clear_interrupt(INT_TIMER5);     // clear flag
   enable_interrupts(INT_TIMER5);   // enable timer5 interrupt
   setup_timer_5(T5_INTERNAL | T5_DIV_BY_8);   // 16-bit, 1.6 usec for one tick
   TMR5ON=0;                        // Clear bit 0 of T5CON register
     
   /* Following 4 commands are only compiled if calibration_mode is DEFINED */
   #ifdef calibration_mode
   clear_interrupt(INT_EXT1);                          
   clear_interrupt(INT_EXT2);                           
   enable_interrupts(INT_EXT1_L2H);
   enable_interrupts(INT_EXT2_L2H);
   #endif
   
   /* Clear external interrupt flag if the interrupt event has occured prior to
    * being enabled */
   clear_interrupt(INT_EXT);
   
   /* Enable external interrupt with low to high edge triggering */
   enable_interrupts(INT_EXT_L2H);
   
   enable_interrupts(GLOBAL);       // enable global interrupt
   
   setup_adc (ADC_CLOCK_INTERNAL);   // use internal clock for adc conversion
   SETUP_ADC_PORTS(sAN0 | sAN1);     // using AN0 and AN1 as analog channels                              
   SET_ADC_CHANNEL(0);  // select channel 0 for next read_adc() function call
   
   /* 10 usec delay for charging internal capacitor of sample and hold circuit
    * in microcontroller. Required every time the channel is changed. */
   delay_us(10);
   
   /* Function called for reading values on channel 0 and channel 1 */
   read_channels(); 
   
   /* There are 3 main tasks. Each is being executed on fixed time intervals. */
   do
   {
      /* Selection of task is done through task variable in int_isr() routine */
      switch(task)
      {
         case 1:   
         { 
            /* If task is 1 then acquire set point from HMI */
            acquire_setpoint(); 
            
            /* Reset task variable to 0 if no change has occured during
             * acquire_setpoint() call */
            if(task_changed==FALSE) task=0;
            break;
         }
         
         case 2:
         {
            /* If task is 2 then read adc channels. */
            read_channels();
            
            /* if calibration mode is not defined, then maintain_output() will
             * be active. When called, it will minimize the error after 
             * acquiring the setpoints. */
            #ifndef calibration_mode
               if(setpoint_acquired==TRUE) maintain_output();
               if(task_changed==FALSE) task=0;   // reset task variable
            #endif 
            break;
         }
         
         case 3:
         {
            /*  If task is 3 then send voltage and current values to HMI */
            show_parameters_on_lcd();
            
            if(task_changed==FALSE) task=0;   // reset task variable
            break;
         }
      }
   }while(TRUE);
}
/****************************** END OF FILE  **********************************/
