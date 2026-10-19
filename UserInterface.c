#include <avr/io.h>        // This defers to avr/io.h for GCC
#include <avr/wdt.h>
#include "avr\eeprom.h"
#include <stdio.h>        
#include <stdlib.h>        
#include <string.h>        
#include <math.h>        
#include "os_api.h"
#include "locked_printf.h"
#include "UserInterface.h"
#include "SerialInterface.h"
#include "HardwareInfo.h"
#include "DeviceIO.h"
#include "ks0xxx.h"
#include "Watchdog.h"

//#include "arial_bold_14.h"  //Arial_Bold_14
//#include "Bitmap.h"       //IMAGE
//#include "corsiva_12.h"      //Corsiva_12
//#include "font12X16.h"    //Font12x16
//#include "Font12X24OnlyDigitsAndDot.h"    //Font12x16
//#include "Font12X16OnlyDigitsAndDot.h"    //Font12x16
//#include "Font_Arial_Nerrow.h"    //Font12x16
//#include "Font_Vrinda.h"    //Font12x16
//#include "font5X7.h"      //Font5x7
//#include "Font_Arial_Narrow_Mid.h"
//#include "Font_arial_Narrow_14_Full.h"
//#include "Font_arial_Narrow_Bold_45.h"
//#include "font_arial_narrow_bold_72.h"
#include "Font_Courier_14_Bold.h"
#include "FontVerdana10x24.h"    //Font12x16
#include "font_arial_narrow_bold_72.h"
#include "font_arial_narrow_bold_58.h"
#include "ArialNarrow88.h"
#include "BodoniMTBlack88.h"
#include "BodoniMTBlack50.h"
#include "BodoniMTBlack20.h"
#include "BodoniMTBlack15.h"
#include "BodoniMTBold15.h"


//#include "font_arial_narrow_bold_45.h"
#include "font6x8.h"      //Font6x8
#include "Arial18Bold.h"      //

#include "error.h"
#include "eepromAdr.h" 
#include "rtc.h" 

#define SOFTWARE_VERSION        306

#define EEPROM_WRITE_TIMEOUT	30
#define DP_AUTOCAL_HOLD_TICKS	60		// SETUP+DOWN hold time for DP auto-zero: 60 x 50 ms = 3 s

#define TRUE 1
#define FALSE 0

#define CENTER 0
#define LEFT 1

/* Default values */
////
#define D_TIME_HOURS                          12               
#define D_TIME_MINUTES                        0                
#define D_TIME_SECONDS                        0                
#define D_DATE_YEAR                           24                
#define D_DATE_MONTH                          7                
#define D_DATE_DAY                            1   
             
#define D_DP1_OUT_LOW_COUNT					0          
#define D_DP1_OUT_HIGH_COUNT				3604          
#define D_DP2_OUT_LOW_COUNT					0              
#define D_DP2_OUT_HIGH_COUNT				3604     
#define D_DP3_OUT_LOW_COUNT					0
#define D_DP3_OUT_HIGH_COUNT				3604        
#define D_TEMP_OUT_LOW_COUNT                0
#define D_TEMP_OUT_HIGH_COUNT               3604
#define D_RH_OUT_LOW_COUNT                  0
#define D_RH_OUT_HIGH_COUNT                 3604
           
#define D_DP1_UNIT						PRESSURE_PA     
#define D_DP2_UNIT						PRESSURE_PA     
#define D_DP3_UNIT						PRESSURE_PA  
#define D_TEMPERATURE_UNIT              TEMPERATURE_DEG_C
#define D_HUMIDITY_UNIT                 HUMIDITY_RH      
#define D_LOGGING_INTERVAL              3600             
#define D_DEVICE_ID                     1
#define D_DP1_ZERO_RNG                  0                
#define D_DP2_ZERO_RNG                  0  
#define D_DP3_ZERO_RNG                  0              
#define D_DP1_ZERO_ADJ                  0                
#define D_DP2_ZERO_ADJ                  0   
#define D_DP3_ZERO_ADJ                  0              
#define D_TEMP_ZERO_ADJ                 0                
#define D_RH_ZERO_ADJ                   0  
#define D_DP1_RANGE						10000
#define D_DP2_RANGE						10000
#define D_DP3_RANGE						10000
              
#define D_BROADCAST_INTERVAL            120   
#define D_BROADCAST_PORT	            2           
#define D_DISPLAY_INTERVAL              1
#define D_SERIAL_BAUD_RATE_VAL          SERIAL_BAUD_57600
#define D_SERIAL_DATA_STOP_PARITY_VAL   ((3<<SERIAL_DATA_BITS_SHIFT) | (0<<SERIAL_STOP_BIT_SHIFT) | (SERIAL_PARITY_NONE << SERIAL_PARITY_SHIFT))//3    // BIT 0-1 = Data bits, BIT 2 = STOP bits, BIT 3-4 = PARITY :: 8 bit, 1 stop, no parity
#define D_BUZZER_ON_TIME                1    
#define D_BUZZER_OFF_TIME               5   
#define D_BUZZER_DISABLE_TIME           0xFFFF

#define D_DP1_PID_STARTUP_PERCENT        0
#define D_DP1_PID_KP                     10
#define D_DP1_PID_TI                     10
#define D_DP1_PID_TD                     1
#define D_DP1_OUT_POLARITY				 1

#define D_DP2_PID_STARTUP_PERCENT       0
#define D_DP2_PID_KP                    10
#define D_DP2_PID_TI                    10
#define D_DP2_PID_TD                    1
#define D_DP2_OUT_POLARITY				1

#define D_TEMP_PID_STARTUP_PERCENT            0
#define D_TEMP_PID_KP                         10
#define D_TEMP_PID_TI                         10
#define D_TEMP_PID_TD                         1
#define D_TEMP_OUT_POLARITY				      1

#define D_RH_PID_STARTUP_PERCENT              0
#define D_RH_PID_KP                           10
#define D_RH_PID_TI                           10
#define D_RH_PID_TD                           1
#define D_RH_OUT_POLARITY				      1

#define D_TEMP_HIGH_PID_RH_CASCADE_FACTOR     0
#define D_TEMP_LOW_PID_RH_CASCADE_FACTOR      0
#define D_TEMP_HIGH_PID_CASCADE_RH_STATE	  0
#define D_TEMP_LOW_PID_CASCADE_RH_STATE       0
#define D_RH_HIGH_PID_TEMP_CASCADE_FACTOR     0
#define D_RH_LOW_PID_TEMP_CASCADE_FACTOR      0
#define D_RH_HIGH_PID_CASCADE_TEMP_STATE	  0
#define D_RH_LOW_PID_CASCADE_TEMP_STATE       0
#define D_TEMP_HIGH_PID_RH_ADJUST             0
#define D_TEMP_LOW_PID_RH_ADJUST              0
#define D_RH_HIGH_PID_TEMP_ADJUST             0
#define D_RH_LOW_PID_TEMP_ADJUST              0




/* MIN values */
////
#define MIN_TIME_HOURS                        0    
#define MIN_TIME_MINUTES                      0    
#define MIN_TIME_SECONDS                      0    
#define MIN_DATE_YEAR                         8    
#define MIN_DATE_MONTH                        1    
#define MIN_DATE_DAY                          1    
#define MIN_DP1_OUT_LOW_COUNT            0    
#define MIN_DP2_OUT_LOW_COUNT           0    
#define MIN_TEMP_OUT_LOW_COUNT                0    
#define MIN_RH_OUT_LOW_COUNT                  0    
#define MIN_LOGGING_INTERVAL                  60   
#define MIN_DEVICE_ID                         0    
#define MIN_BROADCAST_INTERVAL                3    
#define MIN_BROADCAST_PORT					  0
#define MIN_DISPLAY_INTERVAL                  1             
#define MIN_SERIAL_BAUD_RATE_VAL              0
#define MIN_SERIAL_DATA_STOP_PARITY_VAL       0
#define MIN_BUZZER_ON_TIME                    0    
#define MIN_BUZZER_OFF_TIME                   0    
#define MIN_BUZZER_DISABLE_TIME               (-1)
#define MIN_DP1_PID_STARTUP_PERCENT           0
#define MIN_DP1_PID_KP                   1
#define MIN_DP1_PID_TI                   1
#define MIN_DP1_PID_TD                   0
#define MIN_DP2_PID_STARTUP_PERCENT           0
#define MIN_DP2_PID_KP                  1
#define MIN_DP2_PID_TI                  1
#define MIN_DP2_PID_TD                  0
#define MIN_TEMP_PID_STARTUP_PERCENT          0
#define MIN_TEMP_PID_KP                       1
#define MIN_TEMP_PID_TI                       1
#define MIN_TEMP_PID_TD                       0
#define MIN_RH_PID_STARTUP_PERCENT            0
#define MIN_RH_PID_KP                         1
#define MIN_RH_PID_TI                         1
#define MIN_RH_PID_TD                         0
#define MIN_TEMP_HIGH_PID_RH_CASCADE_FACTOR   (-1000)
#define MIN_TEMP_LOW_PID_RH_CASCADE_FACTOR    (-1000)
#define MIN_TEMP_HIGH_PID_CASCADE_RH_STATE	  (-1)
#define MIN_TEMP_LOW_PID_CASCADE_RH_STATE     (-1)
#define MIN_RH_HIGH_PID_TEMP_CASCADE_FACTOR   (-1000)
#define MIN_RH_LOW_PID_TEMP_CASCADE_FACTOR    (-1000)
#define MIN_RH_HIGH_PID_CASCADE_TEMP_STATE	  (-1)
#define MIN_RH_LOW_PID_CASCADE_TEMP_STATE     (-1)
#define MIN_TEMP_HIGH_PID_RH_ADJUST           (-10000)
#define MIN_TEMP_LOW_PID_RH_ADJUST            (-10000)
#define MIN_RH_HIGH_PID_TEMP_ADJUST           (-10000)
#define MIN_RH_LOW_PID_TEMP_ADJUST            (-10000)



/* MAX values */
////
#define MAX_TIME_HOURS                        23
#define MAX_TIME_MINUTES                      59
#define MAX_TIME_SECONDS                      59
#define MAX_DATE_YEAR                         99
#define MAX_DATE_MONTH                        12
#define MAX_DATE_DAY                          31
#define MAX_DP1_OUT_HIGH_COUNT				  4095
#define MAX_DP2_OUT_HIGH_COUNT				  4095
#define MAX_TEMP_OUT_HIGH_COUNT               4095
#define MAX_RH_OUT_HIGH_COUNT                 4095
#define MAX_LOGGING_INTERVAL                  32767
#define MAX_DEVICE_ID                         250
#define MAX_BROADCAST_INTERVAL                32767
#define MAX_BROADCAST_PORT		              2
#define MAX_DISPLAY_INTERVAL                  30
#define MAX_SERIAL_BAUD_RATE_VAL              8
#define MAX_SERIAL_DATA_STOP_PARITY_VAL       23
#define MAX_BUZZER_ON_TIME                    60    
#define MAX_BUZZER_OFF_TIME                   3600    
#define MAX_BUZZER_DISABLE_TIME               32767  
#define MAX_DP1_PID_STARTUP_PERCENT           1000  
#define MAX_DP1_PID_KP                   32767
#define MAX_DP1_PID_TI                   32767
#define MAX_DP1_PID_TD                   32767
#define MAX_DP2_PID_STARTUP_PERCENT           1000
#define MAX_DP2_PID_KP                  32767
#define MAX_DP2_PID_TI                  32767
#define MAX_DP2_PID_TD                  32767
#define MAX_TEMP_PID_STARTUP_PERCENT          1000
#define MAX_TEMP_PID_KP                       32767
#define MAX_TEMP_PID_TI                       32767
#define MAX_TEMP_PID_TD                       32767
#define MAX_RH_PID_STARTUP_PERCENT            1000
#define MAX_RH_PID_KP                         32767
#define MAX_RH_PID_TI                         32767
#define MAX_RH_PID_TD                         32767
#define MAX_TEMP_HIGH_PID_RH_CASCADE_FACTOR   1000
#define MAX_TEMP_LOW_PID_RH_CASCADE_FACTOR    1000
#define MAX_TEMP_HIGH_PID_CASCADE_RH_STATE	  1
#define MAX_TEMP_LOW_PID_CASCADE_RH_STATE     1
#define MAX_RH_HIGH_PID_TEMP_CASCADE_FACTOR   1000
#define MAX_RH_LOW_PID_TEMP_CASCADE_FACTOR    1000
#define MAX_RH_HIGH_PID_CASCADE_TEMP_STATE	  1
#define MAX_RH_LOW_PID_CASCADE_TEMP_STATE     1
#define MAX_TEMP_HIGH_PID_RH_ADJUST           (10000)
#define MAX_TEMP_LOW_PID_RH_ADJUST            (10000)
#define MAX_RH_HIGH_PID_TEMP_ADJUST           (10000)
#define MAX_RH_LOW_PID_TEMP_ADJUST            (10000)



#define BLOCKED         0
#define SENSOR_VALUES   1
#define EDITOR          2
#define SYSTEM_LOG      3

#define NULL ((void *)0)

#define PARA_READ_VALUE_ONLY  0
#define PARA_READ             1
#define PARA_WRITE            2
#define PARA_DEFAULT          3
#define PARA_WRITE_VALUE_ONLY 4

#define CONFIG_PASSWORD           1965
#define MASTER_CONFIG_PASSWORD    31//20217



#define systemErrorFont    Font6x8 
#define smallFont          Font6x8 
#define largeFont          DispFont 
//#define largeFont          Font12X24OnlyDigitsAndDot 
                            
                                        
#define BUZZER_DIR_OP		PORTH_DIRSET = PIN0_bm
#define BUZZER_ON 			PORTH_OUTSET = PIN0_bm
#define BUZZER_OFF			PORTH_OUTCLR = PIN0_bm

#define SETDFLTSW_HIGH		PORTA_OUTSET = PIN6_bm
#define SETDFLTSW_DIR_IN	PORTA_DIRCLR = PIN6_bm
#define SETDFLTSW_IN		(PORTA_IN & PIN6_bm)

#define KEYPAD_PORT        	PORTD_OUT
#define KEYPAD_PORT_DIR    	PORTD_DIR
#define KEYPAD_PORT_IN     	PORTD_IN

#define MODE_SEL_KEY       (PIN4_bm)   
#define SETUP_KEY          (PIN5_bm)   
#define UP_KEY             (PIN6_bm)   
#define DOWN_KEY           (PIN7_bm)   
#define KEYS_EDITOR_MASK   (SETUP_KEY | UP_KEY | DOWN_KEY) 
#define KEYS_ALL_MASK      (KEYS_EDITOR_MASK | MODE_SEL_KEY) 

#define KEYBOARD_IDLE_TIME       60000u
#define KEYBOARD_LOOP_TIME       50
#define KEY_AUTO_CHG_TIME        300
#define KEY_AUTO_CHG_FAST_TIME   3000
#define KEY_AUTO_FAST_STEP       10

#define PARA_SETUP_Y_LOC         4
#define PARA_INFO_Y_LOC          48//20
#define PARA_VAL_Y_LOC           72//32

#define SCREEN_START_X_VAL    1
#define SCREEN_START_Y_VAL    0
#define DATA_Y_OFFSET    (32-SCREEN_START_Y_VAL)//(40-SCREEN_START_Y_VAL)//(10-SCREEN_START_Y_VAL)
#define UNIT_Y_OFFSET    (46-SCREEN_START_Y_VAL)
#define SYSERR_Y_OFFSET  (136-SCREEN_START_Y_VAL)//(55-SCREEN_START_Y_VAL)

//#define DP1_SENSOR_RANGE_PSI        (GetParameterValue(DP1_RANGE)/6895.7)  // Pa to PSI conversion
//#define DP2_SENSOR_RANGE_PSI        (GetParameterValue(DP2_RANGE)/6895.7)  // Pa to PSI conversion
//#define DP3_SENSOR_RANGE_PSI        (GetParameterValue(DP3_RANGE)/6895.7)  // Pa to PSI conversion

//#define DP1_SENSOR_RANGE_MILLIBAR   (DP1_SENSOR_RANGE_PSI*68.9476)    // 2 digit
//#define DP2_SENSOR_RANGE_MILLIBAR  (DP2_SENSOR_RANGE_PSI*68.9476)   // 1 digit
//#define DP1_SENSOR_RANGE_PA         (DP1_SENSOR_RANGE_PSI*6894.76)    // 3 digit
//#define DP2_SENSOR_RANGE_PA        (DP2_SENSOR_RANGE_PSI*6894.76)   // 3 digit
//#define DP1_SENSOR_RANGE_H2O_MM     (DP1_SENSOR_RANGE_PSI*703.069578295612)    // 3 digit
//#define DP2_SENSOR_RANGE_H2O_MM    (DP2_SENSOR_RANGE_PSI*703.069578295612)   // 2 digit
//#define DP1_SENSOR_RANGE_H2O_INCH   (DP1_SENSOR_RANGE_PSI*27.7076)   // 1 Digit
//#define DP2_SENSOR_RANGE_H2O_INCH  (DP2_SENSOR_RANGE_PSI*27.7076)  // 1 Digit
//#define DP1_SENSOR_RANGE_HG_INCH    (DP1_SENSOR_RANGE_PSI*2.03602)     // 0 digit
//#define DP2_SENSOR_RANGE_HG_INCH   (DP2_SENSOR_RANGE_PSI*2.03602)    // 0 digit
//#define DP1_SENSOR_RANGE_PSF        (DP1_SENSOR_RANGE_PSI*144.0)     // 2 digit
//#define DP2_SENSOR_RANGE_PSF       (DP2_SENSOR_RANGE_PSI*144.0)    // 2 digit


#define D_DP1_UPPER_ALARM_OFF_LIMIT              d_pres_abs_upper_alarm_off_limit()   
#define D_DP1_UPPER_ALARM_ON_LIMIT               d_pres_abs_upper_alarm_on_limit()    
#define D_DP1_LOWER_ALARM_OFF_LIMIT              d_pres_abs_lower_alarm_off_limit()   
#define D_DP1_LOWER_ALARM_ON_LIMIT               d_pres_abs_lower_alarm_on_limit()    
#define D_DP2_UPPER_ALARM_OFF_LIMIT             d_pres_diff_upper_alarm_off_limit()  
#define D_DP2_UPPER_ALARM_ON_LIMIT              d_pres_diff_upper_alarm_on_limit()   
#define D_DP2_LOWER_ALARM_OFF_LIMIT             d_pres_diff_lower_alarm_off_limit()  
#define D_DP2_LOWER_ALARM_ON_LIMIT              d_pres_diff_lower_alarm_on_limit()   
#define D_DP3_UPPER_ALARM_OFF_LIMIT            d_pres_diff3_upper_alarm_off_limit()
#define D_DP3_UPPER_ALARM_ON_LIMIT             d_pres_diff3_upper_alarm_on_limit()
#define D_DP3_LOWER_ALARM_OFF_LIMIT            d_pres_diff3_lower_alarm_off_limit()
#define D_DP3_LOWER_ALARM_ON_LIMIT             d_pres_diff3_lower_alarm_on_limit()
#define D_TEMP_UPPER_ALARM_OFF_LIMIT                  d_temp_upper_alarm_off_limit()       
#define D_TEMP_UPPER_ALARM_ON_LIMIT                   d_temp_upper_alarm_on_limit()        
#define D_TEMP_LOWER_ALARM_OFF_LIMIT                  d_temp_lower_alarm_off_limit()       
#define D_TEMP_LOWER_ALARM_ON_LIMIT                   d_temp_lower_alarm_on_limit()        
#define D_RH_UPPER_ALARM_OFF_LIMIT                    d_rh_upper_alarm_off_limit()         
#define D_RH_UPPER_ALARM_ON_LIMIT                     d_rh_upper_alarm_on_limit()          
#define D_RH_LOWER_ALARM_OFF_LIMIT                    d_rh_lower_alarm_off_limit()         
#define D_RH_LOWER_ALARM_ON_LIMIT                     d_rh_lower_alarm_on_limit()  

#define D_PART_MC_UPPER_ALARM_OFF_LIMIT               d_part_mc_upper_alarm_off_limit()
#define D_PART_MC_UPPER_ALARM_ON_LIMIT                d_part_mc_upper_alarm_on_limit()
#define D_PART_VOC_UPPER_ALARM_OFF_LIMIT              d_part_voc_upper_alarm_off_limit()
#define D_PART_VOC_UPPER_ALARM_ON_LIMIT               d_part_voc_upper_alarm_on_limit()
#define D_PART_NOX_UPPER_ALARM_OFF_LIMIT              d_part_nox_upper_alarm_off_limit()
#define D_PART_NOX_UPPER_ALARM_ON_LIMIT               d_part_nox_upper_alarm_on_limit()
        
#define D_DP1_OUT_LOW_PRES                       d_pres_abs_out_low_pres()            
#define D_DP1_OUT_HIGH_PRES                      d_pres_abs_out_high_pres()           
#define D_DP2_OUT_LOW_PRES                      d_pres_diff_out_low_pres()           
#define D_DP2_OUT_HIGH_PRES                     d_pres_diff_out_high_pres()          
#define D_TEMP_OUT_LOW_TEMP                           d_temp_out_low_temp()                
#define D_TEMP_OUT_HIGH_TEMP                          d_temp_out_high_temp()               
#define D_RH_OUT_LOW_RH                               d_rh_out_low_rh()                    
#define D_RH_OUT_HIGH_RH                              d_rh_out_high_rh()                   
#define D_DP1_PID_SET_VALUE							  d_pres_abs_pid_setpoint_limit()
#define D_DP2_PID_SET_VALUE							  d_pres_diff_pid_setpoint_limit()
//#define D_TEMP_PID_SET_VALUE                          ((D_TEMP_UPPER_ALARM_OFF_LIMIT + D_TEMP_UPPER_ALARM_ON_LIMIT)/2)
//#define D_RH_PID_SET_VALUE                            ((D_RH_UPPER_ALARM_OFF_LIMIT + D_RH_UPPER_ALARM_ON_LIMIT)/2)

//#define D_DP1_PID_SET_VALUE                      (0)
//#define D_DP2_PID_SET_VALUE                      (0)
#define D_TEMP_PID_SET_VALUE                     (2600)
#define D_RH_PID_SET_VALUE                       (5500)

#define D_DP1_AREA_LEN                           (600) 
#define D_DP2_AREA_LEN                           (600) 
#define D_DP3_AREA_LEN                           (600) 
#define D_DP1_AREA_WID                           (600)
#define D_DP2_AREA_WID                           (600)
#define D_DP3_AREA_WID                           (600)
#define D_DP1_AREA_RAD                           (300)
#define D_DP2_AREA_RAD                           (300)
#define D_DP3_AREA_RAD                           (300)

#define MIN_DP1_UPPER_ALARM_OFF_LIMIT            min_pres_abs_upper_alarm_off_limit() 
#define MIN_DP1_LOWER_ALARM_ON_LIMIT             min_pres_abs_lower_alarm_on_limit()  
#define MIN_DP2_UPPER_ALARM_OFF_LIMIT           min_pres_diff_upper_alarm_off_limit()
#define MIN_DP2_LOWER_ALARM_ON_LIMIT            min_pres_diff_lower_alarm_on_limit() 
#define MIN_DP3_UPPER_ALARM_OFF_LIMIT          min_pres_diff3_upper_alarm_off_limit()
#define MIN_DP3_LOWER_ALARM_ON_LIMIT           min_pres_diff3_lower_alarm_on_limit()
#define MIN_TEMP_UPPER_ALARM_OFF_LIMIT                min_temp_upper_alarm_off_limit()     
#define MIN_TEMP_LOWER_ALARM_ON_LIMIT                 min_temp_lower_alarm_on_limit()      
#define MIN_RH_UPPER_ALARM_OFF_LIMIT                  min_rh_upper_alarm_off_limit()       
#define MIN_RH_LOWER_ALARM_ON_LIMIT                   min_rh_lower_alarm_on_limit()  
#define MIN_PART_MC_UPPER_ALARM_OFF_LIMIT             min_part_mc_upper_alarm_off_limit()
#define MIN_PART_VOC_UPPER_ALARM_OFF_LIMIT            min_part_voc_upper_alarm_off_limit()
#define MIN_PART_NOX_UPPER_ALARM_OFF_LIMIT            min_part_nox_upper_alarm_off_limit()
      
	  
#define MIN_DP1_RANGE                         0    	  
#define MIN_DP2_RANGE                         0    	  
#define MIN_DP3_RANGE                         0    	  
#define MIN_DP1_ZERO_RNG                         0              
#define MIN_DP2_ZERO_RNG                        0         
#define MIN_DP3_ZERO_RNG                       0        
#define MIN_DP1_ZERO_ADJ                         min_abs_pres_zero_adj()              
#define MIN_DP2_ZERO_ADJ                        min_diff_pres_zero_adj()  
#define MIN_DP3_ZERO_ADJ                       min_diff_pres3_zero_adj()             
#define MIN_TEMP_ZERO_ADJ                             min_temp_zero_adj()                  
#define MIN_RH_ZERO_ADJ                               min_rh_zero_adj()                    
#define MIN_DP1_OUT_LOW_PRES                     min_pres_abs_out_low_pres()          
#define MIN_DP2_OUT_LOW_PRES                    min_pres_diff_out_low_pres()         
#define MIN_TEMP_OUT_LOW_TEMP                         min_temp_out_low_temp()              
#define MIN_RH_OUT_LOW_RH                             min_rh_out_low_rh()                  
#define MIN_DP1_PID_SET_VALUE                    MIN_DP1_UPPER_ALARM_OFF_LIMIT
#define MIN_DP2_PID_SET_VALUE                   MIN_DP2_UPPER_ALARM_OFF_LIMIT
#define MIN_TEMP_PID_SET_VALUE                        MIN_TEMP_UPPER_ALARM_OFF_LIMIT
#define MIN_RH_PID_SET_VALUE                          MIN_RH_UPPER_ALARM_OFF_LIMIT
#define MIN_DP1_AREA                             (1) 
#define MIN_DP2_AREA                            (1) 
#define MIN_DP3_AREA                            (1) 

#define MAX_DP1_UPPER_ALARM_ON_LIMIT             max_pres_abs_upper_alarm_on_limit()  
#define MAX_DP1_LOWER_ALARM_OFF_LIMIT            max_pres_abs_lower_alarm_off_limit() 
#define MAX_DP2_UPPER_ALARM_ON_LIMIT            max_pres_diff_upper_alarm_on_limit() 
#define MAX_DP2_LOWER_ALARM_OFF_LIMIT           max_pres_diff_lower_alarm_off_limit()
#define MAX_DP3_UPPER_ALARM_ON_LIMIT           max_pres_diff3_upper_alarm_on_limit()
#define MAX_DP3_LOWER_ALARM_OFF_LIMIT          max_pres_diff3_lower_alarm_off_limit()
#define MAX_TEMP_UPPER_ALARM_ON_LIMIT                 max_temp_upper_alarm_on_limit()      
#define MAX_TEMP_LOWER_ALARM_OFF_LIMIT                max_temp_lower_alarm_off_limit()     
#define MAX_RH_UPPER_ALARM_ON_LIMIT                   max_rh_upper_alarm_on_limit()        
#define MAX_RH_LOWER_ALARM_OFF_LIMIT                  max_rh_lower_alarm_off_limit()

#define MAX_PART_MC_UPPER_ALARM_ON_LIMIT              max_part_mc_upper_alarm_on_limit()
#define MAX_PART_VOC_UPPER_ALARM_ON_LIMIT             max_part_voc_upper_alarm_on_limit()
#define MAX_PART_NOX_UPPER_ALARM_ON_LIMIT             max_part_nox_upper_alarm_on_limit()

#define MAX_DP1_RANGE                         30000
#define MAX_DP2_RANGE                         30000
#define MAX_DP3_RANGE                         30000
#define MAX_DP1_ZERO_RNG                         max_abs_pres_zero_rng()
#define MAX_DP2_ZERO_RNG                        max_diff_pres_zero_rng()   
#define MAX_DP3_ZERO_RNG                       max_diff_pres3_zero_rng()             
#define MAX_DP1_ZERO_ADJ                         max_abs_pres_zero_adj()              
#define MAX_DP2_ZERO_ADJ                        max_diff_pres_zero_adj()     
#define MAX_DP3_ZERO_ADJ                        max_diff_pres3_zero_adj()         
#define MAX_TEMP_ZERO_ADJ                             max_temp_zero_adj()                  
#define MAX_RH_ZERO_ADJ                               max_rh_zero_adj()                    
#define MAX_DP1_OUT_HIGH_PRES                    max_pres_abs_out_high_pres()         
#define MAX_DP2_OUT_HIGH_PRES                   max_pres_diff_out_high_pres()        
#define MAX_TEMP_OUT_HIGH_TEMP                        max_temp_out_high_temp()             
#define MAX_RH_OUT_HIGH_RH                            max_rh_out_high_rh()                 
#define MAX_DP1_PID_SET_VALUE                    MAX_DP1_UPPER_ALARM_ON_LIMIT
#define MAX_DP2_PID_SET_VALUE                   MAX_DP2_UPPER_ALARM_ON_LIMIT
#define MAX_PRES_DIF3F_PID_SET_VALUE                  MAX_DP3_UPPER_ALARM_ON_LIMIT
#define MAX_TEMP_PID_SET_VALUE                        MAX_TEMP_UPPER_ALARM_ON_LIMIT
#define MAX_RH_PID_SET_VALUE                          MAX_RH_UPPER_ALARM_ON_LIMIT
#define MAX_DP1_AREA                             (32766) 
#define MAX_DP2_AREA                             (32766) 
#define MAX_DP3_AREA                             (32766) 

int d_pres_abs_upper_alarm_off_limit();
int d_pres_abs_upper_alarm_on_limit();      
int d_pres_abs_lower_alarm_off_limit();     
int d_pres_abs_lower_alarm_on_limit();      
int d_pres_diff_upper_alarm_off_limit();    
int d_pres_diff_upper_alarm_on_limit();     
int d_pres_diff_lower_alarm_off_limit();    
int d_pres_diff_lower_alarm_on_limit();   
int d_pres_diff3_upper_alarm_off_limit();
int d_pres_diff3_upper_alarm_on_limit();
int d_pres_diff3_lower_alarm_off_limit();
int d_pres_diff3_lower_alarm_on_limit();  
int d_temp_upper_alarm_off_limit();         
int d_temp_upper_alarm_on_limit();          
int d_temp_lower_alarm_off_limit();         
int d_temp_lower_alarm_on_limit();          
int d_rh_upper_alarm_off_limit();           
int d_rh_upper_alarm_on_limit();            
int d_rh_lower_alarm_off_limit();           
int d_rh_lower_alarm_on_limit();   
int d_pres_abs_pid_setpoint_limit();
int d_pres_diff_pid_setpoint_limit();

int d_part_mc_upper_alarm_off_limit();
int d_part_mc_upper_alarm_on_limit();
int d_part_voc_upper_alarm_off_limit();
int d_part_voc_upper_alarm_on_limit();
int d_part_nox_upper_alarm_off_limit();
int d_part_nox_upper_alarm_on_limit();
		 
int d_pres_abs_out_low_pres();              
int d_pres_abs_out_high_pres();             
int d_pres_diff_out_low_pres();            
int d_pres_diff_out_high_pres();            
int d_temp_out_low_temp();                  
int d_temp_out_high_temp();                 
int d_rh_out_low_rh();                      
int d_rh_out_high_rh();                     
int min_pres_abs_upper_alarm_off_limit();   
int min_pres_abs_lower_alarm_on_limit();    
int min_pres_diff_upper_alarm_off_limit();  
int min_pres_diff_lower_alarm_on_limit();   
int min_pres_diff3_upper_alarm_off_limit();
int min_pres_diff3_lower_alarm_on_limit();
int min_temp_upper_alarm_off_limit();       
int min_temp_lower_alarm_on_limit();        
int min_rh_upper_alarm_off_limit();         
int min_rh_lower_alarm_on_limit();    
int min_part_mc_upper_alarm_off_limit();
int min_part_voc_upper_alarm_off_limit();
int min_part_nox_upper_alarm_off_limit();
      
int min_abs_pres_zero_adj();                
int min_diff_pres_zero_adj();     
int min_diff_pres3_zero_adj();             
int min_temp_zero_adj();                    
int min_rh_zero_adj();                      
int min_pres_abs_out_low_pres();            
int min_pres_diff_out_low_pres();           
int min_temp_out_low_temp();                
int min_rh_out_low_rh();                    
int max_pres_abs_upper_alarm_on_limit();    
int max_pres_abs_lower_alarm_off_limit();   
int max_pres_diff_upper_alarm_on_limit();   
int max_pres_diff_lower_alarm_off_limit();  
int max_pres_diff3_upper_alarm_on_limit();
int max_pres_diff3_lower_alarm_off_limit();
int max_temp_upper_alarm_on_limit();        
int max_temp_lower_alarm_off_limit();       
int max_rh_upper_alarm_on_limit();          
int max_rh_lower_alarm_off_limit();    

int max_part_mc_upper_alarm_on_limit();
int max_part_mc_upper_alarm_off_limit();
int max_part_voc_upper_alarm_on_limit();
int max_part_voc_upper_alarm_off_limit();
int max_part_nox_upper_alarm_on_limit();
int max_part_nox_upper_alarm_off_limit();
     
int max_abs_pres_zero_rng();                
int max_diff_pres_zero_rng();   
int max_diff_pres3_zero_rng();              
int max_abs_pres_zero_adj();                
int max_diff_pres_zero_adj();  
int max_diff_pres3_zero_adj();              
int max_temp_zero_adj();                    
int max_rh_zero_adj();                      
int max_pres_abs_out_high_pres();           
int max_pres_diff_out_high_pres();          
int max_temp_out_high_temp();               
int max_rh_out_high_rh();                   

int findDewPoint(int Tempvalue, int RHvalue);
static void DisplayEditor(char keysPressed, char keys);
static void DisplayLog(char keysPressed, char keys);
uint8_t BCD2HEX(uint8_t bcd);
uint8_t HEX2BCD(uint8_t hex);


static void PrintLine( uint8_t align, char * str );
static void PrintLineCentered( uint8_t yPos, char * str );
static void PrintLineInBox( uint8_t side, uint8_t xPosmin, uint8_t xPosmax, uint8_t yPos, char * str );
static void PrintLineInBoxWOClear(uint8_t side, uint8_t xPosmin, uint8_t xPosmax, uint8_t yPos, char * str );

static char * PresAbsUpperAlarmOffPara( uint8_t operationType, int16_t * paraValue );
static char * PresAbsUpperAlarmOnPara( uint8_t operationType, int16_t * paraValue );
static char * PresAbsLowerAlarmOffPara( uint8_t operationType, int16_t * paraValue );
static char * PresAbsLowerAlarmOnPara( uint8_t operationType, int16_t * paraValue );
static char * TempUpperAlarmOffPara( uint8_t operationType, int16_t * paraValue );
static char * TempUpperAlarmOnPara( uint8_t operationType, int16_t * paraValue );
static char * TempLowerAlarmOffPara( uint8_t operationType, int16_t * paraValue );
static char * TempLowerAlarmOnPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityUpperAlarmOffPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityUpperAlarmOnPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityLowerAlarmOffPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityLowerAlarmOnPara( uint8_t operationType, int16_t * paraValue );
//static char * DisplayIntervalPara(uint8_t operationType, int16_t * paraValue );
//static char * DisplayModePara(uint8_t operationType, int16_t * paraValue );
static char * TimeHourPara( uint8_t operationType, int16_t * paraValue );
static char * TimeMinutePara( uint8_t operationType, int16_t * paraValue );
static char * TimeSecondPara( uint8_t operationType, int16_t * paraValue );
static char * DateYearPara( uint8_t operationType, int16_t * paraValue );
static char * DateMonthPara( uint8_t operationType, int16_t * paraValue );
static char * DateDayPara( uint8_t operationType, int16_t * paraValue );
static char * PressureAbsUnitPara( uint8_t operationType, int16_t * paraValue );
static char * TemperatureUnitPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityUnitPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityZeroAdjustPara(uint8_t operationType, int16_t * paraValue );
static char * AbsPresZeroRangePara(uint8_t operationType, int16_t * paraValue );
static char * AbsPresManpThPara(uint8_t operationType, int16_t * paraValue );
static char * AbsPresManpLmtPara(uint8_t operationType, int16_t * paraValue );
static char * AbsPresZeroAdjustPara(uint8_t operationType, int16_t * paraValue );

static char * PresRange1AdjPara(uint8_t operationType, int16_t * paraValue );
static char * PresRange2AdjPara(uint8_t operationType, int16_t * paraValue );
static char * PresRange3AdjPara(uint8_t operationType, int16_t * paraValue );
static char * PresRange4AdjPara(uint8_t operationType, int16_t * paraValue );
static char * PresRange5AdjPara(uint8_t operationType, int16_t * paraValue );
static char * PresRange6AdjPara(uint8_t operationType, int16_t * paraValue );

static char * TempZeroAdjustPara(uint8_t operationType, int16_t * paraValue );
static char * TempFireAlmSetPara(uint8_t operationType, int16_t * paraValue );
static char * TempFireAlmTimePara(uint8_t operationType, int16_t * paraValue );
static char * DoorAlmTimePara(uint8_t operationType, int16_t * paraValue );
static char * ConfigPasswordPara(uint8_t operationType, int16_t * paraValue );
//static char * LogggingIntervalPara(uint8_t operationType, int16_t * paraValue );
static char * DeviceIdPara(uint8_t operationType, int16_t * paraValue );
static char * BroadcastIntervalPara(uint8_t operationType, int16_t * paraValue );
static char * BroadcastPortPara(uint8_t operationType, int16_t * paraValue );
static char * SerialBaudratePara(uint8_t operationType, int16_t * paraValue );
static char * SerialDataStopParityPara(uint8_t operationType, int16_t * paraValue );
static char * PresAbsRange(uint8_t operationType, int16_t * paraValue );
static char * BuzzerOnPara( uint8_t operationType, int16_t * paraValue );
static char * BuzzerOffPara( uint8_t operationType, int16_t * paraValue );
static char * BuzzerDisablePara( uint8_t operationType, int16_t * paraValue );
static char * TempRHSensTypePara( uint8_t operationType, int16_t * paraValue );
static char * ROOMIdPara(uint8_t operationType, int16_t * paraValue );
static char * AHUIdPara(uint8_t operationType, int16_t * paraValue );

static char * DP1AreaTypePara( uint8_t operationType, int16_t * paraValue );
static char * DP1AreaLengthPara( uint8_t operationType, int16_t * paraValue );
static char * DP1AreaWidthPara( uint8_t operationType, int16_t * paraValue );
static char * DP1AreaRadiousPara( uint8_t operationType, int16_t * paraValue );

static char * PresAbsAlarmSetupPara( uint8_t operationType, int16_t * paraValue );
static char * TempAlarmSetupPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityAlarmSetupPara( uint8_t operationType, int16_t * paraValue );
static char * RTCSetupPara( uint8_t operationType, int16_t * paraValue );

static char * PresAbsSensTypePara(uint8_t operationType, int16_t * paraValue );

static char * VersionPara(uint8_t operationType, int16_t * paraValue );
static char * CustomerIDPara(uint8_t operationType, int16_t * paraValue );
static char * SerialNoPara(uint8_t operationType, int16_t * paraValue );
static char * SystemConfigPara(uint8_t operationType, int16_t * paraValue );


static void ParameterOperationsOnLowSettings( uint8_t operationType, int16_t * paraValue, int16_t dfltValue, int16_t minValue, int16_t maxValParaId, unsigned int eepromAddr );
static void ParameterOperationsOnHighSettings( uint8_t operationType, int16_t * paraValue, int16_t dfltValue, int16_t maxValue, int16_t minValParaId, unsigned int eepromAddr );
static char * ParameterOperationsTimeValues( uint8_t mask, uint8_t maskWriteVal, uint8_t operationType, int16_t * paraValue, int16_t dfltValue, int8_t minValue, int8_t maxValue, uint8_t regAddr );
static char * PressureUnitProcessPara( uint8_t operationType, int16_t * paraValue, int16_t dfltValue, unsigned int eepromAddr );
static char * ParaSetIntegerVal(uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal, int16_t minVal, int16_t maxVal );
static char * ParaSetIntegerVal1(uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal, int16_t minVal, int16_t maxVal );
static char * ParaSetfloatVal(uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal, int16_t minVal, int16_t maxVal );
static char * AlarmSetupPara( uint8_t operationType, int16_t * paraValue, unsigned int eeAddress );
static void ParameterOperationsSettings( uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltValue, int16_t maxValue, int16_t minValue );
static void ParameterOperationsSettingsRollover( uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal, int16_t minVal, int16_t maxVal );
//static void ParameterOperationsSettingsRolloverNoDefault( uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t minVal, int16_t maxVal );

static void FormatDP(uint8_t unit, int16_t value, int16_t range1, uint8_t AreaType, int16_t length, int16_t width, int16_t radious);
static void FormatTemperature( int16_t value, uint8_t decimal);
static void FormatHumidity( int16_t value, uint8_t decimal);
static void FormatFloat(int16_t value);

//static void FormatControlOutput( char * str, int value);
static void FormatError( int8_t error );
//static void FormatDispError( int8_t error );
static char * FormatInteger( int16_t value);
static char * FormatVersion();

static uint8_t IsMasterReset();
static uint8_t IsConfigPassword();
static uint8_t NotConfigPassword();

static uint8_t IsTempOrRHEnabled();
static uint8_t IsTempAndRHEnabled();
static uint8_t IsPresAbsAlarmEnabled();
static uint8_t IsTempAlarmEnabled();
static uint8_t IsHumidityAlarmEnabled();
static uint8_t IsRTCEnabled();

static void SetAllParametersToDefault();

static uint8_t GetParaInfo( PARAMETER reqId, PARA_INFO * paraInfoVal );
static uint8_t GetParaInfoByIndex( uint8_t index, PARA_INFO * paraInfoVal );

unsigned char systemError = ERROR_OK;

static uint8_t displayPage = 0,displayPage1 = 0;

static unsigned char LCDControl = BLOCKED;

unsigned char systemXLoc;
unsigned char systemYLoc;

static uint8_t isMasterReset;

static int16_t configPassword;

static int tempUnit, humidityUnit;
//static uint8_t gu8_fontcolor=WHITE;
static char displayStr[30],displayStr1[30];
static char formatedValue[12];
static const char *formatedUnit;
//static const char * formatedtype;
char gu8_inOutput[16][10]={0};
uint8_t gu8_IsCOMDisable=0, gu8_IsLCDDisable=0; 
uint16_t gu16_AlarmLimitCheckDisableTimer=0; 
int PreTd=0,PostTd=0,TDdiff=0;
uint8_t PreTdErr=0,PostTdErr=0,TDdiffErr=0;
	
static char prevKeys, currKeys, keys, keysPressed;

uint16_t eeprom_address = 0;
int16_t eeprom_value = 0;
uint8_t eeprom_write_timeout = 0;

// Deferred EEPROM write (one slot, written by the main loop after EEPROM_WRITE_TIMEOUT ticks).
// If a different parameter is already pending, write that one out first so it is not lost.
static void QueueParaEepromWrite(unsigned int eepromAddr, int16_t value)
{
	if( eeprom_write_timeout && eeprom_address != eepromAddr )
		OSWriteEEPromWord((unsigned int *)eeprom_address, eeprom_value);

	eeprom_address = eepromAddr;
	eeprom_value = value;
	eeprom_write_timeout = EEPROM_WRITE_TIMEOUT;
}

// Parameter read that also sees a value still waiting in the deferred write slot.
static int16_t ReadParaEeprom(unsigned int eepromAddr)
{
	if( eeprom_write_timeout && eeprom_address == eepromAddr )
		return eeprom_value;
	return (int16_t)OSReadEEPromWord((unsigned int *)eepromAddr);
}
   
//static uint8_t timer, curDisplay;
static SENSOR_VALUE senVal, senVal3;
//static ALARMS alarmOut;
//static sen5x_measured_value_t sen55;
//static uint8_t dispList[6], noOfSensors;

//static OSSemaMutex UIMutex;

unsigned int keybrdIdleTimer;

static TIME curTime;
static char dispValSel = 0;

void UserInterfaceTask( void )
{
	//OSSleep(KEYBOARD_LOOP_TIME);
	static uint8_t DPautocal=0, DPautocalHold=0;

	currKeys = (~KEYPAD_PORT_IN) & KEYS_ALL_MASK;
	keysPressed = keys;
	keys |= (currKeys & prevKeys);
	keys &= (currKeys | prevKeys);
	prevKeys = currKeys;

	keysPressed = (keys^keysPressed) & keys;

	if( keysPressed & MODE_SEL_KEY )
	{
		if( LCDControl == SENSOR_VALUES )
		{
			DisplayEditor( -1, -1 );
		}
		else if ( LCDControl == EDITOR )
		{
			if( IsLoggingEnabled() )
			{
				DisplayLog(-1, -1);
			}
			else
			{
				StartDisplaySensorValues();
			}
		}
		else
		{
			StartDisplaySensorValues();
		}
		keybrdIdleTimer = 0;
	}

	if( LCDControl == SENSOR_VALUES )
	{
		// Both keys held and one of them just pressed (requiring both new presses
		// in the same 50 ms tick made the silence combo almost never work)
		if((( keys & (UP_KEY | DOWN_KEY)) == (UP_KEY | DOWN_KEY)) && ( keysPressed & (UP_KEY | DOWN_KEY)))
		{
			SetBuzzerSuppressionReq();   // Suppress buzzer request (valid when displaying sensor values)
		}
		else if( !(KEYPAD_PORT_IN & (SETUP_KEY | DOWN_KEY) ))	//Auto DP calibration
		{
			// Keys must be held continuously for DP_AUTOCAL_HOLD_TICKS (also debounces),
			// the reading must be valid, and the value goes through SetParameter() so
			// it is clamped to MIN/MAX_DP1_ZERO_ADJ.
			if(DPautocalHold < DP_AUTOCAL_HOLD_TICKS)
			{
				DPautocalHold++;
			}
			else if(!DPautocal)
			{
				if(IsDP1Enabled())
				{
					GetPareValue(DP1_VAL_INDEX, &senVal3);
					if(senVal3.errorCode == ERROR_OK)
					{
						int zeroAdj = (int16_t)senVal3.rawValue;
						SetParameter( DP1_ZERO_ADJ, &zeroAdj );
						DPautocal = 1;
					}
				}
			}

			displayPage1 = DPautocal ? 4 : 0;
		}
		else
		{
			displayPage1 = 0;
			DPautocal = 0;
			DPautocalHold = 0;
		}
		return;//continue;
	}

	if( LCDControl == EDITOR )
	currKeys = KEYS_EDITOR_MASK;
	else if( LCDControl == SYSTEM_LOG )
	currKeys = (UP_KEY | DOWN_KEY );

	if(( keys & currKeys ) == 0)
	{
		keybrdIdleTimer++;
		if( keybrdIdleTimer >= KEYBOARD_IDLE_TIME/KEYBOARD_LOOP_TIME)
		StartDisplaySensorValues();
	}
	else
	{
		keybrdIdleTimer = 0;
			
		if( LCDControl == EDITOR )
			DisplayEditor( keysPressed & KEYS_EDITOR_MASK, keys & KEYS_EDITOR_MASK );
		else if( LCDControl == SYSTEM_LOG )
			DisplayLog( keysPressed & ( UP_KEY | DOWN_KEY ), keys & ( UP_KEY | DOWN_KEY ));
	}
}


void UserInterfaceInit()
{
   isMasterReset = 0;
   configPassword = 0;

   KEYPAD_PORT_DIR  &= ~KEYS_ALL_MASK;
   KEYPAD_PORT |= KEYS_ALL_MASK;

   SETDFLTSW_DIR_IN;
   SETDFLTSW_HIGH;

   BUZZER_DIR_OP;
   BUZZER_OFF;

   prevKeys = 0;
   currKeys = 0;
   keys = 0;
   keysPressed = 0;

   if (IsLCDDisplay())
   {
      systemXLoc = 0;
      //OSSemaCreateMutex(UIMutex);
      //OSSemaGive(UIMutex);
      //OSSemaTakeEver(UIMutex);
	  
      // Initialize the LCD
      uc1638_Init();	
	  
	  // Select a font
	  ks0xxx_SelectFont(Font_Verdana_10x24, ks0xxx_ReadFontData, BLACK);
	  // Print SAP India
	  locked_sprintf_P( displayStr, PSTR("info@sapindia.in"));
	  PrintLineCentered( 16, displayStr );
	  locked_sprintf_P( displayStr, PSTR("NIYAMA"));
	  PrintLineCentered( 48, displayStr );
	  locked_sprintf_P( displayStr, PSTR("Version: %s"), FormatVersion());
	  PrintLineCentered( 80, displayStr );
	
	  ks0xxx_DrawVertLine(0, 0, GLCD_PIXEL_Y-1, BLACK);
	  ks0xxx_DrawVertLine(GLCD_PIXEL_X-1, 0, GLCD_PIXEL_Y-1, BLACK);
	  
	  ks0xxx_DrawHoriLine(1, 0, GLCD_PIXEL_X-2, BLACK);
	  ks0xxx_DrawHoriLine(1, GLCD_PIXEL_Y-1, GLCD_PIXEL_X-2, BLACK);
	  
      systemXLoc = SCREEN_START_X_VAL;
      systemYLoc = SCREEN_START_Y_VAL;
	  
      if(IsMasterReset())
      {
         // Print Master Reset
         locked_sprintf_P( displayStr, PSTR("Master Reset"));
         PrintLineCentered( 112, displayStr );
         isMasterReset = 1;
         SetAllParametersToDefault();
		 isMasterReset = 0;
      }
	  
      //OSSemaGive(UIMutex);
	  
	  wdt_reset();
   }

	_delay_ms(2000);

	keybrdIdleTimer = 0;
	StartDisplaySensorValues();
}

void StartDisplaySensorValues(void)
{
	uc1638_FillScreen(BLANK1);
	
	LCDControl = SENSOR_VALUES;
}

void DisplaySensorValues(void)
{
	//uint8_t errorCode;
	//int value1=0;
	//int PreRHvalue=0,PreTempvalue=0;

	if (IsLCDDisplay())
	{
		if ( LCDControl != SENSOR_VALUES )
		return;

		if(u8_DFUresetTimer)
		{
			uc1638_FillScreen(BLANK1);
			ks0xxx_SelectFont(Font_Verdana_10x24, ks0xxx_ReadFontData, BLACK);
			PrintLineInBoxWOClear(CENTER,0,GLCD_PIXEL_X-1,60,"< DFU MODE >");

			// Countdown and reset are done in PERIODIC_FUNC_NAME(), so they also
			// happen while the editor is open or the LCD is disabled.
			return;
		}
		
		ALARMS alarmOut;
		static uint8_t toggle=0,clear=0;
		alarmOut.alarmByte = GetAlarms();
		if((alarmOut.alarm.fire==1))// || (alarmOut.alarm.door==1))
		{
			uc1638_FillScreen(BLANK1);
			toggle ^= 1;
			if(toggle)
			{
				//ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
				ks0xxx_SelectFont(Font_Verdana_10x24, ks0xxx_ReadFontData, BLACK);
				if(alarmOut.alarm.fire==1) PrintLineInBoxWOClear(CENTER,0,GLCD_PIXEL_X-1,60,"< FIRE DETECTED >");
				//else if(alarmOut.alarm.door==1) PrintLineInBoxWOClear(CENTER,0,GLCD_PIXEL_X-1,60,"< DOOR OPEN >");
				else
				{
					PrintLineInBoxWOClear(CENTER,0,GLCD_PIXEL_X-1,40,"< FIRE DETECTED >");
				    //PrintLineInBoxWOClear(CENTER,0,GLCD_PIXEL_X-1,60,"< DOOR OPEN >");
				}
			}
			
			//if(GetParameterValue(OUTPUT1_MAPPING)==OUTPUT1_NORMAL)
			//{
				//OUTPUT1_HIGH;
			//}
			//else
			//{
				//OUTPUT1_LOW;
			//}

			clear=1;
			return;
		}
		else
		{
			//if(GetParameterValue(OUTPUT1_MAPPING)==OUTPUT1_NORMAL)
			//{
				//OUTPUT1_LOW;
			//}
			//else
			//{
				//OUTPUT1_HIGH;
			//}
		}

		if(displayPage1==4)
		{
			uc1638_FillScreen(BLANK1);
			ks0xxx_SelectFont(Font_Verdana_10x24, ks0xxx_ReadFontData, BLACK);
			PrintLineInBoxWOClear(CENTER,0,GLCD_PIXEL_X-1,60,"DP AUTO CAL DONE");
			
			clear=1;
			return;
		}
		
		if(clear==1)
		{
			StartDisplaySensorValues();
			clear=0;
		}
		
		memset(displayStr,0,sizeof(displayStr));
		memset(displayStr1,0,sizeof(displayStr1));
		displayPage=displayPage1;

		ks0xxx_SelectFont(Bodoni15, ks0xxx_ReadFontData, BLACK);
		sprintf(displayStr1,"AHU:%s",FormatInteger (GetParameterValue( AHU_ID )));
		PrintLineInBox(LEFT,5,120,130,displayStr1);
		
		sprintf(displayStr1,"ROOM:%s",FormatInteger (GetParameterValue( ROOM_ID )));
		PrintLineInBox(LEFT,70,120,130,displayStr1);
		
		sprintf(displayStr1,"ID:%s",FormatInteger (GetParameterValue( DEVICE_ID )));
		PrintLineInBox(LEFT,5,120,116,displayStr1);
		
		sprintf(displayStr1,"%s  ",(CurrentDoorStatus()==OPEN)?"OPEN":"CLOSE");
		PrintLineInBox(LEFT,90,140,144,displayStr1);
		//---------------------------------------------------------------------
		if(IsDP1Enabled() && !IsTempAndRHEnabled())
		{
			GetPareValue(DP1_VAL_INDEX, &senVal);
			if( abs(senVal.convertedValue) < GetParameterValue( DP1_ZERO_RNG ))
			senVal.convertedValue = 0;
		
			if( abs(senVal.convertedValue) < 5000)
			senVal.convertedValue += GetParameterValue( DP_RANGE1_ADJ );
			else if( abs(senVal.convertedValue) < 10000)
			senVal.convertedValue += GetParameterValue( DP_RANGE2_ADJ );
			else if( abs(senVal.convertedValue) < 15000)
			senVal.convertedValue += GetParameterValue( DP_RANGE3_ADJ );
			else if( abs(senVal.convertedValue) < 20000)
			senVal.convertedValue += GetParameterValue( DP_RANGE4_ADJ );
			else if( abs(senVal.convertedValue) < 25000)
			senVal.convertedValue += GetParameterValue( DP_RANGE5_ADJ );
			else if( abs(senVal.convertedValue) < 30000)
			senVal.convertedValue += GetParameterValue( DP_RANGE6_ADJ );
		
			DisplayDP1Only(senVal.errorCode, senVal.convertedValue);
		}
		else if(!IsDP1Enabled() && IsTempAndRHEnabled())
		{
			//---------------------------------------------------------------------
			GetPareValue(HUMIDITY_VAL_INDEX, &senVal);
			DisplayHumidityWithNoDP(senVal.errorCode, senVal.convertedValue);
			//---------------------------------------------------------------------
			GetPareValue(TEMPERATURE_VAL_INDEX, &senVal);
			DisplayTemperatureWithNoDP(senVal.errorCode, senVal.convertedValue);
		}
		else
		{
			GetPareValue(DP1_VAL_INDEX, &senVal);
			if( abs(senVal.convertedValue) < GetParameterValue( DP1_ZERO_RNG ))
			senVal.convertedValue = 0;
			
			if( abs(senVal.convertedValue) < 5000)
			senVal.convertedValue += GetParameterValue( DP_RANGE1_ADJ );
			else if( abs(senVal.convertedValue) < 10000)
			senVal.convertedValue += GetParameterValue( DP_RANGE2_ADJ );
			else if( abs(senVal.convertedValue) < 15000)
			senVal.convertedValue += GetParameterValue( DP_RANGE3_ADJ );
			else if( abs(senVal.convertedValue) < 20000)
			senVal.convertedValue += GetParameterValue( DP_RANGE4_ADJ );
			else if( abs(senVal.convertedValue) < 25000)
			senVal.convertedValue += GetParameterValue( DP_RANGE5_ADJ );
			else if( abs(senVal.convertedValue) < 30000)
			senVal.convertedValue += GetParameterValue( DP_RANGE6_ADJ );
			
			DisplayDP1(senVal.errorCode, senVal.convertedValue);
			//---------------------------------------------------------------------
			GetPareValue(HUMIDITY_VAL_INDEX, &senVal);
			DisplayHumidity(senVal.errorCode, senVal.convertedValue);
			//---------------------------------------------------------------------
			GetPareValue(TEMPERATURE_VAL_INDEX, &senVal);
			DisplayTemperature(senVal.errorCode, senVal.convertedValue);
		}
		//---------------------------------------------------------------------
		if(IsRTCEnabled())
		{
			DisplayRTC();
		}
		//---------------------------------------------------------------------
	}
}

int findDewPoint(int Tempvalue, int RHvalue)
{
	double n=0, d=0;
	double temp=(double)Tempvalue/100;
	double rh=(double)RHvalue/100;
	
	n = (log(rh / 100) + ((17.27 * temp) / (237.3 + temp))) / 17.27;
	d = (237.3 * n) / (1 - n);
	
	return (d*100);
}



static void DisplayLog(char keysPressed, char keys)
{
   LCDControl = SYSTEM_LOG;
   if (IsLCDDisplay())
   {
////
   }
}

// align = 0; left align
// align = 1; center align
static void PrintLine( uint8_t align, char * str )
{
    uint8_t len = ks0xxx_StringWidth(str);
    uint8_t scrWidth = (GLCD_PIXEL_X-1) - systemXLoc, start, spaceWidth = ks0xxx_CharWidth( ' ' ), i;
    if( align == 0 )
       start = 0;
    else
       start = ( scrWidth - len ) / 2;
    for( i = spaceWidth; i < start; i += spaceWidth )
       ks0xxx_PutChar( ' ' );
    ks0xxx_Puts(str);
    for( i = i+len; len < scrWidth; len += spaceWidth )
       ks0xxx_PutChar( ' ' );
}

static void PrintLineCentered( uint8_t yPos, char * str )
{
    uint8_t len;
    uint8_t start;
    uint8_t height = ks0xxx_FontHeight();

    while((len = ks0xxx_StringWidth(str)) > GLCD_PIXEL_X)
       str[strlen(str)-1] = 0;   // remove out of screen charaters

    start = (GLCD_PIXEL_X/2) - len/2;
    ks0xxx_FillRect(0, yPos, start, height, WHITE);
    uc1638_GotoXY(start,yPos);
    ks0xxx_Puts(str);
    ks0xxx_FillRect(start+len, yPos, GLCD_PIXEL_X-start-len, height, WHITE);
}

//static void PrintLineCenteredXY(uint8_t xPos, uint8_t yPos, char * str )
//{
	//uint8_t len;
	//uint8_t start;
	//uint8_t height = ks0xxx_FontHeight();
//
	//while((len = ks0xxx_StringWidth(str)) > (GLCD_PIXEL_X-xPos))
	//str[strlen(str)-1] = 0;   // remove out of screen charaters
//
	//start = xPos + (((GLCD_PIXEL_X-xPos)/2) - len/2);
	//ks0xxx_FillRect(xPos, yPos, start, height, WHITE);
	//uc1638_GotoXY(start,yPos);
	//ks0xxx_Puts(str);
	//ks0xxx_FillRect(start+len, yPos, GLCD_PIXEL_X-start-len, height, WHITE);
//}

static void PrintLineInBox(uint8_t side, uint8_t xPosmin, uint8_t xPosmax, uint8_t yPos, char * str )
{
	uint8_t len = ks0xxx_StringWidth(str);
	uint8_t scrWidth = (xPosmax - xPosmin)-ks0xxx_CharWidth(' ');
	uint8_t spaceWidth = ks0xxx_CharWidth(' ');
	uint8_t start=0, i=0;
	
	if( side == LEFT )
	{
		start = xPosmin;
	}
	else
	{
		start = xPosmin + (( scrWidth - len ) / 2);
	}
	
	uc1638_GotoXY(start,yPos);
	for(i = (xPosmin+spaceWidth); i < start; i += spaceWidth)
	{
		ks0xxx_PutChar(' ');
	}
	
	ks0xxx_Puts(str);
	
	for(i = i+len; len < scrWidth; len += spaceWidth)
	{
		ks0xxx_PutChar(' ');
	}
	
	//-----------------------------------------------------------------------
	//uint8_t len = ks0xxx_StringWidth(str);
	//uint8_t scrWidth = (xPosmax - xPosmin);//-ks0xxx_CharWidth(' ');
	//uint8_t charWidth = ks0xxx_CharWidth(' ');
	//uint8_t start=0, i=0;
	//
	//if( side == LEFT )
	//{
		//start = xPosmin;
	//}
	//else
	//{
		//start = xPosmin + (( scrWidth - len ) / 2);
	//}
	//
	//uc1638_GotoXY(xPosmin,yPos);
	////for(i = (xPosmin+spaceWidth); i < start; i += spaceWidth)
	//for(i = (xPosmin); i <= start; i += charWidth)
	//{
		//ks0xxx_PutChar(' ');
	//}
	//uc1638_GotoXY(start,yPos);
	//ks0xxx_Puts(str);
	//
	//for(i = i+len; len < (scrWidth-len); len += charWidth)
	//{
		//ks0xxx_PutChar(' ');
	//}

	//uint8_t len = ks0xxx_StringWidth(str);
	//uint8_t start,width;
	//uint8_t height = ks0xxx_FontHeight();
//
	//while(len > (xPosmax-xPosmin))
	//{
		//str[strlen(str)-1] = 0;   // remove out of screen characters
	//}
	//
	//width = ((xPosmax-xPosmin)/2) - len/2;
	//start = width + xPosmin + 1;
	//
	//if(side == CENTER)
	//{
		//ks0xxx_FillRect(xPosmin, yPos, width, height, WHITE);
		//uc1638_GotoXY(start,yPos);
		//ks0xxx_Puts(str);
		//ks0xxx_FillRect(xPosmin+width+len, yPos, width, height, WHITE);
	//}
	//else
	//{
		//ks0xxx_FillRect(xPosmin, yPos, xPosmax-xPosmin, height, WHITE);
		//uc1638_GotoXY(xPosmin,yPos);
		//ks0xxx_Puts(str);
	//}
}

static void PrintLineInBoxWOClear(uint8_t side, uint8_t xPosmin, uint8_t xPosmax, uint8_t yPos, char * str )
{
	uint16_t len = ks0xxx_StringWidth(str);
	uint8_t start,width;

	//while(len > (xPosmax-xPosmin))
	if(len > (xPosmax-xPosmin))
	{
		str[strlen(str)-1] = 0;   // remove out of screen characters
	}
	
	width = ((xPosmax-xPosmin)/2) - len/2;
	start = width + xPosmin + 1;
	
	if(side == CENTER)
	{
		uc1638_GotoXY(start,yPos);
		ks0xxx_Puts(str);
	}
	else
	{
		uc1638_GotoXY(xPosmin,yPos);
		ks0xxx_Puts(str);
	}
}

void DisplaySystemError(void)
{
	if (IsLCDDisplay())
	{
		//OSSemaTakeEver(UIMutex);
		
		FormatSystemError( displayStr, systemError);
		ks0xxx_SelectFont(Bodoni15, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,5,GLCD_PIXEL_X/2,145,displayStr);
		
		//OSSemaGive(UIMutex);
	}
}

void DisplayRTC(void)
{
	if (IsLCDDisplay())
	{
		static uint8_t lastsec=0, toggle=0, curhr=0, curhr1=0;

		if(RTCGetTime( &curTime ) == ERROR_OK)
		{
			if(lastsec != curTime.seconds)
			{
				lastsec = curTime.seconds;
				toggle ^= 1;
			}
					
			curhr = BCD2HEX(curTime.hours);
			if(curhr>=12)
			{
				if(curhr>12) curhr -=12;
				displayStr[6] = 'P';
				displayStr[7] = 'M';
			}
			else
			{
				displayStr[6] = 'A';
				displayStr[7] = 'M';
			}
			curhr1 = HEX2BCD(curhr);
					
			displayStr[0] = (curhr1 >> 4) + '0';
			displayStr[1] = (curhr1 & 0xf ) + '0';
			displayStr[2] = (toggle==0)?' ':':';
			displayStr[3] = (curTime.minutes >> 4) + '0';
			displayStr[4] = (curTime.minutes & 0xf ) + '0';
			displayStr[5] = ' ';
					
			displayStr[8] = 0;
		}
		else
		{
			locked_sprintf_P( displayStr, PSTR("RTC ERR"));
		}
		
		//ks0xxx_SelectFont(Bodoni15, ks0xxx_ReadFontData, BLACK);
		ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
		//ks0xxx_SelectFont(Bodoni15, ks0xxx_ReadFontData, BLACK);
		//sprintf(displayStr1,"TIME:%s",displayStr);
		//PrintLineInBox(LEFT,5,120,144,"TIME:");
		//ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,5,80,144,displayStr);
	}
}

char DisplayDP1(uint8_t error, int16_t value)
{
	char noSystemErrDisp = 0;

	if (IsLCDDisplay())
	{
		FormatDP(GetParameterValue( DP1_UNIT ), value, GetParameterValue(DP1_RANGE), GetParameterValue(DP1_AREA_TYPE), GetParameterValue(DP1_AREA_LENGTH), GetParameterValue(DP1_AREA_WIDTH), GetParameterValue(DP1_AREA_RADIOUS));
		formatedValue[strlen(formatedValue)-1] = '\0';  // one digit less resolution then setup
		if(formatedValue[strlen(formatedValue)-1] == '.' )
		formatedValue[strlen(formatedValue)-1] = '\0';  // if last digit '.' remove it
		if( error != ERROR_OK )
		{
			FormatError( error );
		}

		locked_sprintf_P( displayStr, PSTR("DP  (%S)"), formatedUnit);
		ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,25,70,0,displayStr);
		
		locked_sprintf_P( displayStr, PSTR("%s"), formatedValue );
		ks0xxx_SelectFont( Bodoni89, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,25,150,16,displayStr);
		
		uint8_t length = 0;
		memset(displayStr1,0,sizeof(displayStr1));
		
		FormatDP(GetParameterValue( DP1_UNIT ), GetParameterValue( DP1_LOWER_ALARM_ON_LIMIT ), GetParameterValue(DP1_RANGE), GetParameterValue(DP1_AREA_TYPE), GetParameterValue(DP1_AREA_LENGTH), GetParameterValue(DP1_AREA_WIDTH), GetParameterValue(DP1_AREA_RADIOUS));
		formatedValue[strlen(formatedValue)-1] = '\0';  // one digit less resolution then setup
		if(formatedValue[strlen(formatedValue)-1] == '.' )
		formatedValue[strlen(formatedValue)-1] = '\0';  // if last digit '.' remove it
		sprintf(displayStr1,"L%s     ",formatedValue);
		
		length = strlen(displayStr1);
		FormatDP(GetParameterValue( DP1_UNIT ), GetParameterValue( DP1_UPPER_ALARM_ON_LIMIT ), GetParameterValue(DP1_RANGE), GetParameterValue(DP1_AREA_TYPE), GetParameterValue(DP1_AREA_LENGTH), GetParameterValue(DP1_AREA_WIDTH), GetParameterValue(DP1_AREA_RADIOUS));
		formatedValue[strlen(formatedValue)-1] = '\0';  // one digit less resolution then setup
		if(formatedValue[strlen(formatedValue)-1] == '.' )
		formatedValue[strlen(formatedValue)-1] = '\0';  // if last digit '.' remove it
		sprintf(&displayStr1[length],"H%s",formatedValue);
		
		ks0xxx_SelectFont( Bodoni15, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,25,135,90,displayStr1);
	}
	return noSystemErrDisp;
}

char DisplayTemperature(uint8_t error, int16_t value)
{
	char noSystemErrDisp = 0;

	if (IsLCDDisplay())
	{
		FormatTemperature( value, 1 );
		if( error != ERROR_OK )
		{
			FormatError( error );
		}
		
		locked_sprintf_P( displayStr, PSTR("TEMP  (%S)"), formatedUnit);
		ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,150,GLCD_PIXEL_X-1,0,displayStr);
		
		locked_sprintf_P( displayStr, PSTR("%s"), formatedValue );
		ks0xxx_SelectFont(Bodoni50, ks0xxx_ReadFontData, BLACK);	
		PrintLineInBox(LEFT,150,GLCD_PIXEL_X-1,25,displayStr);
		
		uint8_t length = 0;
		memset(displayStr1,0,sizeof(displayStr1));
		FormatTemperature (GetParameterValue( TEMP_LOWER_ALARM_ON_LIMIT ),1);
		sprintf(displayStr1,"L%s     ",formatedValue);
		
		length = strlen(displayStr1);
		FormatTemperature (GetParameterValue( TEMP_UPPER_ALARM_ON_LIMIT ),1);
		sprintf(&displayStr1[length],"H%s",formatedValue);
		
		ks0xxx_SelectFont(Bodoni15, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,150,GLCD_PIXEL_X-1,70,displayStr1);
	}
	return noSystemErrDisp;
}

char DisplayHumidity(uint8_t error, int16_t value)
{
	char noSystemErrDisp = 0;
	
	if (IsLCDDisplay())
	{
		FormatHumidity( value, 1 );
		if( error != ERROR_OK )
		{
			FormatError( error );
		}
		
		locked_sprintf_P( displayStr, PSTR("RH   (%S)"), formatedUnit);
		ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,150,GLCD_PIXEL_X-1,80,displayStr);

		locked_sprintf_P( displayStr, PSTR("%s"), formatedValue );
		ks0xxx_SelectFont(Bodoni50, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,150,GLCD_PIXEL_X-1,105,displayStr);
		
		uint8_t length = 0;
		memset(displayStr1,0,sizeof(displayStr1));
		FormatHumidity (GetParameterValue( RH_LOWER_ALARM_ON_LIMIT ),1);
		sprintf(displayStr1,"L%s     ",formatedValue);
		
		length = strlen(displayStr1);
		FormatHumidity (GetParameterValue( RH_UPPER_ALARM_ON_LIMIT ),1);
		sprintf(&displayStr1[length],"H%s",formatedValue);
		
		ks0xxx_SelectFont(Bodoni15, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,150,GLCD_PIXEL_X-1,150,displayStr1);
	}
	return noSystemErrDisp;
}

char DisplayDP1Only(uint8_t error, int16_t value)
{
	char noSystemErrDisp = 0;
	
	if (IsLCDDisplay())
	{
		FormatDP(GetParameterValue( DP1_UNIT ), value, GetParameterValue(DP1_RANGE), GetParameterValue(DP1_AREA_TYPE), GetParameterValue(DP1_AREA_LENGTH), GetParameterValue(DP1_AREA_WIDTH), GetParameterValue(DP1_AREA_RADIOUS));
		formatedValue[strlen(formatedValue)-1] = '\0';  // one digit less resolution then setup
		if(formatedValue[strlen(formatedValue)-1] == '.' )
		formatedValue[strlen(formatedValue)-1] = '\0';  // if last digit '.' remove it
		if( error != ERROR_OK )
		{
			FormatError( error );
		}

		locked_sprintf_P( displayStr, PSTR("DP  (%S)"), formatedUnit);
		ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,90,GLCD_PIXEL_X-1,0,displayStr);
		
		locked_sprintf_P( displayStr, PSTR("%s"), formatedValue );
		ks0xxx_SelectFont( Bodoni89, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,85,GLCD_PIXEL_X-1,16,displayStr);
		
		uint8_t length = 0;
		memset(displayStr1,0,sizeof(displayStr1));
		
		FormatDP(GetParameterValue( DP1_UNIT ), GetParameterValue( DP1_LOWER_ALARM_ON_LIMIT ), GetParameterValue(DP1_RANGE), GetParameterValue(DP1_AREA_TYPE), GetParameterValue(DP1_AREA_LENGTH), GetParameterValue(DP1_AREA_WIDTH), GetParameterValue(DP1_AREA_RADIOUS));
		formatedValue[strlen(formatedValue)-1] = '\0';  // one digit less resolution then setup
		if(formatedValue[strlen(formatedValue)-1] == '.' )
		formatedValue[strlen(formatedValue)-1] = '\0';  // if last digit '.' remove it
		sprintf(displayStr1,"L%s     ",formatedValue);
		
		length = strlen(displayStr1);
		FormatDP(GetParameterValue( DP1_UNIT ), GetParameterValue( DP1_UPPER_ALARM_ON_LIMIT ), GetParameterValue(DP1_RANGE), GetParameterValue(DP1_AREA_TYPE), GetParameterValue(DP1_AREA_LENGTH), GetParameterValue(DP1_AREA_WIDTH), GetParameterValue(DP1_AREA_RADIOUS));
		formatedValue[strlen(formatedValue)-1] = '\0';  // one digit less resolution then setup
		if(formatedValue[strlen(formatedValue)-1] == '.' )
		formatedValue[strlen(formatedValue)-1] = '\0';  // if last digit '.' remove it
		sprintf(&displayStr1[length],"H%s",formatedValue);
		
		ks0xxx_SelectFont( Bodoni15, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,95,GLCD_PIXEL_X-1,90,displayStr1);
	}
	return noSystemErrDisp;
}

char DisplayTemperatureWithNoDP(uint8_t error, int16_t value)
{
	char noSystemErrDisp = 0;

	if (IsLCDDisplay())
	{
		FormatTemperature( value, 1 );
		if( error != ERROR_OK )
		{
			FormatError( error );
		}
		
		locked_sprintf_P( displayStr, PSTR("TEMP  (%S)"), formatedUnit);
		ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,15,GLCD_PIXEL_X/2,0,displayStr);
		
		locked_sprintf_P( displayStr, PSTR("%s"), formatedValue );
		ks0xxx_SelectFont(Bodoni89, ks0xxx_ReadFontData, BLACK);		
		PrintLineInBox(LEFT,15,GLCD_PIXEL_X/2,16,displayStr);
		
		uint8_t length = 0;
		memset(displayStr1,0,sizeof(displayStr1));
		FormatTemperature (GetParameterValue( TEMP_LOWER_ALARM_ON_LIMIT ),1);
		sprintf(displayStr1,"L%s     ",formatedValue);
		
		length = strlen(displayStr1);
		FormatTemperature (GetParameterValue( TEMP_UPPER_ALARM_ON_LIMIT ),1);
		sprintf(&displayStr1[length],"H%s",formatedValue);
		
		ks0xxx_SelectFont(Bodoni15, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,25,GLCD_PIXEL_X/2,90,displayStr1);
	}
	return noSystemErrDisp;
}

char DisplayHumidityWithNoDP(uint8_t error, int16_t value)
{
	char noSystemErrDisp = 0;
	
	if (IsLCDDisplay())
	{
		FormatHumidity( value, 1 );
		if( error != ERROR_OK )
		{
			FormatError( error );
		}
		
		locked_sprintf_P( displayStr, PSTR("RH  (%S)"), formatedUnit);
		ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,145,GLCD_PIXEL_X-1,0,displayStr);

		locked_sprintf_P( displayStr, PSTR("%s"), formatedValue );
		ks0xxx_SelectFont( Bodoni89, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,130,GLCD_PIXEL_X-1,16,displayStr);
		
		uint8_t length = 0;
		memset(displayStr1,0,sizeof(displayStr1));
		FormatHumidity (GetParameterValue( RH_LOWER_ALARM_ON_LIMIT ),1);
		sprintf(displayStr1,"L%s     ",formatedValue);
		
		length = strlen(displayStr1);
		FormatHumidity (GetParameterValue( RH_UPPER_ALARM_ON_LIMIT ),1);
		sprintf(&displayStr1[length],"H%s",formatedValue);
		
		ks0xxx_SelectFont(Bodoni15, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,140,GLCD_PIXEL_X-1,90,displayStr1);
	}
	return noSystemErrDisp;
}

//#pragma O0
/* Converts and formats the sting for output */
static void FormatDP(uint8_t unit, int16_t value, int16_t range1, uint8_t AreaType, int16_t length, int16_t width, int16_t radious)
{
	double result,area=1;
	uint8_t decimal;  // minimum value is 1
	
	//range = range1;
	//range /= 6894.7;	//(GetParameterValue(DP1_RANGE)/6895.7)  // Pa to PSI conversion
	
	if( unit == (uint8_t)PRESSURE_CFM )
	{
		if(AreaType==PRES_AREA_SQUARE)
		{
			area = length;
			area *= width;
			area /= 92903.04;
		}
		else if(AreaType==PRES_AREA_ROUND)
		{
			//area = (3.14159265359 * radious * radious)/1000000;
			area = radious;
			area *= radious;
			area *= 3.14159265359;
			area /= 92903.04;
		}
		else
		{
			area = 1.0;
		}
	}
	
	if (IsLCDDisplay())
	{
		if( unit == (uint8_t)PRESSURE_MILIBAR )
		{
			result = value * 0.01;
			formatedUnit = PSTR("mBAR");
			decimal = 4;
		}
		else if( unit == (uint8_t)PRESSURE_PA )
		{
			result = value;
			formatedUnit = PSTR("Pa");
			if( result > -0.1 && result < 0 )
			result = 0;
			
			if(result<-range1) result=-(range1-1);
			if(result>range1) result=(range1-1);
			
			if((result<10000) && (result>-10000)) 	decimal = 2;
			else decimal = 1;
		}
		else if( unit == (uint8_t)PRESSURE_HG_INCH )
		{
			result = value * 0.000295299;
			formatedUnit = PSTR("iHg");
			decimal = 5;
		}
		else if( unit == (uint8_t)PRESSURE_PSF )
		{
			result = value * 0.020885433;
			formatedUnit = PSTR("PSF");
			decimal = 3;
		}
		else if( unit == (uint8_t)PRESSURE_H2O_MM )
		{
			result = value * 0.101971621298;
			formatedUnit = PSTR("mmWC");
			decimal = 2;
		}
		else if( unit == (uint8_t)PRESSURE_H2O_INCH )
		{
			result = value * 0.00401463075976;
			formatedUnit = PSTR("iH2O");
			decimal = 4;
		}
		else if( unit == (uint8_t)PRESSURE_CFM )
		{
			result = (4005.0 * sqrt(value * 0.00401463075976) * area);
			formatedUnit = PSTR("CFM");
			decimal = 1;
		}
		else
		{
			result = value * 0.00014503773773;
			formatedUnit = PSTR("PSI");
			decimal = 5;
		}

		result /= 100;
		
		memset(formatedValue,0,sizeof(formatedValue));
		dtostrf( result, sizeof(formatedValue)-1, decimal, formatedValue);
		strcpy( formatedValue, strtrim(formatedValue));
	}
	return;
}

//#pragma Os

/* Converts and formats the sting for output */
static void FormatTemperature(int16_t value, uint8_t decimal)
{
   double result;
   tempUnit = GetParameterValue( TEMPERATURE_UNIT );

   if (IsLCDDisplay())
   {
      if( tempUnit == TEMPERATURE_DEG_F )
      {
         formatedUnit = PSTR("*F");
         result = ((double)value * 9 / 500) + 32;
      }
      else if( tempUnit == TEMPERATURE_DEG_K )
      {
         formatedUnit = PSTR("*K");
         result = value / 100.0 + 273.15;
      }
      else
      {
         formatedUnit = PSTR("*C");
         result = value / 100.0;
      }

	  memset(formatedValue,0,sizeof(formatedValue));
      dtostrf( result, sizeof(formatedValue)-1, decimal, formatedValue);
      strcpy( formatedValue, strtrim(formatedValue));
   }
   return;
}

/* Converts and formats the sting for output */
static void FormatHumidity(int16_t value, uint8_t decimal)
{
   double result;

   humidityUnit = GetParameterValue( HUMIDITY_UNIT );

   if (IsLCDDisplay())
   {
      formatedUnit = PSTR("%");
      result = value / 100.0;
	  memset(formatedValue,0,sizeof(formatedValue));
      dtostrf( result, sizeof(formatedValue)-1, decimal, formatedValue);
      strcpy( formatedValue, strtrim(formatedValue));
   }
   return;
}

/* Converts and formats the sting for output */
static void FormatFloat(int16_t value)
{
	double result;

	result = value / 100.0;
	dtostrf( result, sizeof(formatedValue)-1, 1, formatedValue);
	strcpy( formatedValue, strtrim(formatedValue));

	return;
}

//-------------------------------
// Convert BCD 1 byte to HEX 1 byte
//-------------------------------
uint8_t BCD2HEX(uint8_t bcd)
{
	uint8_t temp=0;
	temp=((bcd>>4)*10);
	temp+=(bcd & 0x0F);
	
	return temp;
}

//-------------------------------
// Convert HEX 1 byte to BCD 1 byte
//-------------------------------
uint8_t HEX2BCD(uint8_t hex)
{
	uint8_t temp=0;
	temp = ((hex / 10)<<4);
	temp |= (hex % 10);
	return temp;
}


/* formats the sting for output */
char * FormatSystemError( char * str, int16_t value )
{
	static uint8_t lastsec=0, toggle=0, curhr=0, curhr1=0;
	
	if (IsLCDDisplay())
	{		
		if(dispValSel == (char)0) 
		{
			if(IsRTCEnabled())
			{
				if(RTCGetTime( &curTime ) == ERROR_OK)
				{
					if(lastsec != curTime.seconds)
					{
						lastsec = curTime.seconds;
						toggle ^= 1;
					}
					
					curhr = BCD2HEX(curTime.hours);
					if(curhr>=12)
					{
						if(curhr>12) curhr -=12;
						displayStr[6] = 'p';
						displayStr[7] = 'm';
					}
					else
					{
						displayStr[6] = 'a';
						displayStr[7] = 'm';
					}
					curhr1 = HEX2BCD(curhr);
					
					displayStr[0] = (curhr1 >> 4) + '0';
					displayStr[1] = (curhr1 & 0xf ) + '0';
					displayStr[2] = (toggle==0)?' ':':';
					displayStr[3] = (curTime.minutes >> 4) + '0';
					displayStr[4] = (curTime.minutes & 0xf ) + '0';
					displayStr[5] = ' ';
					
					displayStr[8] = 0;
				}
				else
				{
					locked_sprintf_P( str, PSTR("RTC ERR"));
				}
				
				ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
				PrintLineInBox(LEFT,5,70,125,"CLOCK");
			}
			else
			{
				dispValSel++;
			}
		}
		
		if((dispValSel == (char)1) && ( value != ERROR_OK ))
		{
			ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, WHITE);
			PrintLineInBoxWOClear(LEFT,5,GLCD_PIXEL_X/2,125,"SYS ERR");
			locked_sprintf_P( str, PSTR("%d"), value );
		}
		else
		{
			dispValSel++;
		}
		
		dispValSel = (dispValSel + 1) %2;
	}

	return str;
}

///* formats the sting for output */
//static void FormatDispError( int8_t error )
//{
   //if (IsLCDDisplay())
      //locked_sprintf_P( formatedValue, PSTR("ER.%d"), error );
   //return;
//}

/* formats the sting for output */
static void FormatError( int8_t error )
{
   if (IsLCDDisplay())
      locked_sprintf_P( formatedValue, PSTR("E%d"), error );
   return;
}

/* formats the sting for output */
static char * FormatInteger( int16_t value)
{
	memset(displayStr,0,sizeof(displayStr));
	
   if (IsLCDDisplay())
   {
      locked_sprintf_P( displayStr, PSTR("%d"), value );
      return displayStr;
   }
   return NULL;
}

static char * Formatfloat( int16_t value)
{
	memset(displayStr,0,sizeof(displayStr));
	
	if (IsLCDDisplay())
	{
		dtostrf( (float)value/100, 4, 2, displayStr);
		return displayStr;
	}
	return NULL;
}

static char * FormatVersion()
{
	int softver = SOFTWARE_VERSION; 
    int major = softver/100;
	softver %= 100;
    int minor = softver/10;
	softver %= 10;
	int change = softver;
	
    locked_sprintf_P( formatedValue, PSTR("%d.%d.%d"), major, minor, change );

    return formatedValue;
}


char * FormatDP1OneLine( char * str, uint8_t error, int16_t value )
{
   if( error != ERROR_OK )
   {
      FormatError( error );
   }
   else
   {
	   FormatDP(GetParameterValue( DP1_UNIT ), value, GetParameterValue(DP1_RANGE), GetParameterValue(DP1_AREA_TYPE), GetParameterValue(DP1_AREA_LENGTH), GetParameterValue(DP1_AREA_WIDTH), GetParameterValue(DP1_AREA_RADIOUS));
   }
   locked_sprintf_P( str, PSTR("DP1(%S): %s"), formatedUnit, formatedValue );
   return str;
}

char * FormatTemperatureOneLine( char * str, uint8_t error, int16_t value, uint8_t decimal)
{
   FormatTemperature( value, decimal );
   if( error != ERROR_OK )
      FormatError( error );
   locked_sprintf_P( str, PSTR("TEMP(%S): %s"), formatedUnit, formatedValue );
   return str;
}

char * FormatHumidityOneLine( char * str, uint8_t error, int16_t value, uint8_t decimal )
{
   FormatHumidity( value, decimal );
   if( error != ERROR_OK )
      FormatError( error );
   locked_sprintf_P( str, PSTR("HUMI(%S): %s"), formatedUnit, formatedValue );
   return str;
}

const PARA_INFO paraTableLCD [] PROGMEM =
{
	{ DEVICE_ID,                        IsSerialEnabled,    NULL,                   "Device ID",          DeviceIdPara  },
	{ TEMP_FIRE_ALM_SET,                IsTemperatureEnabled,       NULL,           "FireAlmSet",   TempFireAlmSetPara  },
	{ TEMP_FIRE_ALM_TIME,                 IsTemperatureEnabled,       NULL,           "FireTimeSet(Sec)",  TempFireAlmTimePara  },
	{ DOOR_ALARM_TIME,                 NULL,       NULL,           "DoorAlmTime(Sec)",  DoorAlmTimePara  },
		
	{ DP1_UNIT,					   IsDP1Enabled,  NULL,           "DP1 Unit",           PressureAbsUnitPara  },
	{ DP1_AREA_TYPE,               IsMasterReset, IsDP1CFMEnabled,				"DP1 AREA TYPE",     DP1AreaTypePara  },
	{ DP1_AREA_LENGTH,             IsMasterReset, IsDP1CFMAreaSquare,				"DP1 AREA LENGTH",   DP1AreaLengthPara  },
	{ DP1_AREA_WIDTH,              IsMasterReset, IsDP1CFMAreaSquare,				"DP1 AREA WIDTH",    DP1AreaWidthPara  },
	{ DP1_AREA_RADIOUS,            IsMasterReset, IsDP1CFMAreaRound,				"DP1 AREA RAD",  DP1AreaRadiousPara  },
	{ DP1_ZERO_RNG,                IsDP1Enabled,  NULL,								"DP1 ZeroRng",   AbsPresZeroRangePara  },
	{ DP1_ZERO_ADJ,                IsDP1Enabled,  NULL,								"DP1 ZeroAdj",   AbsPresZeroAdjustPara  },
	{ DP1_MANP_THRESHOLD,          IsMasterReset,      IsDP1Enabled,      "DP1 Manp Threshold",  AbsPresManpThPara  },
	{ DP1_MANP_MAX_LIMIT,          IsMasterReset,      IsDP1Enabled,      "DP1 Manp Limit",  AbsPresManpLmtPara  },
	{ DP1_ALARM_SETUP,             IsAlarmEnabled,         IsDP1Enabled, "DP1 Alarm Setup",      PresAbsAlarmSetupPara },
	{ DP1_UPPER_ALARM_ON_LIMIT,    IsPresAbsAlarmEnabled,  IsDP1Enabled, "DP1 Upr Alarm On",     PresAbsUpperAlarmOnPara },
	{ DP1_UPPER_ALARM_OFF_LIMIT,   IsPresAbsAlarmEnabled,  IsDP1Enabled, "DP1 Upr Alarm Off",    PresAbsUpperAlarmOffPara  },
	{ DP1_LOWER_ALARM_ON_LIMIT,    IsPresAbsAlarmEnabled,  IsDP1Enabled, "DP1 Lwr Alarm On",     PresAbsLowerAlarmOnPara },
	{ DP1_LOWER_ALARM_OFF_LIMIT,   IsPresAbsAlarmEnabled,  IsDP1Enabled, "DP1 Lwr Alarm Off",    PresAbsLowerAlarmOffPara  },
	
	{ TEMPERATURE_UNIT,                 IsTemperatureEnabled,       NULL,           "Temperature Unit",   TemperatureUnitPara  },
	{ TEMP_ZERO_ADJ,                    IsTemperatureEnabled,       NULL,           "TempZeroAdjCnt",  TempZeroAdjustPara  },
	{ TEMP_ALARM_SETUP,                 IsAlarmEnabled,         IsTemperatureEnabled,      "Temp Alarm Setup",      TempAlarmSetupPara },
	{ TEMP_UPPER_ALARM_ON_LIMIT,        IsTempAlarmEnabled,     IsTemperatureEnabled,      "Temp Upr Alarm On",     TempUpperAlarmOnPara  },
	{ TEMP_UPPER_ALARM_OFF_LIMIT,       IsTempAlarmEnabled,     IsTemperatureEnabled,      "Temp Upr Alarm Off",    TempUpperAlarmOffPara  },
	{ TEMP_LOWER_ALARM_ON_LIMIT,        IsTempAlarmEnabled,     IsTemperatureEnabled,      "Temp Lwr Alarm On",     TempLowerAlarmOnPara  },
	{ TEMP_LOWER_ALARM_OFF_LIMIT,       IsTempAlarmEnabled,     IsTemperatureEnabled,      "Temp Lwr Alarm Off",    TempLowerAlarmOffPara  },
	
	{ HUMIDITY_UNIT,                    IsHumidityEnabled,          NULL,           "Humidity Unit",      HumidityUnitPara  },
	{ RH_ZERO_ADJ,                      IsHumidityEnabled,          NULL,           "RH ZeroAdjCnt",    HumidityZeroAdjustPara    },
	{ RH_ALARM_SETUP,                   IsAlarmEnabled,         IsHumidityEnabled,         "RH Alarm Setup",      HumidityAlarmSetupPara },
	{ RH_UPPER_ALARM_ON_LIMIT,          IsHumidityAlarmEnabled, IsHumidityEnabled,         "RH Upr Alarm On",     HumidityUpperAlarmOnPara  },
	{ RH_UPPER_ALARM_OFF_LIMIT,         IsHumidityAlarmEnabled, IsHumidityEnabled,         "RH Upr Alarm Off",    HumidityUpperAlarmOffPara  },
	{ RH_LOWER_ALARM_ON_LIMIT,          IsHumidityAlarmEnabled, IsHumidityEnabled,         "RH Lwr Alarm On",     HumidityLowerAlarmOnPara  },
	{ RH_LOWER_ALARM_OFF_LIMIT,         IsHumidityAlarmEnabled, IsHumidityEnabled,         "RH Lwr Alarm Off",    HumidityLowerAlarmOffPara  },
	
	{ RTC_ENABLE_DISABLE,               NULL,               NULL,                   "Real Time Clock",    RTCSetupPara },
	{ TIME_HOURS,                       IsRTCEnabled,       NULL,                   "Time: Hour",         TimeHourPara  },
	{ TIME_MINUTES,                     IsRTCEnabled,       NULL,                   "Time: Minute",       TimeMinutePara  },
	{ TIME_SECONDS,                     IsRTCEnabled,       NULL,                   "Time: Second",       TimeSecondPara  },
	{ DATE_YEAR,                        IsRTCEnabled,       NULL,                   "Date: Year",         DateYearPara  },
	{ DATE_MONTH,                       IsRTCEnabled,       NULL,                   "Date: Month",        DateMonthPara  },
	{ DATE_DAY,                         IsRTCEnabled,       NULL,                   "Date: Day",          DateDayPara  },
	
	{ BROADCAST_INTERVAL,               IsSerialEnabled,    IsBroadcastEnabled,     "BroadcastInterval", BroadcastIntervalPara  },
	{ BROADCAST_PORT,               IsSerialEnabled,    IsBroadcastEnabled,     "Broadcast Port", BroadcastPortPara  },
		
	{ BUZZER_ON_TIME,                   IsAlarmEnabled,             NULL,           "BuzzerONTime",     BuzzerOnPara },
	{ BUZZER_OFF_TIME,                  IsAlarmEnabled,             NULL,           "BuzzerOFFTime",    BuzzerOffPara },     
	{ BUZZER_DISABLE_TIME,              IsAlarmEnabled,             NULL,           "BuzzerDISTime",    BuzzerDisablePara },     
	
	{ SOFT_VERSION,                     NULL,                       NULL,           "Version",            VersionPara },
	
	{ TEMP_RH_SENS_TYPE,                IsMasterReset,      IsTempOrRHEnabled,      "Temp/RH Sens type",  TempRHSensTypePara  },	
	{ SERIAL_BAUD_RATE_VAL,             IsMasterReset,      IsSerialEnabled,        "Serial Baudrate",    SerialBaudratePara  },
	{ SERIAL_DATA_STOP_PARITY_VAL,      IsMasterReset,      IsSerialEnabled,        "Ser D:St:Par Bits",  SerialDataStopParityPara },
		
	{ AHU_ID,                     NULL,                       NULL,           "AHU ID",            AHUIdPara },	
	{ ROOM_ID,                     NULL,                       NULL,           "ROOM ID",            ROOMIdPara },
	
	{ DP1_SENS_TYPE,               IsMasterReset,      IsDP1Enabled, "DP1SensType",     PresAbsSensTypePara  },
	{ DP1_RANGE,                   IsMasterReset,      IsDP1Enabled, "DP1SenRng(Pa)",   PresAbsRange  },
	{ DP_RANGE1_ADJ,               CheckSerialSet,      0, "DP RANGE1 ADJ",     PresRange1AdjPara  },
	{ DP_RANGE2_ADJ,              CheckSerialSet,      0,  "DP RANGE2 ADJ",        PresRange2AdjPara  },
	{ DP_RANGE3_ADJ,             CheckSerialSet,      0,  "DP RANGE3 ADJ",       PresRange3AdjPara  },
	{ DP_RANGE4_ADJ,               CheckSerialSet,      0, "DP RANGE4 ADJ",     PresRange4AdjPara  },
	{ DP_RANGE5_ADJ,              CheckSerialSet,      0,  "DP RANGE5 ADJ",        PresRange5AdjPara  },
	{ DP_RANGE6_ADJ,             CheckSerialSet,      0,  "DP RANGE6 ADJ",       PresRange6AdjPara  },
	
	{ CUSTOMER_ID,                      NULL,                       NULL,           "CustomerID",         CustomerIDPara  },
	{ SERIAL_NO,                        NULL,                       NULL,           "SerialNo",           SerialNoPara  },
	{ CONFIG_SYSTEM,                    NULL,                       NULL,           "SystemConfig",       SystemConfigPara  },
	{ CFG_PASSWORD,                     IsMasterReset,      NotConfigPassword,      "Config Password",    ConfigPasswordPara  },
};


static unsigned char editorState = 0;
static int activePara = 0;
static PARA_INFO activeParaInfo;
static int activeParaValue;

// Change the edited value without 16-bit overflow (holding UP at 32767 used to wrap
// negative and jump to the minimum). The parameter handler then clamps to its own range.
static void StepParaValue( int step )
{
	int32_t v = (int32_t)activeParaValue + step;
	if( v > 32767 ) v = 32767;
	if( v < -32768 ) v = -32768;
	activeParaValue = (int)v;
}
static unsigned int keyTimer;

static void DisplayEditor(char keysPressed, char keys)
{
   uint8_t paraCnt, paraUpReq;
   char * dispStr;

   LCDControl = EDITOR;
   if( keys == (char)-1 && keysPressed == (char)-1 )
   {
      /* Display Keyboard start message */
      //OSSemaTakeEver(UIMutex);
      if (IsLCDDisplay())
      {
			uc1638_FillScreen(BLANK1);
			
			ks0xxx_SelectFont( Font_Verdana_10x24, ks0xxx_ReadFontData, BLACK);
			
			uc1638_GotoXY(systemXLoc, PARA_SETUP_Y_LOC);
			PrintLine(1, "PARAMETER EDITOR" );
			ks0xxx_DrawHoriLine(1, 24, 238, BLACK);
			uc1638_GotoXY(systemXLoc, 48);
			PrintLine(1, "Use PARA key to" );
			uc1638_GotoXY(systemXLoc, 72);
			PrintLine(1, "  select parameter" );
			uc1638_GotoXY(systemXLoc, 96);
			PrintLine(1, "UP/DN to change" );
			uc1638_GotoXY(systemXLoc, 120);
			PrintLine(1, "the parameter value" );
      }
      //OSSemaGive(UIMutex);
      editorState = 1;
      activePara = -1;
      keyTimer = 0;
      return;
   }

   if( editorState == (unsigned char)1 )
   {  // wait for setup key to be pressed. Ignore up/down keys until then
      if(( keysPressed & SETUP_KEY) == 0 )
         return;
      //OSSemaTakeEver(UIMutex);
      if (IsLCDDisplay())
      {
         uc1638_FillScreen(BLANK1);
         uc1638_GotoXY(systemXLoc, PARA_SETUP_Y_LOC);
         //ks0xxx_Puts_P(PSTR("PARAMETER EDITOR"));
		 PrintLine(1, "PARAMETER EDITOR" );
		 ks0xxx_DrawHoriLine(1, 24, 238, BLACK);
      }
      //OSSemaGive(UIMutex);
   }

   if(( keys & SETUP_KEY) != 0 )
   {
      paraUpReq = 0;
      if(( keysPressed & UP_KEY ) != 0 && ( keys & DOWN_KEY ) == 0 )
         paraUpReq = 1;
      if(( keysPressed & DOWN_KEY ) != 0 && ( keys & UP_KEY ) == 0 )
      {  // treat as setup key
         keysPressed |= SETUP_KEY;
         keysPressed &= ~DOWN_KEY;
      }

      if(( keysPressed & SETUP_KEY) != 0 || paraUpReq )
      { 
		 // Setup key pressed select next parameter
         for( paraCnt = 0; paraCnt < sizeof(paraTableLCD)/sizeof(paraTableLCD[0]); paraCnt++ )
         {
			 wdt_reset();
			 
            if( paraUpReq )
            {
               if( activePara )
                  activePara--;
               else
                  activePara = sizeof(paraTableLCD)/sizeof(paraTableLCD[0])-1;
            }
            else
            {
               activePara++;
               if(activePara >= sizeof(paraTableLCD)/sizeof(paraTableLCD[0]))
                  activePara = 0;
            }
   
            GetParaInfoByIndex( activePara, &activeParaInfo );
            if( activeParaInfo.priValidationFunction != NULL )
               if((*activeParaInfo.priValidationFunction)() == 0 )
                  continue;
            if( activeParaInfo.secValidationFunction != NULL )
               if((*activeParaInfo.secValidationFunction)() == 0 )
                  continue;
            break;
         }
         if( paraCnt == sizeof(paraTableLCD)/sizeof(paraTableLCD[0]))
         {  /* No parameter is required */
            StartDisplaySensorValues();
            return;
         }
         // Get and Display current Parameter value
         //OSSemaTakeEver(UIMutex);
         if (IsLCDDisplay())
         {
            uc1638_GotoXY(systemXLoc, PARA_INFO_Y_LOC);
            PrintLine(1, activeParaInfo.displayText );
            uc1638_GotoXY(systemXLoc, PARA_VAL_Y_LOC);
			
			PrintLine(1, ( * activeParaInfo.parameterHandler)( PARA_READ, &activeParaValue ));
         }
         //OSSemaGive(UIMutex);
   
         editorState = 2;
         return;
      }
      return;
   }

   if( editorState == (unsigned char)2 )
   {  
      // wait for at least one key is pressed
      if(( keysPressed & UP_KEY ) != 0 && ( keys & DOWN_KEY ) == 0 )
      {
		  StepParaValue(1);
         //OSSemaTakeEver(UIMutex);
         dispStr = ( * activeParaInfo.parameterHandler)( PARA_WRITE, &activeParaValue);  // Para value +
      }
      else if(( keys & UP_KEY ) == 0 && ( keysPressed & DOWN_KEY ) != 0 )
      {
		  StepParaValue(-1);
         //OSSemaTakeEver(UIMutex);
         dispStr = ( * activeParaInfo.parameterHandler)( PARA_WRITE, &activeParaValue);  // Para value -
      }
      else if(( keysPressed & UP_KEY ) != 0 && ( keysPressed & DOWN_KEY ) != 0 )
      {
		 //OSSemaTakeEver(UIMutex);
         dispStr = ( * activeParaInfo.parameterHandler)( PARA_DEFAULT, &activeParaValue);  // Para default
      }
      else
         return;

      // Display current Parameter value
      if (IsLCDDisplay())
      {
         uc1638_GotoXY(systemXLoc, PARA_INFO_Y_LOC);
         PrintLine( 1, activeParaInfo.displayText );
         uc1638_GotoXY(systemXLoc, PARA_VAL_Y_LOC);
         PrintLine( 1, dispStr );
      }
      //OSSemaGive(UIMutex);

      editorState = 3;
      return;
   }

   if( editorState == (unsigned char)3 )
   {
      if(( keysPressed & ( UP_KEY | DOWN_KEY )) != 0 )
         keyTimer = 0; // reset auto up/down timer counter

      if((( keysPressed & UP_KEY ) != 0 && ( keys & DOWN_KEY ) != 0 ) || (( keys & UP_KEY ) != 0 && ( keysPressed & DOWN_KEY ) != 0 ))
      {
         //OSSemaTakeEver(UIMutex);
		 dispStr = ( * activeParaInfo.parameterHandler)( PARA_DEFAULT, &activeParaValue );  // Para default
      }
      else if (( keys & DOWN_KEY ) == 0 && ( keys & UP_KEY ) == 0 )
         return;  // Do not do anything 
      else if (( keys & DOWN_KEY ) != 0 && ( keys & UP_KEY ) != 0 )
         return;  // Do not do anything 
      else if (( keysPressed & UP_KEY ) != 0)
      {
		 StepParaValue(1);
         //OSSemaTakeEver(UIMutex);
         dispStr = ( * activeParaInfo.parameterHandler)( PARA_WRITE, &activeParaValue);  // Para value +
      }
      else if(( keysPressed & DOWN_KEY ) != 0)
      {
		 StepParaValue(-1);
         //OSSemaTakeEver(UIMutex);
         dispStr = ( * activeParaInfo.parameterHandler)( PARA_WRITE, &activeParaValue);  // Para value -
      }
      else
      {
         // Process timeouts and set new values
         // increment timer counter
         keyTimer++;
         if(( keyTimer % ( KEY_AUTO_CHG_TIME/KEYBOARD_LOOP_TIME )) != 0 )
            return;   /* 300 ms update time */
         if(( keyTimer / ( KEY_AUTO_CHG_FAST_TIME/KEYBOARD_LOOP_TIME )) == 0 )
            paraCnt = 1;
         else if(( keyTimer / ( KEY_AUTO_CHG_FAST_TIME/KEYBOARD_LOOP_TIME )) == 1 )
            paraCnt = KEY_AUTO_FAST_STEP;
         else
            paraCnt = KEY_AUTO_FAST_STEP * KEY_AUTO_FAST_STEP;

         if (( keys & DOWN_KEY ) != 0 )
		 {
			StepParaValue(-(int)paraCnt);
		 }
         else
		 {
			StepParaValue((int)paraCnt);
		 }

		
         //OSSemaTakeEver(UIMutex);
         dispStr = ( * activeParaInfo.parameterHandler)( PARA_WRITE, &activeParaValue);  // Para default
      }
      // Display current Parameter value
      if (IsLCDDisplay())
      {
         uc1638_GotoXY(systemXLoc, PARA_INFO_Y_LOC);
         PrintLine( 1, activeParaInfo.displayText );
         uc1638_GotoXY(systemXLoc, PARA_VAL_Y_LOC);
         PrintLine( 1, dispStr );
      }
      //OSSemaGive(UIMutex);
   }
}

static void SetAllParametersToDefault(void)
{
	PARA_INFO paraInfoVal;
	uint8_t i;
	int16_t paraValue;

	for (i = 0; i < (uint8_t)(sizeof(paraTableLCD)/sizeof(paraTableLCD[0])); i++)
	{
		GetParaInfoByIndex( i, &paraInfoVal );
		(*paraInfoVal.parameterHandler)( PARA_DEFAULT, &paraValue );
	}
}


// returns error code as a function return and read value in paraValue
uint8_t GetParameter( char paraId, int *paraValue )
{
   PARA_INFO paraInfoVal;

   if( GetParaInfo( paraId, &paraInfoVal ) != 0 )
      return ERROR_PARA_INVALID;

	if( paraInfoVal.priValidationFunction != NULL && paraInfoVal.priValidationFunction != IsMasterReset)
      if((*paraInfoVal.priValidationFunction)() == 0 )
         return ERROR_PARA_NOT_SUPORTED;
   if( paraInfoVal.secValidationFunction != NULL && paraInfoVal.secValidationFunction != IsMasterReset)
      if((*paraInfoVal.secValidationFunction)() == 0 )
         return ERROR_PARA_NOT_SUPORTED;
		 
		 
   //if( paraInfoVal.priValidationFunction != NULL )
      //if((*paraInfoVal.priValidationFunction)() == 0 )
         //return ERROR_PARA_NOT_SUPORTED;
   //if( paraInfoVal.secValidationFunction != NULL )
      //if((*paraInfoVal.secValidationFunction)() == 0 )
         //return ERROR_PARA_NOT_SUPORTED;
   if( paraInfoVal.parameterHandler != NULL )
      (* paraInfoVal.parameterHandler)( PARA_READ_VALUE_ONLY, paraValue );

   return ERROR_OK;
}

uint8_t GetParameterStr( uint8_t paraId, uint8_t *parastr )
{
	uint16_t eepromAddr=EA_IP1_HIGH_NAME;
	
	switch(paraId)
	{
		case IP1_HIGH_NAME: eepromAddr = EA_IP1_HIGH_NAME;	break;
		case IP1_LOW_NAME:  eepromAddr = EA_IP1_LOW_NAME;	break;
		case IP2_HIGH_NAME: eepromAddr = EA_IP2_HIGH_NAME;	break;
		case IP2_LOW_NAME:  eepromAddr = EA_IP2_LOW_NAME;	break;
		case IP3_HIGH_NAME: eepromAddr = EA_IP3_HIGH_NAME;	break;
		case IP3_LOW_NAME:  eepromAddr = EA_IP3_LOW_NAME;	break;
		case IP4_HIGH_NAME: eepromAddr = EA_IP4_HIGH_NAME;	break;
		case IP4_LOW_NAME:  eepromAddr = EA_IP4_LOW_NAME;	break;
		case OP1_HIGH_NAME: eepromAddr = EA_OP1_HIGH_NAME;	break;
		case OP1_LOW_NAME:  eepromAddr = EA_OP1_LOW_NAME;	break;
		case OP2_HIGH_NAME: eepromAddr = EA_OP2_HIGH_NAME;	break;
		case OP2_LOW_NAME:  eepromAddr = EA_OP2_LOW_NAME;	break;
		case OP3_HIGH_NAME: eepromAddr = EA_OP3_HIGH_NAME;	break;
		case OP3_LOW_NAME:  eepromAddr = EA_OP3_LOW_NAME;	break;
		case OP4_HIGH_NAME: eepromAddr = EA_OP4_HIGH_NAME;	break;
		case OP4_LOW_NAME:  eepromAddr = EA_OP4_LOW_NAME;	break;
		
		default:	return ERROR_PARA_INVALID;		break;
	}
	
	OSReadEEPromBlock((uint8_t *)eepromAddr, parastr, 10);
	
	return ERROR_OK;
}

void SetParameterValue(int eepromAddr, int paraValue )
{
	OSWriteEEPromWord((unsigned int *)eepromAddr, paraValue);
}

// returns error code as a function return and actual set value in paraValue
uint8_t SetParameter( char paraId, int *paraValue )
{
   PARA_INFO paraInfoVal;

   if( GetParaInfo( paraId, &paraInfoVal ) != 0 )
      return ERROR_PARA_INVALID;

   if( paraInfoVal.priValidationFunction != NULL && paraInfoVal.priValidationFunction != IsMasterReset)
      if((*paraInfoVal.priValidationFunction)() == 0 )
         return ERROR_PARA_NOT_SUPORTED;
   if( paraInfoVal.secValidationFunction != NULL && paraInfoVal.secValidationFunction != IsMasterReset)
      if((*paraInfoVal.secValidationFunction)() == 0 )
         return ERROR_PARA_NOT_SUPORTED;
		 
   if( paraInfoVal.parameterHandler != NULL )
   {
		(* paraInfoVal.parameterHandler)( PARA_WRITE_VALUE_ONLY, paraValue );
   }
   return ERROR_OK;
}

// returns error code as a function return and actual set value in paraValue
uint8_t SetParameterStr( uint8_t paraId, uint8_t *parastr )
{
	uint16_t eepromAddr=EA_IP1_HIGH_NAME;
	
	switch(paraId)
	{
		case IP1_HIGH_NAME: eepromAddr = EA_IP1_HIGH_NAME;	break;
		case IP1_LOW_NAME:  eepromAddr = EA_IP1_LOW_NAME;	break;
		case IP2_HIGH_NAME: eepromAddr = EA_IP2_HIGH_NAME;	break;
		case IP2_LOW_NAME:  eepromAddr = EA_IP2_LOW_NAME;	break;
		case IP3_HIGH_NAME: eepromAddr = EA_IP3_HIGH_NAME;	break;
		case IP3_LOW_NAME:  eepromAddr = EA_IP3_LOW_NAME;	break;
		case IP4_HIGH_NAME: eepromAddr = EA_IP4_HIGH_NAME;	break;
		case IP4_LOW_NAME:  eepromAddr = EA_IP4_LOW_NAME;	break;
		case OP1_HIGH_NAME: eepromAddr = EA_OP1_HIGH_NAME;	break;
		case OP1_LOW_NAME:  eepromAddr = EA_OP1_LOW_NAME;	break;
		case OP2_HIGH_NAME: eepromAddr = EA_OP2_HIGH_NAME;	break;
		case OP2_LOW_NAME:  eepromAddr = EA_OP2_LOW_NAME;	break;
		case OP3_HIGH_NAME: eepromAddr = EA_OP3_HIGH_NAME;	break;
		case OP3_LOW_NAME:  eepromAddr = EA_OP3_LOW_NAME;	break;
		case OP4_HIGH_NAME: eepromAddr = EA_OP4_HIGH_NAME;	break;
		case OP4_LOW_NAME:  eepromAddr = EA_OP4_LOW_NAME;	break;
		default:	return ERROR_PARA_INVALID;		break;
	}
	
	OSWriteEEPromBlock((uint8_t *)eepromAddr, parastr, 10);
	
	return ERROR_OK;
}

int GetParameterValue( PARAMETER paraId )
{
   PARA_INFO paraInfoVal;
   int16_t value = 0;

   if( GetParaInfo( paraId, &paraInfoVal ) == 0 )
      (* paraInfoVal.parameterHandler)( PARA_READ_VALUE_ONLY, &value );

   return value;
}

/* Returns 0 if no error else returns 1 */
static uint8_t GetParaInfo( PARAMETER reqId, PARA_INFO * paraInfoVal )
{
   uint8_t * start, * storePtr;
   uint8_t i, id;

   for( i = 0, start = (uint8_t *)paraTableLCD; i < (uint8_t)(sizeof (paraTableLCD) / sizeof( PARA_INFO)); i++, start += sizeof( PARA_INFO))
   {
      id = pgm_read_byte( start );
      if( id == (uint8_t)reqId)
      {
         storePtr = (uint8_t *)paraInfoVal;
         for( i = 0; i < sizeof(PARA_INFO); i++, storePtr++, start++)
            *storePtr = pgm_read_byte( start );
         return 0;
      }
   }
   return 1;
}

/* Returns 0 if no error else returns 1 */
static uint8_t GetParaInfoByIndex( uint8_t index, PARA_INFO * paraInfoVal )
{
   uint8_t * start = (uint8_t *)paraTableLCD, *storePtr;
   uint8_t i;

   if( index >= (uint8_t)(sizeof (paraTableLCD) / sizeof( PARA_INFO)))
      return 1;

   start += (sizeof( PARA_INFO) * index);
   storePtr = (uint8_t *)paraInfoVal;
   for( i = 0; i < sizeof(PARA_INFO); i++, storePtr++, start++)
      *storePtr = pgm_read_byte( start );
   return 0;
}


static void ParameterOperationsOnLowSettings( uint8_t operationType, int16_t * paraValue, int16_t dfltValue, int16_t minValue, int16_t maxValParaId, unsigned int eepromAddr )
{
   int16_t temp;

   if( operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_READ )
   {
      *paraValue = ReadParaEeprom(eepromAddr);
	  return;
   }
   if(operationType == (uint8_t)PARA_DEFAULT )
   {
	   *paraValue = dfltValue;
   }
   else
   {
	   if(!gu16_AlarmLimitCheckDisableTimer)
	   {
		  if ( *paraValue <= minValue)
			 *paraValue = minValue;
		  temp = GetParameterValue(maxValParaId);
		  if ( *paraValue >= temp)
			 *paraValue = temp-1;
	   }
   }

	if(!isMasterReset)
	{
	   QueueParaEepromWrite(eepromAddr, *paraValue);
	}
	else
	{
		OSWriteEEPromWord((unsigned int *)eepromAddr, *paraValue);
	}
}


static void ParameterOperationsOnHighSettings( uint8_t operationType, int16_t * paraValue, int16_t dfltValue, int16_t maxValue, int16_t minValParaId, unsigned int eepromAddr )
{
   int16_t temp;

   if( operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_READ )
   {
      *paraValue = ReadParaEeprom(eepromAddr);
	  return;
   }
   else if(operationType == (uint8_t)PARA_DEFAULT )
   {
	   *paraValue = dfltValue;
   }
   else
   {
	   if(!gu16_AlarmLimitCheckDisableTimer)
	   {
		  temp = GetParameterValue(minValParaId);
		  if ( *paraValue <= temp)
			 *paraValue = temp+1;
		  if ( *paraValue > maxValue)
			 *paraValue = maxValue;
	   }
   }
   
   if(!isMasterReset)
   {
	   QueueParaEepromWrite(eepromAddr, *paraValue);
   }
   else
   {
	   OSWriteEEPromWord((unsigned int *)eepromAddr, *paraValue);
   }
}

static char * PresAbsUpperAlarmOffPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                    paraValue, 
                                    D_DP1_UPPER_ALARM_OFF_LIMIT, 
                                    MIN_DP1_UPPER_ALARM_OFF_LIMIT, 
                                    DP1_UPPER_ALARM_ON_LIMIT, 
                                    EA_DP1_UPPER_ALARM_OFF_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;

   return FormatDP1OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresAbsUpperAlarmOnPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                     paraValue, 
                                     D_DP1_UPPER_ALARM_ON_LIMIT, 
                                     MAX_DP1_UPPER_ALARM_ON_LIMIT, 
                                     DP1_UPPER_ALARM_OFF_LIMIT, 
                                     EA_DP1_UPPER_ALARM_ON_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatDP1OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresAbsLowerAlarmOffPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                    paraValue, 
                                    D_DP1_LOWER_ALARM_OFF_LIMIT, 
                                    MAX_DP1_LOWER_ALARM_OFF_LIMIT, 
                                    DP1_LOWER_ALARM_ON_LIMIT, 
                                    EA_DP1_LOWER_ALARM_OFF_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatDP1OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresAbsLowerAlarmOnPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                     paraValue, 
                                     D_DP1_LOWER_ALARM_ON_LIMIT, 
                                     MIN_DP1_LOWER_ALARM_ON_LIMIT, 
                                     DP1_LOWER_ALARM_OFF_LIMIT, 
                                     EA_DP1_LOWER_ALARM_ON_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatDP1OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * TempUpperAlarmOffPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                    paraValue, 
                                    D_TEMP_UPPER_ALARM_OFF_LIMIT, 
                                    MIN_TEMP_UPPER_ALARM_OFF_LIMIT, 
                                    TEMP_UPPER_ALARM_ON_LIMIT, 
                                    EA_TEMP_UPPER_ALARM_OFF_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatTemperatureOneLine( displayStr, ERROR_OK, *paraValue, 2 );
}

static char * TempUpperAlarmOnPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                     paraValue, 
                                     D_TEMP_UPPER_ALARM_ON_LIMIT, 
                                     MAX_TEMP_UPPER_ALARM_ON_LIMIT, 
                                     TEMP_UPPER_ALARM_OFF_LIMIT, 
                                     EA_TEMP_UPPER_ALARM_ON_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatTemperatureOneLine( displayStr, ERROR_OK, *paraValue, 2 );
}


static char * TempLowerAlarmOffPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                    paraValue, 
                                    D_TEMP_LOWER_ALARM_OFF_LIMIT, 
                                    MAX_TEMP_LOWER_ALARM_OFF_LIMIT, 
                                    TEMP_LOWER_ALARM_ON_LIMIT, 
                                    EA_TEMP_LOWER_ALARM_OFF_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatTemperatureOneLine( displayStr, ERROR_OK, *paraValue, 2 );
}

static char * TempLowerAlarmOnPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                     paraValue, 
                                     D_TEMP_LOWER_ALARM_ON_LIMIT, 
                                     MIN_TEMP_LOWER_ALARM_ON_LIMIT, 
                                     TEMP_LOWER_ALARM_OFF_LIMIT, 
                                     EA_TEMP_LOWER_ALARM_ON_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatTemperatureOneLine( displayStr, ERROR_OK, *paraValue, 2 );
}

static char * HumidityUpperAlarmOffPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                    paraValue, 
                                    D_RH_UPPER_ALARM_OFF_LIMIT, 
                                    MIN_RH_UPPER_ALARM_OFF_LIMIT, 
                                    RH_UPPER_ALARM_ON_LIMIT, 
                                    EA_RH_UPPER_ALARM_OFF_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatHumidityOneLine( displayStr, ERROR_OK, *paraValue, 2 );
}

static char * HumidityUpperAlarmOnPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                     paraValue, 
                                     D_RH_UPPER_ALARM_ON_LIMIT, 
                                     MAX_RH_UPPER_ALARM_ON_LIMIT, 
                                     RH_UPPER_ALARM_OFF_LIMIT, 
                                     EA_RH_UPPER_ALARM_ON_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatHumidityOneLine( displayStr, ERROR_OK, *paraValue, 2 );
}

static char * HumidityLowerAlarmOffPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                    paraValue, 
                                    D_RH_LOWER_ALARM_OFF_LIMIT, 
                                    MAX_RH_LOWER_ALARM_OFF_LIMIT, 
                                    RH_LOWER_ALARM_ON_LIMIT, 
                                    EA_RH_LOWER_ALARM_OFF_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatHumidityOneLine( displayStr, ERROR_OK, *paraValue, 2 );
}

static char * HumidityLowerAlarmOnPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                     paraValue, 
                                     D_RH_LOWER_ALARM_ON_LIMIT, 
                                     MIN_RH_LOWER_ALARM_ON_LIMIT, 
                                     RH_LOWER_ALARM_OFF_LIMIT, 
                                     EA_RH_LOWER_ALARM_ON_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatHumidityOneLine( displayStr, ERROR_OK, *paraValue, 2 );
}

static char * PressureAbsUnitPara( uint8_t operationType, int16_t * paraValue )
{
   return PressureUnitProcessPara( operationType, paraValue, D_DP1_UNIT, EA_DP1_UNIT );
}

static char * PressureUnitProcessPara( uint8_t operationType, int16_t * paraValue, int16_t dfltValue, unsigned int eepromAddr )
{
   ParameterOperationsSettingsRollover( operationType, paraValue, eepromAddr, dfltValue, PRESSURE_PSI, PRESSURE_CFM );

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
      if( *paraValue == PRESSURE_PSI )
         locked_sprintf_P( displayStr, PSTR("PSI"));
      else if( *paraValue == PRESSURE_MILIBAR )
         locked_sprintf_P( displayStr, PSTR("mBAR"));
      else if( *paraValue == PRESSURE_PA )
         locked_sprintf_P( displayStr, PSTR("Pa"));
      else if( *paraValue == PRESSURE_H2O_MM )
         locked_sprintf_P( displayStr, PSTR("mmWC"));
      else if( *paraValue == PRESSURE_H2O_INCH )
         locked_sprintf_P( displayStr, PSTR("iH2O"));
      else if( *paraValue == PRESSURE_HG_INCH )
         locked_sprintf_P( displayStr, PSTR("inHg"));
      else if( *paraValue == PRESSURE_PSF )
         locked_sprintf_P( displayStr, PSTR("PSF"));
      else if( *paraValue == PRESSURE_CFM )
         locked_sprintf_P( displayStr, PSTR("CFM"));
      else
         locked_sprintf_P( displayStr, PSTR("Unknown"));
   }
   return displayStr;
}

static char * TemperatureUnitPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettingsRollover( operationType, paraValue, EA_TEMPERATURE_UNIT, D_TEMPERATURE_UNIT, TEMPERATURE_DEG_C, TEMPERATURE_DEG_K );

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
      if( *paraValue == TEMPERATURE_DEG_C )
         locked_sprintf_P( displayStr, PSTR("*C"));
      else if( *paraValue == TEMPERATURE_DEG_F )
         locked_sprintf_P( displayStr, PSTR("*F"));
      else if( *paraValue == TEMPERATURE_DEG_K )
         locked_sprintf_P( displayStr, PSTR("*K"));
      else
         locked_sprintf_P( displayStr, PSTR("Unknown"));
   }
   return displayStr;
}

static char * HumidityUnitPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettingsRollover( operationType, paraValue, EA_HUMIDITY_UNIT, D_HUMIDITY_UNIT, HUMIDITY_RH, HUMIDITY_RH );

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
      if( *paraValue == HUMIDITY_RH )
         locked_sprintf_P( displayStr, PSTR("%%RH"));
      else
         locked_sprintf_P( displayStr, PSTR("Unknown"));
   }
   return displayStr;
}

static char * DP1AreaLengthPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettings(operationType,
	paraValue,
	EA_DP1_AREA_LENGTH,
	D_DP1_AREA_LEN,
	MIN_DP1_AREA,
	MAX_DP1_AREA );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	//sprintf( displayStr, "%d.%d mm", *paraValue / 1000, *paraValue % 1000 );
	sprintf( displayStr, "%d mm", *paraValue );
	return displayStr;
}

static char * DP1AreaWidthPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettings(operationType,
	paraValue,
	EA_DP1_AREA_WIDTH,
	D_DP1_AREA_WID,
	MIN_DP1_AREA,
	MAX_DP1_AREA );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	//sprintf( displayStr, "%d.%d", *paraValue / 1000, *paraValue % 1000 );
	sprintf( displayStr, "%d mm", *paraValue );
	return displayStr;
}

static char * DP1AreaRadiousPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettings(operationType,
	paraValue,
	EA_DP1_AREA_RADIOUS,
	D_DP1_AREA_RAD,
	MIN_DP1_AREA,
	MAX_DP1_AREA );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	//sprintf( displayStr, "%d.%d", *paraValue / 1000, *paraValue % 1000 );
	sprintf( displayStr, "%d mm", *paraValue );
	return displayStr;
}

static char * SerialBaudratePara(uint8_t operationType, int16_t * paraValue )
{
   const char * baudDisp;

   ParameterOperationsSettingsRollover( operationType, paraValue, EA_SERIAL_BAUD_RATE_VAL, D_SERIAL_BAUD_RATE_VAL, MIN_SERIAL_BAUD_RATE_VAL, MAX_SERIAL_BAUD_RATE_VAL );

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
      switch( *paraValue )
      {
         case SERIAL_BAUD_1200: baudDisp = PSTR("1200");
                                break;
         case SERIAL_BAUD_2400: baudDisp = PSTR("2400");
                                break;
         case SERIAL_BAUD_4800: baudDisp = PSTR("4800");
                                break;
         case SERIAL_BAUD_14400: baudDisp = PSTR("14400");
                                break;
         case SERIAL_BAUD_19200: baudDisp = PSTR("19200");
                                break;
         case SERIAL_BAUD_28800: baudDisp = PSTR("28800");
                                break;
         case SERIAL_BAUD_38400: baudDisp = PSTR("38400");
                                break;
         case SERIAL_BAUD_57600: baudDisp = PSTR("57600");
                                break;
         case SERIAL_BAUD_9600: 
         default:               baudDisp = PSTR("9600");
                                break;

      }
      locked_sprintf_P( displayStr, baudDisp);
   }
   return displayStr;
}

static char * SerialDataStopParityPara(uint8_t operationType, int16_t * paraValue )
{
   const char * parity;

   ParameterOperationsSettingsRollover( operationType, paraValue, EA_SERIAL_DATA_STOP_PARITY_VAL, D_SERIAL_DATA_STOP_PARITY_VAL, MIN_SERIAL_DATA_STOP_PARITY_VAL, MAX_SERIAL_DATA_STOP_PARITY_VAL );

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
      switch(( *paraValue & SERIAL_PARITY_MASK ) >> SERIAL_PARITY_SHIFT )
      {
         case SERIAL_PARITY_EVEN: parity = PSTR("EVEN");
                                  break;
         case SERIAL_PARITY_ODD:  parity = PSTR("ODD");
                                  break;
         case SERIAL_PARITY_NONE: 
         default:                 parity = PSTR("NONE");
                                  break;
      }
      locked_sprintf_P( displayStr, PSTR( "%d:%d:%S" ), ((*paraValue&SERIAL_DATA_BITS_MASK)>>SERIAL_DATA_BITS_SHIFT) + 5,
                                                 ((*paraValue&SERIAL_STOP_BIT_MASK)>>SERIAL_STOP_BIT_SHIFT)+1,
                                                 parity );
   }
   return displayStr;
}


static char * PresAbsRange(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetfloatVal( operationType, paraValue, EA_DP1_RANGE, D_DP1_RANGE, MIN_DP1_RANGE, MAX_DP1_RANGE );
	//
   //if(( operationType == (uint8_t)PARA_READ_VALUE_ONLY ) || ( operationType == (uint8_t)PARA_READ ))
   //{
      //*paraValue = OSReadEEPromWord((unsigned int *)EA_DP1_RANGE);
	  //return FormatInteger( *paraValue );
   //}
   //else if( operationType == (uint8_t)PARA_WRITE || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY)
   //{
         //if( *paraValue < 0 )
            //*paraValue = 0; // roll over
   //}
   //else if( operationType == (uint8_t)PARA_DEFAULT )
   //{
	   //*paraValue = D_DP1_RANGE;
   //}
   //
   //OSWriteEEPromWord((unsigned int *)EA_DP1_RANGE, *paraValue);
   //
   //if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      //return NULL;
   //return FormatInteger( *paraValue );
}

static char * TimeHourPara( uint8_t operationType, int16_t * paraValue )
{
   return ParameterOperationsTimeValues( 0x3f, 0x0, operationType, paraValue, D_TIME_HOURS, MIN_TIME_HOURS, MAX_TIME_HOURS, RTC_TIMEHOUR_REG );
}

static char * TimeMinutePara( uint8_t operationType, int16_t * paraValue )
{
   return ParameterOperationsTimeValues( 0x7f, 0, operationType, paraValue, D_TIME_MINUTES, MIN_TIME_MINUTES, MAX_TIME_MINUTES, RTC_TIMEMIN_REG );
}

static char * TimeSecondPara( uint8_t operationType, int16_t * paraValue )
{
   return ParameterOperationsTimeValues( 0x7f, 0, operationType, paraValue, D_TIME_SECONDS, MIN_TIME_SECONDS, MAX_TIME_SECONDS, RTC_TIMESEC_REG );
}

static uint8_t BcdRegToBin( uint8_t regAddr, uint8_t mask, uint8_t dflt )
{
   uint8_t temp;
   if( RTCReadTimeDateRegister(regAddr, &temp ) != ERROR_OK )
      return dflt;
   temp &= mask;
   return (temp & 0xf) + ( temp >> 4 ) * 10;
}

// Days in the month currently stored in the RTC (years 2000-2099)
static uint8_t RTCDaysInMonth( void )
{
   static const uint8_t days[12] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
   uint8_t month = BcdRegToBin( RTC_DATE_MONTH_REG, 0x1f, 1 );
   uint8_t year = BcdRegToBin( RTC_DATE_YEAR_REG, 0xff, 0 );

   if( month < 1 || month > 12 ) return 31;
   if( month == 2 && ( year % 4 ) == 0 ) return 29;
   return days[month-1];
}

// After month/year change, pull the day back if it no longer exists (e.g. 31 -> Feb)
static void RTCClampDay( uint8_t operationType )
{
   uint8_t day, maxDay;

   if( operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_READ )
      return;
   day = BcdRegToBin( RTC_DATE_DATE_REG, 0x3f, 1 );
   maxDay = RTCDaysInMonth();
   if( day > maxDay )
      RTCWriteTimeDateRegister( RTC_DATE_DATE_REG, ((maxDay / 10) << 4 ) + maxDay % 10 );
}

static char * DateYearPara( uint8_t operationType, int16_t * paraValue )
{
   char * str = ParameterOperationsTimeValues( 0xff, 0x00, operationType, paraValue, D_DATE_YEAR, MIN_DATE_YEAR, MAX_DATE_YEAR, RTC_DATE_YEAR_REG );
   RTCClampDay( operationType );
   return str;
}

static char * DateMonthPara( uint8_t operationType, int16_t * paraValue )
{
   char * str = ParameterOperationsTimeValues( 0x1f, 0x0, operationType, paraValue, D_DATE_MONTH, MIN_DATE_MONTH, MAX_DATE_MONTH, RTC_DATE_MONTH_REG );
   RTCClampDay( operationType );
   return str;
}

static char * DateDayPara( uint8_t operationType, int16_t * paraValue )
{
   // Upper limit follows the current month instead of always 31
   uint8_t maxDay = MAX_DATE_DAY;
   if( operationType != (uint8_t)PARA_READ_VALUE_ONLY && operationType != (uint8_t)PARA_READ )
      maxDay = RTCDaysInMonth();
   return ParameterOperationsTimeValues( 0x3f, 0x0, operationType, paraValue, D_DATE_DAY, MIN_DATE_DAY, maxDay, RTC_DATE_DATE_REG );
}


static char * ParameterOperationsTimeValues( uint8_t mask, uint8_t maskWriteVal, uint8_t operationType, int16_t * paraValue, int16_t dfltValue, int8_t minValue, int8_t maxValue, uint8_t regAddr )
{
   uint8_t temp;

   if( operationType == (uint8_t)PARA_READ_VALUE_ONLY )
   {
      *paraValue = dfltValue;
       if( RTCReadTimeDateRegister(regAddr, &temp ) == ERROR_OK )
       {
          temp = temp & mask;
          *paraValue = (temp & 0xf) + ( temp >> 4 ) * 10;
       }
   }
   else if( operationType == (uint8_t)PARA_READ )
   {
      *paraValue = dfltValue;
       if( RTCReadTimeDateRegister(regAddr, &temp ) == ERROR_OK )
       {
          temp = temp & mask;
          *paraValue = (temp & 0xf) + ( temp >> 4 ) * 10;
       }
   }
   else
   {
      if( operationType == (uint8_t)PARA_DEFAULT )
         *paraValue = dfltValue;

      if ( *paraValue < minValue)
         *paraValue = maxValue;   // Roll over
      if ( *paraValue > maxValue)
         *paraValue = minValue;   // Roll over

      temp = *paraValue;
      temp = ((temp / 10) << 4 ) + temp % 10;
      RTCWriteTimeDateRegister(regAddr, temp | maskWriteVal);
      *paraValue = dfltValue;
      if( RTCReadTimeDateRegister(regAddr, &temp ) == ERROR_OK )
      {
         temp = temp & mask;
         *paraValue = (temp & 0xf) + ( temp >> 4 ) * 10;
      }
   }
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}


static char * ConfigPasswordPara(uint8_t operationType, int16_t * paraValue )
{
   if( operationType == (uint8_t)PARA_READ_VALUE_ONLY )
   {
      *paraValue = configPassword;
   }
   else if( operationType == (uint8_t)PARA_WRITE || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY)
      configPassword = *paraValue;
   else 
      *paraValue = configPassword;

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}

static char * CustomerIDPara(uint8_t operationType, int16_t * paraValue )
{
   if( operationType == (uint8_t)PARA_READ_VALUE_ONLY )
   {
      *paraValue = ~OSReadEEPromWord((unsigned int *)EA_CUSTOMER_ID);
   }
   else if( IsConfigPassword() && (operationType == (uint8_t)PARA_WRITE || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY))
      OSWriteEEPromWord((unsigned int *)EA_CUSTOMER_ID, ~(*paraValue));
   else
      *paraValue = ~OSReadEEPromWord((unsigned int *)EA_CUSTOMER_ID);

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}

static char * SerialNoPara(uint8_t operationType, int16_t * paraValue )
{
   if( operationType == (uint8_t)PARA_READ_VALUE_ONLY )
   {
      *paraValue = ~OSReadEEPromWord((unsigned int *)EA_SERIAL_NO);
   }
   else if( IsConfigPassword() && (operationType == (uint8_t)PARA_WRITE || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY))
      OSWriteEEPromWord((unsigned int *)EA_SERIAL_NO, ~(*paraValue));
   else
      *paraValue = ~OSReadEEPromWord((unsigned int *)EA_SERIAL_NO);

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}


static char * SystemConfigPara(uint8_t operationType, int16_t * paraValue )
{
   if( operationType == (uint8_t)PARA_READ_VALUE_ONLY )
   {
      *paraValue = ~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM);
   }
   else if( IsConfigPassword() && (operationType == (uint8_t)PARA_WRITE || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY))
   {
      OSWriteEEPromWord((unsigned int *)EA_CONFIG_SYSTEM, ~(*paraValue));
   }
   else
   {
      *paraValue = ~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM);
   }

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}

//static char * DisplayIntervalPara(uint8_t operationType, int16_t * paraValue )
//{
   //return ParaSetIntegerVal( operationType, paraValue, EA_DISPLAY_INTERVAL, D_DISPLAY_INTERVAL, MIN_DISPLAY_INTERVAL, MAX_DISPLAY_INTERVAL );
//}
//
//static char * DisplayModePara(uint8_t operationType, int16_t * paraValue )
//{
   //return ParaSetIntegerVal( operationType, paraValue, EA_DISPLAY_MODE, 1, 0, 1 );
//}

//static char * LogggingIntervalPara(uint8_t operationType, int16_t * paraValue )
//{
//   return ParaSetIntegerVal( operationType, paraValue, EA_LOGGING_INTERVAL, D_LOGGING_INTERVAL, MIN_LOGGING_INTERVAL, MAX_LOGGING_INTERVAL );
//}

static char * ROOMIdPara(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetIntegerVal1( operationType, paraValue, EA_ROOM_ID, 0, 0, 1000 );
}

static char * AHUIdPara(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetIntegerVal1( operationType, paraValue, EA_AHU_ID, 0, 0, 1000 );
}

static char * DeviceIdPara(uint8_t operationType, int16_t * paraValue )
{
   return ParaSetIntegerVal( operationType, paraValue, EA_DEVICE_ID, D_DEVICE_ID, MIN_DEVICE_ID, MAX_DEVICE_ID );
}

static char * BroadcastIntervalPara(uint8_t operationType, int16_t * paraValue )
{
   return ParaSetIntegerVal( operationType, paraValue, EA_BROADCAST_INTERVAL, D_BROADCAST_INTERVAL, MIN_BROADCAST_INTERVAL, MAX_BROADCAST_INTERVAL );
}

static char * BroadcastPortPara(uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettingsRollover( operationType, paraValue, EA_BROADCAST_PORT, D_BROADCAST_PORT, MIN_BROADCAST_PORT, MAX_BROADCAST_PORT );

	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	if (IsLCDDisplay())
	{
		if( *paraValue == 0 )
		locked_sprintf_P( displayStr, PSTR("Wifi"));
		else if( *paraValue == 1 )
		locked_sprintf_P( displayStr, PSTR("Zigbee"));
		else if( *paraValue == 2 )
		locked_sprintf_P( displayStr, PSTR("Both"));
		else
		locked_sprintf_P( displayStr, PSTR("Unknown"));
	}
	return displayStr;
}

static char * AbsPresZeroRangePara(uint8_t operationType, int16_t * paraValue )
{
   return ParaSetfloatVal( operationType, paraValue, EA_DP1_ZERO_RNG, D_DP1_ZERO_RNG, MIN_DP1_ZERO_RNG, MAX_DP1_ZERO_RNG );
}

static char * AbsPresManpThPara(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetfloatVal( operationType, paraValue, EA_DP1_MANP_THRESHOLD, 0, 0, 1000 );	
}

static char * AbsPresManpLmtPara(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetfloatVal( operationType, paraValue, EA_DP1_MANP_MAX_LIMIT, 0, GetParameterValue(DP1_MANP_THRESHOLD), GetParameterValue(DP1_RANGE) );
}

static char * AbsPresZeroAdjustPara(uint8_t operationType, int16_t * paraValue )
{
   return ParaSetfloatVal( operationType, paraValue, EA_DP1_ZERO_ADJ, D_DP1_ZERO_ADJ, MIN_DP1_ZERO_ADJ, MAX_DP1_ZERO_ADJ );
}

static char * PresRange1AdjPara(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetfloatVal( operationType, paraValue, EA_DP_RANGE1_ADJ, 0, -1000, 1000 );
}

static char * PresRange2AdjPara(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetfloatVal( operationType, paraValue, EA_DP_RANGE2_ADJ, 0, -1000, 1000 );
}

static char * PresRange3AdjPara(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetfloatVal( operationType, paraValue, EA_DP_RANGE3_ADJ, 0, -1000, 1000 );
}

static char * PresRange4AdjPara(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetfloatVal( operationType, paraValue, EA_DP_RANGE4_ADJ, 0, -1000, 1000 );
}

static char * PresRange5AdjPara(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetfloatVal( operationType, paraValue, EA_DP_RANGE5_ADJ, 0, -1000, 1000 );
}

static char * PresRange6AdjPara(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetfloatVal( operationType, paraValue, EA_DP_RANGE6_ADJ, 0, -1000, 1000 );
}

static char * HumidityZeroAdjustPara(uint8_t operationType, int16_t * paraValue )
{
   //if( operationType == PARA_DEFAULT )
      //return FormatInteger( *paraValue );
   return ParaSetIntegerVal( operationType, paraValue, EA_RH_ZERO_ADJ, D_RH_ZERO_ADJ, MIN_RH_ZERO_ADJ, MAX_RH_ZERO_ADJ );
}

static char * TempZeroAdjustPara(uint8_t operationType, int16_t * paraValue )
{
   //if( operationType == PARA_DEFAULT )
      //return FormatInteger( *paraValue );
   return ParaSetIntegerVal( operationType, paraValue, EA_TEMP_ZERO_ADJ, D_TEMP_ZERO_ADJ, MIN_TEMP_ZERO_ADJ, MAX_TEMP_ZERO_ADJ );
}

static char * TempFireAlmSetPara(uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsOnHighSettings(operationType,
	paraValue,
	5000,
	10000,
	TEMP_UPPER_ALARM_ON_LIMIT,
	EA_TEMP_FIRE_ALM_SET );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	return FormatTemperatureOneLine( displayStr, ERROR_OK, *paraValue, 2 );
}

static char * TempFireAlmTimePara(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetIntegerVal( operationType, paraValue, EA_TEMP_FIRE_TIME_SET, 60, 15, 3600 );
}

static char * DoorAlmTimePara(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetIntegerVal( operationType, paraValue, EA_DOOR_ALM_TIME_SET, 60, 0, 3600 );
}


static char * VersionPara(uint8_t operationType, int16_t * paraValue )
{
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
   {
      *paraValue = SOFTWARE_VERSION;
      return NULL;
   }
   return FormatVersion();
}

static char * BuzzerOnPara( uint8_t operationType, int16_t * paraValue )
{
   return ParaSetIntegerVal( operationType, paraValue, EA_BUZZER_ON_TIME, D_BUZZER_ON_TIME, MIN_BUZZER_ON_TIME, MAX_BUZZER_ON_TIME );
}


static char * BuzzerOffPara( uint8_t operationType, int16_t * paraValue )
{
   return ParaSetIntegerVal( operationType, paraValue, EA_BUZZER_OFF_TIME, D_BUZZER_OFF_TIME, MIN_BUZZER_OFF_TIME, MAX_BUZZER_OFF_TIME );
}

static char * BuzzerDisablePara( uint8_t operationType, int16_t * paraValue )
{
   return ParaSetIntegerVal( operationType, paraValue, EA_BUZZER_DISABLE_TIME, D_BUZZER_DISABLE_TIME, MIN_BUZZER_DISABLE_TIME, MAX_BUZZER_DISABLE_TIME );
}

static char * TempRHSensTypePara( uint8_t operationType, int16_t * paraValue )
{
   //ParameterOperationsSettingsRolloverNoDefault( operationType, paraValue, EA_TEMP_RH_SENS_TYPE, 0, 3 );
	ParameterOperationsSettingsRollover( operationType, paraValue, EA_TEMP_RH_SENS_TYPE, TEMP_RH_SENS_SHT25, 0, 2 );

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
       switch( *paraValue )
       {
          case TEMP_RH_SENS_SHT25: locked_sprintf_P( displayStr, PSTR("SHT25"));
                                   break;
          //case TEMP_RH_SENS_SHT35: locked_sprintf_P( displayStr, PSTR("SHT35"));
                                   //break;
          case TEMP_RH_SENS_IDT_HS3100: 
                                   locked_sprintf_P( displayStr, PSTR("IDT3100"));
                                   break;
          case TEMP_RH_SENS_NONE: 
          default:                 locked_sprintf_P( displayStr, PSTR("NONE"));
                                   break;
       }
   }
   return displayStr;
}

static char * PresAbsSensTypePara(uint8_t operationType, int16_t * paraValue )
{
   //ParameterOperationsSettingsRolloverNoDefault( operationType, paraValue, EA_DP1_SENS_TYPE, 0, 3 )
	ParameterOperationsSettingsRollover( operationType, paraValue, EA_DP1_SENS_TYPE, PRES_SENS_SM9543, 0, 3 );
	
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
       switch( *paraValue )
       {                  
          case PRES_SENS_SM9543:    locked_sprintf_P( displayStr, PSTR("SM9543"));
                                    break;
          case PRES_SENS_XGZP6891D:   locked_sprintf_P( displayStr, PSTR("XGZP6891D"));
                                    break;
		  case PRES_SENS_WF200DP:   locked_sprintf_P( displayStr, PSTR("WF200DP"));
									 break;
          case PRES_SENS_NONE: 
          default:                  locked_sprintf_P( displayStr, PSTR("NONE"));
                                    break;
       }
   }
   return displayStr;
}

static char * DP1AreaTypePara(uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettingsRollover( operationType, paraValue, EA_DP1_AREA_TYPE, PRES_AREA_SQUARE, 0, 1 );
	
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	if (IsLCDDisplay())
	{
		switch( *paraValue )
		{
			case PRES_AREA_SQUARE:    locked_sprintf_P( displayStr, PSTR("SQUARE"));
			break;
			case PRES_AREA_ROUND:     locked_sprintf_P( displayStr, PSTR("ROUND"));
			break;
			default:                  locked_sprintf_P( displayStr, PSTR("UNKNOWN"));
			break;
		}
	}
	return displayStr;
}

static char * PresAbsAlarmSetupPara( uint8_t operationType, int16_t * paraValue )
{
   return AlarmSetupPara( operationType, paraValue, EA_DP1_ALARM_SETUP );
}

static char * TempAlarmSetupPara( uint8_t operationType, int16_t * paraValue )
{
   return AlarmSetupPara( operationType, paraValue, EA_TEMP_ALARM_SETUP );
}

static char * HumidityAlarmSetupPara( uint8_t operationType, int16_t * paraValue )
{
   return AlarmSetupPara( operationType, paraValue, EA_RH_ALARM_SETUP );
}

static char * RTCSetupPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettingsRollover( operationType, paraValue, EA_RTC_SETUP, RTC_ENABLED, 0, 1 );

	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	if (IsLCDDisplay())
	{
		switch( *paraValue )
		{
			case RTC_DISABLED: locked_sprintf_P( displayStr, PSTR("DISABLED"));
			break;
			case RTC_ENABLED: locked_sprintf_P( displayStr, PSTR("ENABLED"));
			break;
			default:          locked_sprintf_P( displayStr, PSTR("UNKOWN"));
			break;
		}
	}
	return displayStr;
}

static char * AlarmSetupPara( uint8_t operationType, int16_t * paraValue, unsigned int eeAddress )
{
   ParameterOperationsSettingsRollover( operationType, paraValue, eeAddress, ALARM_ENB_BUZ_ON, 0, 2 );

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
      switch( *paraValue )
      {
         case ALARM_DISABLED: locked_sprintf_P( displayStr, PSTR("DISABLED"));
                                  break;
         case ALARM_ENB_BUZ_OFF: locked_sprintf_P( displayStr, PSTR("ENB_BUZ_OFF"));
                                  break;
         case ALARM_ENB_BUZ_ON:  locked_sprintf_P( displayStr, PSTR("ENB_BUZ_ON"));
                                  break;
         default:                 locked_sprintf_P( displayStr, PSTR("UNKOWN"));
                                  break;
      }
   }
   return displayStr;

}

static void ParameterOperationsSettings( uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal, int16_t minVal, int16_t maxVal )
{
   if(( operationType == (uint8_t)PARA_READ_VALUE_ONLY ) || ( operationType == (uint8_t)PARA_READ ))
   {
      *paraValue = ReadParaEeprom(eepromAddr);
	  return;
   }
   else if( operationType == (uint8_t)PARA_DEFAULT )
   {
	   *paraValue = dfltVal;
   }
   else
   {
      if( *paraValue < minVal )
      *paraValue = minVal;
      else if ( *paraValue > maxVal )
      *paraValue = maxVal;
   }

   if(!isMasterReset)
   {
	   QueueParaEepromWrite(eepromAddr, *paraValue);
   }
   else
   {
	   OSWriteEEPromWord((unsigned int *)eepromAddr, *paraValue);
   }
}


static void ParameterOperationsSettingsRollover( uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal, int16_t minVal, int16_t maxVal )
{
   if(( operationType == (uint8_t)PARA_READ_VALUE_ONLY ) || ( operationType == (uint8_t)PARA_READ ))
   {
      *paraValue = ReadParaEeprom(eepromAddr);
	  return;
   }
   else if( operationType == (uint8_t)PARA_DEFAULT )
   {
	   *paraValue = dfltVal;
   }
   else
   {
      if( *paraValue < minVal )
         *paraValue = maxVal; 
      else if ( *paraValue > maxVal )
         *paraValue = minVal;
   }

   if(!isMasterReset)
   {
	   QueueParaEepromWrite(eepromAddr, *paraValue);
   }
   else
   {
	   OSWriteEEPromWord((unsigned int *)eepromAddr, *paraValue);
   }
}

//static void ParameterOperationsSettingsRolloverNoDefault( uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t minVal, int16_t maxVal )
//{
	//if(( operationType == (uint8_t)PARA_READ_VALUE_ONLY ) || ( operationType == (uint8_t)PARA_READ ))
	//{
		//*paraValue = OSReadEEPromWord((unsigned int *)eepromAddr);
	//}
	//else
	//{
		//if( *paraValue < minVal )
		//*paraValue = maxVal;
		//else if ( *paraValue > maxVal )
		//*paraValue = minVal;
//
		//eeprom_address = eepromAddr;
		//eeprom_value = *paraValue;
		//eeprom_write_timeout = 2;
		//
		////OSWriteEEPromWord((unsigned int *)eepromAddr, *paraValue);
	//}
//}


static char * ParaSetIntegerVal(uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal, int16_t minVal, int16_t maxVal )
{
   ParameterOperationsSettings( operationType, paraValue, eepromAddr, dfltVal, minVal, maxVal );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
	  
   if( operationType == (uint8_t)PARA_DEFAULT )
	*paraValue = dfltVal;
	
   return FormatInteger(*paraValue);
}

static char * ParaSetfloatVal(uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal, int16_t minVal, int16_t maxVal )
{
	ParameterOperationsSettings( operationType, paraValue, eepromAddr, dfltVal, minVal, maxVal );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	
	if( operationType == (uint8_t)PARA_DEFAULT )
	*paraValue = dfltVal;
	
	return Formatfloat(*paraValue);
}


static char * ParaSetIntegerVal1(uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal, int16_t minVal, int16_t maxVal )
{
	ParameterOperationsSettings( operationType, paraValue, eepromAddr, dfltVal, minVal, maxVal );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	
	if( operationType == (uint8_t)PARA_DEFAULT )
	*paraValue = dfltVal;
	
	if (IsLCDDisplay())
	{
		if( *paraValue == 0 )
			locked_sprintf_P( displayStr, PSTR("NONE"));
		else
			FormatInteger(*paraValue);
	}

	return displayStr;
}

static uint8_t IsMasterReset()
{
   return (SETDFLTSW_IN == 0)? (uint8_t)1 : (uint8_t)0;
}


static uint8_t IsConfigPassword()
{
   if( configPassword == MASTER_CONFIG_PASSWORD )
      return 1;
   
   // check zero or 0xffff
   if( GetParameterValue( CONFIG_SYSTEM ) == 0 && configPassword == CONFIG_PASSWORD )
      return 1;
   if( GetParameterValue( CONFIG_SYSTEM ) == -1 && configPassword == CONFIG_PASSWORD )
      return 1;
   return  0;
}


static uint8_t NotConfigPassword()
{
   if( configPassword == MASTER_CONFIG_PASSWORD )
      return 0;
   
   return (configPassword != CONFIG_PASSWORD)? 1 : 0;
}


void UIResourceLock()
{
   //OSSemaTakeEver(UIMutex);
}

void UIResourceUnlock()
{
   //OSSemaGive(UIMutex);
}


void BuzzerOn()
{
   BUZZER_ON;
}

void BuzzerOff()
{
   BUZZER_OFF;
}

int d_pres_abs_upper_alarm_off_limit()
{ 
	return 1490;
}
int d_pres_abs_upper_alarm_on_limit()       
{  
	return 1510;
}
             
int d_pres_abs_lower_alarm_off_limit()      
{  
	return 510;
}

int d_pres_abs_lower_alarm_on_limit()       
{  
	return 490;
}
             
int d_pres_diff_upper_alarm_off_limit()     
{  
   return 1490;
}
             
int d_pres_diff_upper_alarm_on_limit()      
{  
   return 1510;
}
             
int d_pres_diff_lower_alarm_off_limit()     
{  
   return 510;
}
            
int d_pres_diff_lower_alarm_on_limit()      
{  
   return 490;
}

int d_pres_diff3_upper_alarm_off_limit()
{
	return 1490;
}

int d_pres_diff3_upper_alarm_on_limit()
{
	return 1510;
}

int d_pres_diff3_lower_alarm_off_limit()
{
	return 510;
}

int d_pres_diff3_lower_alarm_on_limit()
{
	return 490;
}

int d_pres_abs_pid_setpoint_limit()
{
	return 500;
}

int d_pres_diff_pid_setpoint_limit()
{
	return 500;
}

            
int d_temp_upper_alarm_off_limit()          
{  
   return 2500;
}
             
int d_temp_upper_alarm_on_limit()           
{  
   return 2501;
}
             
int d_temp_lower_alarm_off_limit()          
{  
   return 2100;
}
             
int d_temp_lower_alarm_on_limit()           
{  
   return 2099;
}
             
int d_rh_upper_alarm_off_limit()            
{  
   return 5500;
}
             
int d_rh_upper_alarm_on_limit()             
{  
   return 5501;
}
             
int d_rh_lower_alarm_off_limit()            
{  
   return 4500;
}
             
int d_rh_lower_alarm_on_limit()             
{  
   return 4499;
}

int d_part_mc_upper_alarm_off_limit()
{
	return 4000;
}

int d_part_mc_upper_alarm_on_limit()
{
	return 5000;
}

int d_part_voc_upper_alarm_off_limit()
{
	return 3500;
}

int d_part_voc_upper_alarm_on_limit()
{
	return 4000;
}   

int d_part_nox_upper_alarm_off_limit()
{
	return 20;
}

int d_part_nox_upper_alarm_on_limit()
{
	return 25;
}           

int d_pres_abs_out_low_pres()               
{  
	return 500;
}
             
int d_pres_abs_out_high_pres()              
{
   return 1000;
}
             
int d_pres_diff_out_low_pres()             
{  
	return 500;
}
             
int d_pres_diff_out_high_pres()             
{  
	return 1000;
}
             
int d_temp_out_low_temp()                   
{  
   return 2100;
}
             
int d_temp_out_high_temp()                  
{  
   return 2500;
}
             
int d_rh_out_low_rh()                       
{  
   return 4500;
}
             
int d_rh_out_high_rh()                      
{  
   return 5500;
}
             

int min_pres_abs_upper_alarm_off_limit()    
{  
   return 0;
}
    
int min_pres_abs_lower_alarm_on_limit()     
{  
   return 0;
}
    
int min_pres_diff_upper_alarm_off_limit()   
{  
   return 0;
}

int min_pres_diff_lower_alarm_on_limit()    
{  
   return 0;
}

int min_pres_diff3_upper_alarm_off_limit()
{
	return 0;
}

int min_pres_diff3_lower_alarm_on_limit()
{
	return 0;
}

int min_temp_upper_alarm_off_limit()        
{  
   return -4000;
}
    
int min_temp_lower_alarm_on_limit()         
{  
   return -4000;
}
    
int min_rh_upper_alarm_off_limit()          
{  
   return 0;
}
    
int min_rh_lower_alarm_on_limit()           
{  
   return 0;
}

int min_part_mc_upper_alarm_off_limit()
{
	return 0;
}

int min_part_voc_upper_alarm_off_limit()
{
	return 0;
}

int min_part_nox_upper_alarm_off_limit()
{
	return 0;
}

int min_abs_pres_zero_adj()                 
{  
   return -GetParameterValue(DP1_RANGE);
}
                
int min_diff_pres_zero_adj()                
{  
   return -GetParameterValue(DP2_RANGE);
}

int min_diff_pres3_zero_adj()
{
	return -GetParameterValue(DP3_RANGE);
}
                
int min_temp_zero_adj()                     
{  
   return -2000;
}
                
int min_rh_zero_adj()                       
{  
   return -2000;
}
                

int min_pres_abs_out_low_pres()             
{  
   return 0;
}
    
int min_pres_diff_out_low_pres()            
{  
   return 0;
}
    
int min_temp_out_low_temp()                 
{  
   return -4000;
}
    
int min_rh_out_low_rh()                     
{  
   return 0;
}
    
int max_pres_abs_upper_alarm_on_limit()     
{  
   return GetParameterValue(DP1_RANGE);
}

int max_pres_abs_lower_alarm_off_limit()    
{  
   return GetParameterValue(DP1_RANGE);
}

int max_pres_diff_upper_alarm_on_limit()    
{  
   return GetParameterValue(DP2_RANGE);
}

int max_pres_diff_lower_alarm_off_limit()   
{  
   return GetParameterValue(DP2_RANGE);
}

int max_pres_diff3_upper_alarm_on_limit()
{
	return GetParameterValue(DP3_RANGE);
}

int max_pres_diff3_lower_alarm_off_limit()
{
	return GetParameterValue(DP3_RANGE);
}

int max_temp_upper_alarm_on_limit()         
{  
   return 12380;
}

int max_temp_lower_alarm_off_limit()        
{  
   return 12380;
}

int max_rh_upper_alarm_on_limit()           
{  
   return 10000;
}

int max_rh_lower_alarm_off_limit()          
{  
   return 10000;
}

int max_part_mc_upper_alarm_on_limit()
{
	return 10000;
}

int max_part_voc_upper_alarm_on_limit()
{
	return 5000;
}

int max_part_nox_upper_alarm_on_limit()
{
	return 50;
}

int max_abs_pres_zero_rng()                 
{  
   return GetParameterValue(DP1_RANGE);
}
                
int max_diff_pres_zero_rng()                
{  
   return GetParameterValue(DP2_RANGE);
}
                
int max_diff_pres3_zero_rng()
{
	return GetParameterValue(DP3_RANGE);
}
 
int max_abs_pres_zero_adj()                 
{  
   return GetParameterValue(DP1_RANGE);
}
                
int max_diff_pres_zero_adj()                
{  
   return GetParameterValue(DP2_RANGE);
}

int max_diff_pres3_zero_adj()
{
	return GetParameterValue(DP3_RANGE);
}
                
int max_temp_zero_adj()                     
{  
   return 2000;
}
                
int max_rh_zero_adj()                       
{  
   return 2000;
}
                

int max_pres_abs_out_high_pres()            
{  
   return GetParameterValue(DP1_RANGE);
}

int max_pres_diff_out_high_pres()           
{  
   return GetParameterValue(DP2_RANGE);
}

int max_temp_out_high_temp()                
{  
   return 12380;
}

int max_rh_out_high_rh()                    
{  
   return 10000;
}


static uint8_t IsTempOrRHEnabled()
{
    return(IsTemperatureEnabled() || IsHumidityEnabled());
}

static uint8_t IsTempAndRHEnabled()
{
	return(IsTemperatureEnabled() && IsHumidityEnabled());
}


static uint8_t IsPresAbsAlarmEnabled()
{
   return IsAlarmEnabled() && GetParameterValue(DP1_ALARM_SETUP) != ALARM_DISABLED;
}

static uint8_t IsTempAlarmEnabled()
{
   return IsAlarmEnabled() && GetParameterValue(TEMP_ALARM_SETUP) != ALARM_DISABLED;
}

static uint8_t IsHumidityAlarmEnabled()
{
   return IsAlarmEnabled() && GetParameterValue(RH_ALARM_SETUP) != ALARM_DISABLED;
}

static uint8_t IsRTCEnabled()
{
	return GetParameterValue(RTC_ENABLE_DISABLE) != RTC_DISABLED;
}

//uint8_t GetDisplayPara(void)
//{
	//return 0;
//}

//#pragma GCC pop_options



