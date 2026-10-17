#include <avr/io.h>        // This defers to avr/io.h for GCC
#include <stdlib.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "os_api.h"
#include "error.h"
#include "DeviceIO.h"
#include "HardwareInfo.h"
#include "TempRHSens.h"
#include "UserInterface.h"
#include "SerialInterface.h"
#include "DAC7573.h"
#include "locked_printf.h"
#include "PressureSens.h"
#include "eepromAdr.h"

#define TRUE  1
#define FALSE 0

#define SENSOR_1  0
#define SENSOR_2  1

static SENSOR_VALUE sensorVal[TOTAL_VAL_INDEX];
uint8_t dp_sw_factor_add[3]={0};

// Kalman filter structure
typedef struct {
	float Q;       // Process noise covariance
	float R;       // Measurement noise covariance
	float X;       // State estimate
	float P;       // Estimate covariance
	float K;       // Kalman gain
} KalmanFilter;
static KalmanFilter Kalmanfilter[TOTAL_VAL_INDEX] = {0};
	
void Kalman_Init(KalmanFilter *kf, float q, float r, float initial_estimate);
int Kalman_Update(KalmanFilter *kf, float measurement);

// Initialize the Kalman filter
void Kalman_Init(KalmanFilter *kf, float q, float r, float initial_estimate)
{
	kf->Q = q;          // Process noise covariance
	kf->R = r;          // Measurement noise covariance
	kf->X = initial_estimate;  // Initial estimate
	kf->P = 1000.0;     // Large initial covariance: first measurement is taken almost as-is
	                    // (with 1.0 the first output was only ~91% of the reading, from 0)
}

// Update the Kalman filter with a new measurement
int Kalman_Update(KalmanFilter *kf, float measurement)
{
	// Prediction update
	kf->P += kf->Q;

	// Measurement update
	kf->K = kf->P / (kf->P + kf->R);
	kf->X += kf->K * (measurement - kf->X);
	kf->P *= (1 - kf->K);

	return (int)kf->X;
}


	
//static OSSemaMutex DeviceValueMutex;

static void AveragePara( uint8_t SenNo, uint8_t error, unsigned int rawVal, int convertedVal );

static void DoIOPortMapping();

static ALARMS alarmOut;
static ALARMS oldAlarmOut;
static ALARMS buzzerOut;
static LEDS ledState;
static SENSOR_VALUE senVal;
//static sen5x_measured_value_t sen55;

//#pragma GCC push_options
//#pragma GCC optimize ("O0")

/*
   Device IO Task
*/

void setDooralarm(void)
{
	alarmOut.alarm.door=1;
}

void clearDooralarm(void)
{
	alarmOut.alarm.door=0;
}

#define WINDOW_SIZE  9   // must be odd, <= 39

int lastParaValue[TOTAL_VAL_INDEX] = {0};
static uint8_t ignoreCnter[TOTAL_VAL_INDEX]={0};
static int buffer[TOTAL_VAL_INDEX][WINDOW_SIZE]={0};
static int sorted[TOTAL_VAL_INDEX][WINDOW_SIZE]={0};   // temp buffer (global, no stack use)

static unsigned char index[TOTAL_VAL_INDEX] = {0};
static unsigned char count[TOTAL_VAL_INDEX] = {0};

int median_filter(uint8_t para, int new_sample)
{
	unsigned char i, j;
	int key;

	// Insert into circular buffer
	buffer[para][index[para]++] = new_sample;
	if (index[para] >= WINDOW_SIZE)
	index[para] = 0;

	if (count[para] < WINDOW_SIZE)
	count[para]++;

	// Copy active elements into sorted array
	for (i = 0; i < count[para]; i++)
	sorted[para][i] = buffer[para][i];

	// Insertion sort (in-place on sorted[])
	for (i = 1; i < count[para]; i++) {
		key = sorted[para][i];
		j = i;

		while (j > 0 && sorted[para][j - 1] > key) {
			sorted[para][j] = sorted[para][j - 1];
			j--;
		}
		sorted[para][j] = key;
	}

	// Return median
	return sorted[para][count[para] >> 1]; // divide by 2
}


#define SPIKE_LIMIT			1000	// max step accepted between samples
#define SPIKE_MAX_REJECT	4		// after this many rejected samples (1 s) accept the new level
static uint8_t spikeRejectCnt[TOTAL_VAL_INDEX]={0};

// Median + spike filter, then publish via AveragePara() when publish != 0.
// Returns the accepted value (the last accepted one if this sample was rejected).
// Jumps larger than SPIKE_LIMIT (either direction) are ignored for up to SPIKE_MAX_REJECT
// samples; before, only upward jumps were checked and the value could freeze forever.
static int SpikeFilter( uint8_t idx, uint8_t retVal, unsigned int rawVal, int convertedValue, uint8_t publish )
{
	int diff;
	
	convertedValue = median_filter(idx, convertedValue);
	
	if(ignoreCnter[idx]<15)
	{
		if(publish) AveragePara(idx, retVal, rawVal, convertedValue );
		lastParaValue[idx]=convertedValue;
		ignoreCnter[idx]++;
		return convertedValue;
	}
	
	diff = convertedValue-lastParaValue[idx];
	if((diff < SPIKE_LIMIT) && (diff > -SPIKE_LIMIT))
	{
		convertedValue = convertedValue + (0.02 * diff);
	}
	else if(spikeRejectCnt[idx] < SPIKE_MAX_REJECT)
	{
		spikeRejectCnt[idx]++;
		return lastParaValue[idx];	// rejected sample, keep last accepted value
	}
	
	spikeRejectCnt[idx]=0;
	if(publish) AveragePara(idx, retVal, rawVal, convertedValue );
	lastParaValue[idx]=convertedValue;
	return convertedValue;
}


#define DP_SW_FACT_DIVISION		10
static int16_t su16_dpAlmdiff[3]={0},su16_convertedValue[3]={0},LastDPManpThreshold[3]={0},LastDPManLimit[3]={0};
static uint8_t gu8_dp_sw_factor_add_cnt[3]={0};
	
void clearDp1ManpPara(void)
{
	su16_convertedValue[0] = 0;
	gu8_dp_sw_factor_add_cnt[0] = 0;
}	

void clearDp2ManpPara(void)
{
	su16_convertedValue[1] = 0;
	gu8_dp_sw_factor_add_cnt[1] = 0;
}

void clearDp3ManpPara(void)
{
	su16_convertedValue[2] = 0;
	gu8_dp_sw_factor_add_cnt[2] = 0;
}

void DEVICEIO_FUNC_NAME( void )
{
	uint8_t retVal;
	int value;
	int convertedValue;
	static uint16_t TempfireAlmTimer=0,TempAlmTimer=0;
	
	systemError = ERROR_OK;

	if (IsDP1Enabled())
	{
		retVal = GetSensorDP1( &value );
		
		//------------------------------------------------------------------------------
		// Convert the value to unit
		if ( retVal == ERROR_OK )
		{
			sensorVal[DP1_VAL_INDEX].rawValue = value;
			convertedValue = value - GetParameterValue(DP1_ZERO_ADJ);
		}
		else
		{
			sensorVal[DP1_VAL_INDEX].rawValue = value = convertedValue = 0;
			ignoreCnter[DP1_VAL_INDEX]=0;
			Kalman_Init(&Kalmanfilter[DP1_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
			for(uint8_t i=0; i<WINDOW_SIZE; i++) 
			{
				buffer[DP1_VAL_INDEX][i]=0;
				sorted[DP1_VAL_INDEX][i]=0;
			}
		}
		
		convertedValue = SpikeFilter(DP1_VAL_INDEX, retVal, value, convertedValue, 0);	// published once below, after the offset
		//------------------------------------------------------------------------------
		
		if(GetParameterValue(DP1_MANP_THRESHOLD) != LastDPManpThreshold[0])
		{
			LastDPManpThreshold[0] = GetParameterValue(DP1_MANP_THRESHOLD);
			clearDp1ManpPara();
		}
		
		if(GetParameterValue(DP1_MANP_MAX_LIMIT) != LastDPManLimit[0])
		{
			LastDPManLimit[0] = GetParameterValue(DP1_MANP_MAX_LIMIT);
			clearDp1ManpPara();
		}
		
		if(GetParameterValue(DP1_MANP_THRESHOLD)>100)
		{
			su16_dpAlmdiff[0] = GetParameterValue(DP1_MANP_MAX_LIMIT);
			
			if(dp_sw_factor_add[0])
			{
				if(convertedValue > GetParameterValue(DP1_MANP_THRESHOLD))
				{
					if(gu8_dp_sw_factor_add_cnt[0]<DP_SW_FACT_DIVISION)
					{
						su16_convertedValue[0] += (su16_dpAlmdiff[0]/DP_SW_FACT_DIVISION);
						gu8_dp_sw_factor_add_cnt[0]++;
					}
				}
				else
				{
					if(gu8_dp_sw_factor_add_cnt[0])
					{
						su16_convertedValue[0] -= (su16_dpAlmdiff[0]/DP_SW_FACT_DIVISION);
						gu8_dp_sw_factor_add_cnt[0]--;
					}
				}
				
				dp_sw_factor_add[0] = 0;
			}
			convertedValue += su16_convertedValue[0];
		}
		
		AveragePara(DP1_VAL_INDEX, retVal, value, convertedValue );
	}
		
	oldAlarmOut.alarmByte = alarmOut.alarmByte;
	if (IsAlarmEnabled())
	{
		if( IsDP1Enabled() && GetParameterValue(DP1_ALARM_SETUP) != ALARM_DISABLED)
		{
			GetPareValue(DP1_VAL_INDEX, &senVal);//GetDP1( &senVal );
			
			if( senVal.errorCode == ERROR_OK )
			{
				if( senVal.convertedValue < GetParameterValue(DP1_LOWER_ALARM_ON_LIMIT))
					alarmOut.alarm.absPressLow = 1;
				if( senVal.convertedValue > GetParameterValue(DP1_UPPER_ALARM_ON_LIMIT))
					alarmOut.alarm.absPressHigh = 1;
				if( senVal.convertedValue > GetParameterValue(DP1_LOWER_ALARM_OFF_LIMIT))
					alarmOut.alarm.absPressLow = 0;
				if( senVal.convertedValue < GetParameterValue(DP1_UPPER_ALARM_OFF_LIMIT))
					alarmOut.alarm.absPressHigh = 0;

				ledState.led.absPressAlarm = alarmOut.alarm.absPressLow | alarmOut.alarm.absPressHigh;
				ledState.led.absPressOk = ~alarmOut.alarm.absPressHigh;
			}
			else
			{
				ledState.led.absPressOk = ledState.led.absPressAlarm = 0;
			}
		}
		else
		{
			ledState.led.absPressOk = ledState.led.absPressAlarm = 0;
			alarmOut.alarm.absPressLow = 0;
			alarmOut.alarm.absPressHigh = 0;
		}
	
		//if (IsTemperatureEnabled() && GetParameterValue(TEMP_ALARM_SETUP) != ALARM_DISABLED)
		if (IsTemperatureEnabled())
		{
			GetPareValue(TEMPERATURE_VAL_INDEX, &senVal);//GetTemperature( &senVal );
			if( senVal.errorCode == ERROR_OK )
			{
				if(GetParameterValue(TEMP_ALARM_SETUP) != ALARM_DISABLED)
				{
					if( senVal.convertedValue < GetParameterValue(TEMP_LOWER_ALARM_ON_LIMIT))
					alarmOut.alarm.tempLow = 1;
					if( senVal.convertedValue > GetParameterValue(TEMP_UPPER_ALARM_ON_LIMIT))
					{
						alarmOut.alarm.tempHigh = 1;
						TempAlmTimer++;
						if(TempAlmTimer>60)
						{
							TempAlmTimer=0;
							OUTPUT5_HIGH;
						}
					}
						
					if( senVal.convertedValue > GetParameterValue(TEMP_LOWER_ALARM_OFF_LIMIT))
					alarmOut.alarm.tempLow = 0;
					if( senVal.convertedValue < GetParameterValue(TEMP_UPPER_ALARM_OFF_LIMIT))
					{
						alarmOut.alarm.tempHigh = 0;
						OUTPUT5_LOW;
					}
						

					ledState.led.tempAlarm = alarmOut.alarm.tempLow | alarmOut.alarm.tempHigh;
					ledState.led.tempOk = ~alarmOut.alarm.tempHigh;
				}
				else
				{
					ledState.led.tempOk = ledState.led.tempAlarm = 0;
				}
					
				if( senVal.convertedValue >= GetParameterValue(TEMP_FIRE_ALM_SET))
				{
					TempfireAlmTimer++;
					if(TempfireAlmTimer>=GetParameterValue(TEMP_FIRE_ALM_TIME))
					{
						alarmOut.alarm.fire = 1;
						TempfireAlmTimer=0;
					}
				}
				else
				{
					TempfireAlmTimer=0;
					alarmOut.alarm.fire = 0;
				}
			}
			else
			{
				ledState.led.tempOk = ledState.led.tempAlarm = 0;
				alarmOut.alarm.fire = 0;
			}
		}
		else
		{
			ledState.led.tempOk = ledState.led.tempAlarm = 0;
			alarmOut.alarm.tempLow = 0;
			alarmOut.alarm.tempHigh = 0;
			alarmOut.alarm.fire = 0;
		}

		if (IsHumidityEnabled() && GetParameterValue(RH_ALARM_SETUP) != ALARM_DISABLED)
		{
			GetPareValue(HUMIDITY_VAL_INDEX, &senVal);//GetHumidity( &senVal );
			if( senVal.errorCode == ERROR_OK )
			{
				if( senVal.convertedValue < GetParameterValue(RH_LOWER_ALARM_ON_LIMIT))
				alarmOut.alarm.humidityLow = 1;
				if( senVal.convertedValue > GetParameterValue(RH_UPPER_ALARM_ON_LIMIT))
				alarmOut.alarm.humidityHigh = 1;
				if( senVal.convertedValue > GetParameterValue(RH_LOWER_ALARM_OFF_LIMIT))
				alarmOut.alarm.humidityLow = 0;
				if( senVal.convertedValue < GetParameterValue(RH_UPPER_ALARM_OFF_LIMIT))
				alarmOut.alarm.humidityHigh = 0;

				ledState.led.humidityAlarm = alarmOut.alarm.humidityLow | alarmOut.alarm.humidityHigh;
				ledState.led.humidityOk = ~alarmOut.alarm.humidityHigh;
			}
			else
			{
				ledState.led.humidityOk = ledState.led.humidityAlarm = 0;
			}
		}
		else
		{
			ledState.led.humidityOk = ledState.led.humidityAlarm = 0;
			alarmOut.alarm.humidityLow = 0;
			alarmOut.alarm.humidityHigh = 0;
		}
			
		if( IsSerialEnabled() && IsBroadcastEnabled() && oldAlarmOut.alarmByte != alarmOut.alarmByte )
		{
			oldAlarmOut.alarmByte ^= alarmOut.alarmByte;
			if( oldAlarmOut.alarm.absPressHigh )
			{  // Change in state
				SerialPrintAlarm( ALARM_UPPER_DP1, alarmOut.alarm.absPressHigh );
			}
			if( oldAlarmOut.alarm.absPressLow )
			{  // Change in state
				SerialPrintAlarm( ALARM_LOWER_DP1, alarmOut.alarm.absPressLow );
			}
			if( oldAlarmOut.alarm.diffPressHigh )
			{  // Change in state
				SerialPrintAlarm( ALARM_UPPER_DP2, alarmOut.alarm.diffPressHigh );
			}
			if( oldAlarmOut.alarm.diffPressLow )
			{  // Change in state
				SerialPrintAlarm( ALARM_LOWER_DP2, alarmOut.alarm.diffPressLow );
			}
			if( oldAlarmOut.alarm.diffPress3High )
			{  // Change in state
				SerialPrintAlarm( ALARM_UPPER_DP3, alarmOut.alarm.diffPress3High );
			}
			if( oldAlarmOut.alarm.diffPress3Low )
			{  // Change in state
				SerialPrintAlarm( ALARM_LOWER_DP3, alarmOut.alarm.diffPress3Low );
			}
			if( oldAlarmOut.alarm.tempHigh )
			{  // Change in state
				SerialPrintAlarm( ALARM_UPPER_TEMPERATURE, alarmOut.alarm.tempHigh );
			}
			if( oldAlarmOut.alarm.tempLow )
			{  // Change in state
				SerialPrintAlarm( ALARM_LOWER_TEMPERATURE, alarmOut.alarm.tempLow );
			}
			if( oldAlarmOut.alarm.humidityHigh )
			{  // Change in state
				SerialPrintAlarm( ALARM_UPPER_HUMIDITY, alarmOut.alarm.humidityHigh );
			}
			if( oldAlarmOut.alarm.humidityLow )
			{  // Change in state
				SerialPrintAlarm( ALARM_LOWER_HUMIDITY, alarmOut.alarm.humidityLow );
			}
		}

		//Set buzzer and relay out values based on alarm setup parameter
		if( GetParameterValue(DP1_ALARM_SETUP) == ALARM_ENB_BUZ_ON )
		{
			buzzerOut.alarm.absPressHigh = alarmOut.alarm.absPressHigh;
			buzzerOut.alarm.absPressLow = alarmOut.alarm.absPressLow;
		}
		else
		{
			buzzerOut.alarm.absPressHigh = 0;
			buzzerOut.alarm.absPressLow = 0;
		}
		 
		if( GetParameterValue(DP2_ALARM_SETUP) == ALARM_ENB_BUZ_ON )
		{
			buzzerOut.alarm.diffPressHigh = alarmOut.alarm.diffPressHigh;
			buzzerOut.alarm.diffPressLow = alarmOut.alarm.diffPressLow;
		}
		else
		{
			buzzerOut.alarm.diffPressHigh = 0;
			buzzerOut.alarm.diffPressLow = 0;
		}
			
		if( GetParameterValue(DP3_ALARM_SETUP) == ALARM_ENB_BUZ_ON )
		{
			buzzerOut.alarm.diffPress3High = alarmOut.alarm.diffPress3High;
			buzzerOut.alarm.diffPress3Low = alarmOut.alarm.diffPress3Low;
		}
		else
		{
			buzzerOut.alarm.diffPress3High = 0;
			buzzerOut.alarm.diffPress3Low = 0;
		}
			
		if( GetParameterValue(TEMP_ALARM_SETUP) == ALARM_ENB_BUZ_ON )
		{
			buzzerOut.alarm.tempHigh = alarmOut.alarm.tempHigh;
			buzzerOut.alarm.tempLow = alarmOut.alarm.tempLow;
		}
		else
		{
			buzzerOut.alarm.tempHigh = 0;
			buzzerOut.alarm.tempLow = 0;
		}
			
		if( GetParameterValue(RH_ALARM_SETUP) == ALARM_ENB_BUZ_ON )
		{
			buzzerOut.alarm.humidityHigh = alarmOut.alarm.humidityHigh;
			buzzerOut.alarm.humidityLow = alarmOut.alarm.humidityLow;
		}
		else
		{
			buzzerOut.alarm.humidityHigh = 0;
			buzzerOut.alarm.humidityLow = 0;
		}
			
		buzzerOut.alarm.fire = alarmOut.alarm.fire;
		buzzerOut.alarm.door = alarmOut.alarm.door;
	}

	//LED_PORT = ~ledState.ledByte;
	if(ledState.led.humidityAlarm) LED_HUM_ALM_ON; else LED_HUM_ALM_OFF;
	if(ledState.led.absPressOk) LED_ABD_PRES_OK_ON; else LED_ABD_PRES_OK_OFF;
	if(ledState.led.absPressAlarm) LED_ABD_PRES_ALM_ON; else LED_ABD_PRES_ALM_OFF;
	if(ledState.led.diffPressOk) LED_DP2_OK_ON; else LED_DP2_OK_OFF;
	if(ledState.led.diffPressAlarm) LED_DP2_ALM_ON; else LED_DP2_ALM_OFF;
	if(ledState.led.tempOk) LED_TEMP_OK_ON; else LED_TEMP_OK_OFF;
	if(ledState.led.tempAlarm) LED_TEMP_ALM_ON; else LED_TEMP_ALM_OFF;
	if(ledState.led.humidityOk) LED_HUM_OK_ON; else LED_HUM_OK_OFF;
	if(ledState.led.humidityAlarm) LED_HUM_ALM_ON; else LED_HUM_ALM_OFF;
}
//#pragma GCC pop_options

static void DoIOPortMapping()
{
   if(alarmOut.alarm.fire==1) return;
   //if(alarmOut.alarm.door==1) return;
	
   uint8_t inputs = INPUT_PORT_IN & 0x1F;
   if(DOOR_SENSE) inputs |= PIN5_bm;

   int mapping, pol = 0;
   
   //mapping = GetParameterValue(OUTPUT1_MAPPING);
   //if( mapping )
   //{
      //mapping = mapping - 1;  // Normalize to 0 numbering from 1
      //pol = mapping % 2;  // Last bet as polarity, pol = 0 for normal, 1 for reverse polarity
      //mapping /= 2;  // Discard last bit
	  //mapping = 0x1 << mapping;   // generate mask to read input
      //pol = pol * mapping;  // Set polarity mask
	  //if(( inputs ^ pol ) & mapping )  // Normalize input based on output polarity
	     //OUTPUT1_HIGH;
      //else
         //OUTPUT1_LOW;
   //}
   mapping = GetParameterValue(OUTPUT2_MAPPING);
   if( mapping )
   {
      mapping = mapping - 1;  // Normalize to 0 numbering from 1
      pol = mapping % 2;  // Last bet as polarity, pol = 0 for normal, 1 for reverse polarity
      mapping /= 2;  // Discard last bit
	  mapping = 0x1 << mapping;   // generate mask to read input
      pol = pol * mapping;  // Set polarity mask
	  if(( inputs ^ pol ) & mapping )  // Normalize input based on output polarity
	     OUTPUT2_HIGH; 
      else
         OUTPUT2_LOW;
   }
   mapping = GetParameterValue(OUTPUT3_MAPPING);
   if( mapping )
   {
      mapping = mapping - 1;  // Normalize to 0 numbering from 1
      pol = mapping % 2;  // Last bet as polarity, pol = 0 for normal, 1 for reverse polarity
      mapping /= 2;  // Discard last bit
	  mapping = 0x1 << mapping;   // generate mask to read input
      pol = pol * mapping;  // Set polarity mask
	  if(( inputs ^ pol ) & mapping )  // Normalize input based on output polarity
	     OUTPUT3_HIGH; 
      else
         OUTPUT3_LOW;
   }
   mapping = GetParameterValue(OUTPUT4_MAPPING);
   if( mapping )
   {
      mapping = mapping - 1;  // Normalize to 0 numbering from 1
      pol = mapping % 2;  // Last bet as polarity, pol = 0 for normal, 1 for reverse polarity
      mapping /= 2;  // Discard last bit
	  mapping = 0x1 << mapping;   // generate mask to read input
      pol = pol * mapping;  // Set polarity mask
	  if(( inputs ^ pol ) & mapping )  // Normalize input based on output polarity
	     OUTPUT4_HIGH;  
      else
         OUTPUT4_LOW;
   }
}


void SetOutput( uint8_t outValue, uint8_t * retVal )
{
   int mapping;

   //mapping = GetParameterValue(OUTPUT1_MAPPING);
   //if( !mapping )
   //{
	  //if( outValue & 0x1 )
	     //OUTPUT1_HIGH;
	  //else
	     //OUTPUT1_LOW;
   //}
   mapping = GetParameterValue(OUTPUT2_MAPPING);
   if( !mapping )
   {
	  if( outValue & 0x2 )
	     OUTPUT2_HIGH;
	  else
	     OUTPUT2_LOW;
   }
   mapping = GetParameterValue(OUTPUT3_MAPPING);
   if( !mapping )
   {
	  if( outValue & 0x4 )
	     OUTPUT3_HIGH;
	  else
	     OUTPUT3_LOW;
   }
   mapping = GetParameterValue(OUTPUT4_MAPPING);
   if( !mapping )
   {
	  if( outValue & 0x8 )
	     OUTPUT4_HIGH;
	  else
	     OUTPUT4_LOW;
   }
   *retVal = (OUTPUT_PORT & 0x0F); 
}

void GetOutput( uint8_t * retVal )
{
   *retVal = (OUTPUT_PORT & 0x0F); 
}

void GetInput( uint8_t * retVal )
{
	uint8_t inputs = INPUT_PORT_IN & 0x1F;
	if(DOOR_SENSE) inputs |= PIN5_bm;
	
   *retVal = inputs; 
}

uint16_t tempDelay = 1000;

void TEMPRHIO_FUNC_NAME( void )
{
	uint8_t retVal = -1, checkSum;
	unsigned int value, value1;
	int convertedValue;
	
	if( GetParameterValue(TEMP_RH_SENS_TYPE) == TEMP_RH_SENS_SHT25 )
	{
		if (IsTemperatureEnabled())
		{   
			retVal = GetSensorSHT25Temperature(0, &value, &checkSum );
			
			// Convert the value to unit
			if ( retVal == ERROR_OK )
			{
				convertedValue = value + GetParameterValue(TEMP_ZERO_ADJ);
			}
			else
			{
				value = convertedValue = 0;
				ignoreCnter[TEMPERATURE_VAL_INDEX]=0;
				Kalman_Init(&Kalmanfilter[TEMPERATURE_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
				for(uint8_t i=0; i<WINDOW_SIZE; i++)
				{
					buffer[TEMPERATURE_VAL_INDEX][i]=0;
					sorted[TEMPERATURE_VAL_INDEX][i]=0;
				}
			}
			
			convertedValue = SpikeFilter(TEMPERATURE_VAL_INDEX, retVal, value, convertedValue, 1);
		}

		if (IsHumidityEnabled())
		{  
			retVal = GetSensorSHT25Humidity(0, &value, &checkSum );
			
			// Convert the value to unit
			if ( retVal == ERROR_OK )
			{
				convertedValue = value + GetParameterValue(RH_ZERO_ADJ);
			}
			else
			{
				value = convertedValue = 0;
				ignoreCnter[HUMIDITY_VAL_INDEX]=0;
				Kalman_Init(&Kalmanfilter[HUMIDITY_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
				for(uint8_t i=0; i<WINDOW_SIZE; i++)
				{
					buffer[HUMIDITY_VAL_INDEX][i]=0;
					sorted[HUMIDITY_VAL_INDEX][i]=0;
				}
			}
			
			convertedValue = SpikeFilter(HUMIDITY_VAL_INDEX, retVal, value, convertedValue, 1);
		}
	}
	else if( GetParameterValue(TEMP_RH_SENS_TYPE) == TEMP_RH_SENS_IDT_HS3100 )
	{
		if(IsTemperatureEnabled() || IsHumidityEnabled())
		{
			// Non-blocking: read the conversion started on the previous call
			// (250 ms ago, sensor needs ~35 ms), then start the next one.
			static uint8_t hs3100Started = 0;

			if(hs3100Started)
				retVal = GetSensorReadings_IDT_HS3100(0, &value, &value1 );
			else
				retVal = ERROR_TEMP_RH_SENS_I2C;

			hs3100Started = (StartSensor_IDT_HS3100(0) == ERROR_OK);
		}

		if(IsTemperatureEnabled())
		{   
			// Convert the value to unit
			if ( retVal == ERROR_OK )
			{
				convertedValue = (value * 16500l / 0x3FFF) - 4000 + GetParameterValue(TEMP_ZERO_ADJ);    /* degree C */
			}
			else
			{
				value = convertedValue = 0;
				ignoreCnter[TEMPERATURE_VAL_INDEX]=0;
				Kalman_Init(&Kalmanfilter[TEMPERATURE_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
				for(uint8_t i=0; i<WINDOW_SIZE; i++)
				{
					buffer[TEMPERATURE_VAL_INDEX][i]=0;
					sorted[TEMPERATURE_VAL_INDEX][i]=0;
				}
			}
			
			convertedValue = SpikeFilter(TEMPERATURE_VAL_INDEX, retVal, value, convertedValue, 1);
		}
		
		if (IsHumidityEnabled())
		{  
			// Convert the value to unit
			if ( retVal == ERROR_OK )
			{
				convertedValue = (value1 * 10000l / 0x3FFF) + GetParameterValue(RH_ZERO_ADJ); 
			}
			else
			{
				value1 = convertedValue = 0;
				ignoreCnter[HUMIDITY_VAL_INDEX]=0;
				Kalman_Init(&Kalmanfilter[HUMIDITY_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
				for(uint8_t i=0; i<WINDOW_SIZE; i++)
				{
					buffer[HUMIDITY_VAL_INDEX][i]=0;
					sorted[HUMIDITY_VAL_INDEX][i]=0;
				}
			}
			
			convertedValue = SpikeFilter(HUMIDITY_VAL_INDEX, retVal, value1, convertedValue, 1);
		}
	}
}

static void AveragePara( uint8_t SenNo, uint8_t error, unsigned int rawVal, int convertedVal )
{
	if( error != ERROR_OK )
	{
		// Sensor error: restart the filters so the error placeholder (0) is not
		// mixed into the first good readings (that caused false LOW alarms).
		Kalman_Init(&Kalmanfilter[SenNo], 0.01, 0.1, 0.0);
		count[SenNo] = 0;
		index[SenNo] = 0;
		spikeRejectCnt[SenNo] = 0;
	}
	else
	{
		convertedVal = Kalman_Update(&Kalmanfilter[SenNo], convertedVal);
	}
	//OSSemaTakeEver(DeviceValueMutex);
	sensorVal[SenNo].errorCode = error;
	sensorVal[SenNo].rawValue = rawVal;
	sensorVal[SenNo].convertedValue = convertedVal;
	//OSSemaGive(DeviceValueMutex);
}

static int isBuzzerSuppressionReq = FALSE;			

void GetPareValue(uint8_t ParaNumber, SENSOR_VALUE * value )
{
	//OSSemaTakeEver(DeviceValueMutex);
	value->errorCode = sensorVal[ParaNumber].errorCode;
	value->rawValue = sensorVal[ParaNumber].rawValue;
	value->convertedValue = sensorVal[ParaNumber].convertedValue;

	if(ParaNumber==HUMIDITY_VAL_INDEX)
	{
		if(value->convertedValue>10000) value->convertedValue = 10000;
	}
	//OSSemaGive(DeviceValueMutex);
}

uint8_t GetSystemError()
{
   return systemError;
}

uint16_t GetAlarms()
{
   return alarmOut.alarmByte;
}

unsigned int AlarmActive_Buzzer()
{
   return buzzerOut.alarmByte;
}


void SetBuzzerSuppressionReq()
{
   isBuzzerSuppressionReq = TRUE;
}

int GetAndResetBuzzerSuppressionReq()
{
   if( isBuzzerSuppressionReq )
   {
      isBuzzerSuppressionReq = FALSE;
	  return TRUE;
   }
   return FALSE;
}

void DeviceIOInit()
{
   uint8_t i;

   for( i = 0; i < sizeof(sensorVal)/sizeof(sensorVal[0]); i++ )
   {
       sensorVal[i].errorCode = -1;
       sensorVal[i].rawValue = -1;
       sensorVal[i].convertedValue = -1;
   }

   //OSSemaCreateMutex( DeviceValueMutex );
   //OSSemaGive(DeviceValueMutex);

   /* Spawn the producer and consumer. */
   //xTaskCreate( DEVICEIO_FUNC_NAME, "DIO", DEVICEIO_STACK_SIZE, ( void * ) 0, DEVICEIO_PRIORITY, NULL );
   //xTaskCreate( TEMPRHIO_FUNC_NAME, "TIO", TEMPRHIO_STACK_SIZE, ( void * ) 0, TEMPRHIO_PRIORITY, NULL );
   
   ledState.ledByte = 0;
   oldAlarmOut.alarmByte = alarmOut.alarmByte = 0;
   LED_ABD_PRES_OK_DIR_OUT;
   LED_ABD_PRES_ALM_DIR_OUT;
   LED_DP2_OK_DIR_OUT;
   LED_DP2_ALM_DIR_OUT;
   LED_TEMP_OK_DIR_OUT;
   LED_TEMP_ALM_DIR_OUT;
   LED_HUM_OK_DIR_OUT;
   LED_HUM_ALM_DIR_OUT;
   
   //OUTPUT_PORT_DIR = 0xFF;  // As outputs
   //
   //uint16_t Output;
   //
   //Output = OSReadEEPromWord((unsigned int *)EA_OUTPUT);
   //if(Output > 0x0F)
   //{
	   //
   //}
   //else
   //{
	   //SetOutput( (uint8_t)Output, (uint8_t*)&Output );
   //}
   
   //OUTPUT_PORT = 0xE0;      // set high

   INPUT_PORT_DIR = 0xE0;  // As inputs
   INPUT_PORT = 0xFF;      // Pullups on
   
   // Kalman filter for pressure
   Kalman_Init(&Kalmanfilter[DP1_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
   Kalman_Init(&Kalmanfilter[DP2_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
   Kalman_Init(&Kalmanfilter[DP3_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
   Kalman_Init(&Kalmanfilter[TEMPERATURE_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
   Kalman_Init(&Kalmanfilter[HUMIDITY_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
   Kalman_Init(&Kalmanfilter[TEMPERATURE2_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
   Kalman_Init(&Kalmanfilter[HUMIDITY2_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
}

uint8_t CurrentDoorStatus(void)
{
	if(DOOR_SENSE)
	{
		return OPEN;
	}
	else
	{
		return CLOSE;
	}
}



