#include <avr/io.h>        // This defers to avr/io.h for GCC
#include <avr/wdt.h>
#include "os_api.h"
#include "error.h"
#include "PeriodicProcess.h"
#include "HardwareInfo.h"
#include "UserInterface.h"
#include "SerialInterface.h"
#include "DeviceIO.h"
#include "eepromAdr.h"

static int serialTime,serialPort, loggingTime;
static int buzzerSpan, buzzerOnTime, buzzerTimeCounter, buzzerDisableTime, oldBuzzVal, currBuzzVal;

void PERIODIC_FUNC_NAME( void )
{
	static uint16_t DoorOpenAlarmTimer = 0;
	
	if(CurrentDoorStatus()==OPEN)
	{
		if(GetParameterValue( DOOR_ALARM_TIME ))
		{
			if(DoorOpenAlarmTimer<60000)DoorOpenAlarmTimer++;
			if(DoorOpenAlarmTimer >= GetParameterValue( DOOR_ALARM_TIME ))
			{
				OUTPUT1_LOW;
				OUTPUT2_LOW;
				OUTPUT3_LOW;
				OUTPUT4_LOW;
				setDooralarm();
			}
		}
	}
	else
	{
		OUTPUT1_HIGH;
		OUTPUT2_HIGH;
		OUTPUT3_HIGH;
		OUTPUT4_HIGH;
		
		clearDooralarm();
		DoorOpenAlarmTimer = 0;
	}
	
	if(!gu8_IsLCDDisable)
	{
		DisplaySensorValues();
	}
	
	if(gu16_AlarmLimitCheckDisableTimer)
	{
		gu16_AlarmLimitCheckDisableTimer--;
		if(!gu16_AlarmLimitCheckDisableTimer)
		{
			if(GetParameterValue(DP1_UPPER_ALARM_OFF_LIMIT) > GetParameterValue(DP1_UPPER_ALARM_ON_LIMIT))
			{
				SetParameterValue(EA_DP1_UPPER_ALARM_OFF_LIMIT, GetParameterValue(DP1_UPPER_ALARM_ON_LIMIT)-100);
			}
			
			if(GetParameterValue(DP2_UPPER_ALARM_OFF_LIMIT) > GetParameterValue(DP2_UPPER_ALARM_ON_LIMIT))
			{
				SetParameterValue(EA_DP2_UPPER_ALARM_OFF_LIMIT, GetParameterValue(DP2_UPPER_ALARM_ON_LIMIT)-100);
			}
			
			if(GetParameterValue(DP3_UPPER_ALARM_OFF_LIMIT) > GetParameterValue(DP3_UPPER_ALARM_ON_LIMIT))
			{
				SetParameterValue(EA_DP3_UPPER_ALARM_OFF_LIMIT, GetParameterValue(DP3_UPPER_ALARM_ON_LIMIT)-100);
			}
			
			if(GetParameterValue(TEMP_UPPER_ALARM_OFF_LIMIT) > GetParameterValue(TEMP_UPPER_ALARM_ON_LIMIT))
			{
				SetParameterValue(EA_TEMP_UPPER_ALARM_OFF_LIMIT, GetParameterValue(TEMP_UPPER_ALARM_ON_LIMIT)-100);
			}
			
			if(GetParameterValue(RH_UPPER_ALARM_OFF_LIMIT) > GetParameterValue(RH_UPPER_ALARM_ON_LIMIT))
			{
				SetParameterValue(EA_RH_UPPER_ALARM_OFF_LIMIT, GetParameterValue(RH_UPPER_ALARM_ON_LIMIT)-100);
			}
			
			if(GetParameterValue(DP1_LOWER_ALARM_OFF_LIMIT) < GetParameterValue(DP1_LOWER_ALARM_ON_LIMIT))
			{
				SetParameterValue(EA_DP1_LOWER_ALARM_OFF_LIMIT, GetParameterValue(DP1_LOWER_ALARM_ON_LIMIT)+100);
			}
			
			if(GetParameterValue(DP2_LOWER_ALARM_OFF_LIMIT) < GetParameterValue(DP2_LOWER_ALARM_ON_LIMIT))
			{
				SetParameterValue(EA_DP2_LOWER_ALARM_OFF_LIMIT, GetParameterValue(DP2_LOWER_ALARM_ON_LIMIT)+100);
			}
			
			if(GetParameterValue(DP3_LOWER_ALARM_OFF_LIMIT) < GetParameterValue(DP3_LOWER_ALARM_ON_LIMIT))
			{
				SetParameterValue(EA_DP3_LOWER_ALARM_OFF_LIMIT, GetParameterValue(DP3_LOWER_ALARM_ON_LIMIT)+100);
			}
			
			if(GetParameterValue(TEMP_LOWER_ALARM_OFF_LIMIT) < GetParameterValue(TEMP_LOWER_ALARM_ON_LIMIT))
			{
				SetParameterValue(EA_TEMP_LOWER_ALARM_OFF_LIMIT, GetParameterValue(TEMP_LOWER_ALARM_ON_LIMIT)+100);
			}
			
			if(GetParameterValue(RH_LOWER_ALARM_OFF_LIMIT) < GetParameterValue(RH_LOWER_ALARM_ON_LIMIT))
			{
				SetParameterValue(EA_RH_LOWER_ALARM_OFF_LIMIT, GetParameterValue(RH_LOWER_ALARM_ON_LIMIT)+100);
			}
		}
	}
	
	if(u8_resetTimer)
	{
		u8_resetTimer--;
		if(!u8_resetTimer)
		{
			//SoftReset ------------------
			CPU_CCP  = CCP_IOREG_gc;
			RST.CTRL = RST_SWRST_bm ;
		}
	}
	
	if(u8_DFUresetTimer)		// set by the serial DFU command; display shows "< DFU MODE >"
	{
		u8_DFUresetTimer--;
		if(!u8_DFUresetTimer)
		{
			//SoftReset ------------------
			CPU_CCP  = CCP_IOREG_gc;
			RST.CTRL = RST_SWRST_bm ;
		}
	}

	if( IsLoggingEnabled() && --loggingTime == 0 )
	{
		loggingTime = GetParameterValue( LOGGING_INTERVAL );
	}

	if( IsSerialEnabled() && IsBroadcastEnabled() && --serialTime == 0 )
	{
		serialTime = GetParameterValue( BROADCAST_INTERVAL );
		serialPort = GetParameterValue( BROADCAST_PORT );
		
		SerialPrinter();
		SerialBroadcast(serialPort);
	}

	currBuzzVal = AlarmActive_Buzzer();
	oldBuzzVal &= currBuzzVal;

	if( currBuzzVal ^ oldBuzzVal )
	buzzerDisableTime = 0;
	if( GetAndResetBuzzerSuppressionReq() )
	buzzerDisableTime = GetParameterValue( BUZZER_DISABLE_TIME );
	if( buzzerDisableTime > 0)
	buzzerDisableTime--;
	
	oldBuzzVal = currBuzzVal;
	buzzerOnTime = GetParameterValue( BUZZER_ON_TIME );
	buzzerSpan = buzzerOnTime + GetParameterValue( BUZZER_OFF_TIME );
	
	if(( buzzerTimeCounter < buzzerOnTime ) && buzzerOnTime && buzzerDisableTime <= 0)
	{
		if( currBuzzVal )
		BuzzerOn();
		else
		BuzzerOff();
	}
	else
	{
		BuzzerOff();
	}
	
	if( buzzerSpan > 0 )		// ON+OFF time can both be 0: avoid modulo by zero
		buzzerTimeCounter = (buzzerTimeCounter + 1) % buzzerSpan;
	else
		buzzerTimeCounter = 0;
}

void SuppressBuzzer()
{
   buzzerDisableTime = GetParameterValue( BUZZER_DISABLE_TIME );
}


void PeriodicProcessInit()
{
	loggingTime = GetParameterValue( LOGGING_INTERVAL );
	serialTime = GetParameterValue( BROADCAST_INTERVAL );
	serialPort = GetParameterValue( BROADCAST_PORT );
	buzzerTimeCounter = 0;
	buzzerDisableTime = GetParameterValue( BUZZER_DISABLE_TIME );
}


