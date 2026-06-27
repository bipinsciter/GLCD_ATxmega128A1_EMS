#include <avr/io.h>        // This defers to avr/io.h for GCC
#include <avr/wdt.h>
#include "os_api.h"
#include "error.h"
#include "PeriodicProcess.h"
#include "HardwareInfo.h"
#include "UserInterface.h"
#include "SerialInterface.h"
#include "DeviceIO.h"
#include "pid.h"

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
			gu8_AlarmLimitCheckDisable=0;
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
	
	buzzerTimeCounter = (buzzerTimeCounter + 1) % buzzerSpan;

	if( IsAlalogOutEnabled() && IsPIDEnabled())
	{
		PIDLoopControl();
	}
	  
	//OSSleep( 1000 );
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


