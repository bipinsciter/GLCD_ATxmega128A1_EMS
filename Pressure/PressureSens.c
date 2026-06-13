#include "SW_I2C1Interface.h"
#include "SW_I2C2Interface.h"
#include "SW_I2C3Interface.h"
#include "SW_I2C4Interface.h"
#include "PressureSens.h"
#include "error.h"
#include "UserInterface.h"
#include <util/delay.h>

#define SENS_ADD_SM9543			(0x28*2)
#define SENS_ADD_XGZP6891D	    (0x7F*2)

uint8_t GetSensorDP1( int * pressure)
{
	switch( GetParameterValue(DP1_SENS_TYPE))
	{
		case PRES_SENS_SM9543:    return GetSensorDP1_SM9543( pressure );
		
		case PRES_SENS_XGZP6891D:   return GetSensorDP1_XGZP6891D( pressure );
		
		case PRES_SENS_NONE:
		default:                  
			*pressure = 0;
			return ERROR_DP1_SENS_I2C;
	}
}

uint8_t GetSensorDP2( int * pressure)
{
	switch( GetParameterValue(DP2_SENS_TYPE))
	{ 
		case PRES_SENS_SM9543:    return GetSensorDP2_SM9543( pressure );

		case PRES_SENS_XGZP6891D:   return GetSensorDP2_XGZP6891D( pressure );

		case PRES_SENS_NONE:
		default:                 
			*pressure = 0;
			return ERROR_DP2_SENS_I2C;
	}
}

uint8_t GetSensorDP3( int * pressure )
{
	switch( GetParameterValue(DP3_SENS_TYPE))
	{
		case PRES_SENS_SM9543:    return GetSensorDP3_SM9543( pressure );

		case PRES_SENS_XGZP6891D:   return GetSensorDP3_XGZP6891D( pressure );

		case PRES_SENS_NONE:
		default:
		*pressure = 0;
		return ERROR_DP3_SENS_I2C;
	}
}

uint8_t GetSensorDP1_SM9543( int * pressure )
{
	uint8_t data[2];
	uint16_t differanceDP=0;
	float dpVal = 0;
	
	if( SW_I2C3ReadBytes( SENS_ADD_SM9543, 2, (uint8_t *)&data[0]))
	return ERROR_DP1_SENS_I2C;
	
	differanceDP = data[0];
	differanceDP <<= 8;
	differanceDP |= data[1];
	
	if((differanceDP & 0xC000) == 0xC000) 
	{
		return ERROR_DP1_SENS_I2C;
	}
	else
	{
		differanceDP &= 0x3FFF;
		
		dpVal = (float)differanceDP-1638;
		dpVal *= 0.0762951094834821;;
		dpVal -= 500;;
		
		*pressure = (uint16_t)(dpVal*100);
		
		return ERROR_OK;
	}
}

uint8_t GetSensorDP2_SM9543( int * pressure )
{
	uint8_t data[2];
	uint16_t differanceDP=0;
	float dpVal = 0;
	
	if( SW_I2C4ReadBytes( SENS_ADD_SM9543, 2, (uint8_t *)&data[0]))
	return ERROR_DP2_SENS_I2C;
	
	differanceDP = data[0];
	differanceDP <<= 8;
	differanceDP |= data[1];
	
	if((differanceDP & 0xC000) == 0xC000)
	{
		return ERROR_DP2_SENS_I2C;
	}
	else
	{
		differanceDP &= 0x3FFF;
		
		dpVal = (float)differanceDP-1638;
		dpVal *= 0.0762951094834821;;
		dpVal -= 500;;
		
		*pressure = (uint16_t)(dpVal*100);
		
		return ERROR_OK;
	}
}

uint8_t GetSensorDP3_SM9543( int *pressure)
{
	uint8_t data[2];
	uint16_t differanceDP=0;
	float dpVal = 0;
	
	if( SW_I2C2ReadBytes( SENS_ADD_SM9543, 2, (uint8_t *)&data[0]))
	return ERROR_DP3_SENS_I2C;
	
	differanceDP = data[0];
	differanceDP <<= 8;
	differanceDP |= data[1];
	
	if((differanceDP & 0xC000) == 0xC000)
	{
		return ERROR_DP3_SENS_I2C;
	}
	else
	{
		differanceDP &= 0x3FFF;
		
		dpVal = (float)differanceDP-1638;
		dpVal *= 0.0762951094834821;;
		dpVal -= 500;;
		
		*pressure = (uint16_t)(dpVal*100);
		
		return ERROR_OK;
	}
}

uint8_t GetSensorDP1_XGZP6891D( int *pressure)
{
	unsigned long differanceDP=0;
	float dpVal = 0;
	uint8_t temp=0, data[2]={0x30,0x0A};
	uint8_t data1[3]={0x00,0x00,0x00};
	
	if( SW_I2C3WriteBytes( SENS_ADD_XGZP6891D, 2, &data[0]))
	return ERROR_DP1_SENS_I2C;

	//_delay_ms(20);
	
	do
	{
		data[0]=0x30;
		SW_I2C3WriteReadBytes( SENS_ADD_XGZP6891D, 1, &data[0], 1, &data[1]);
		_delay_ms(5);
		if(++temp>4) break;
		
	}while(data[1] & 0x08);
	
	data[0]=0x06;
	if( SW_I2C3WriteReadBytes( SENS_ADD_XGZP6891D, 1, &data[0], 3, &data1[0]))
	return ERROR_DP1_SENS_I2C;
	
	differanceDP = data1[0];
	differanceDP<<=8;
	differanceDP |= data1[1];
	differanceDP<<=8;
	differanceDP |= data1[2];
	
	differanceDP &= 0xFFFFFF;
	
	if(differanceDP == 0xFFFFFF) 
	{
		ERROR_DP1_SENS_I2C;
	}
	else
	{
		if (differanceDP & 0x800000) 
		{
			differanceDP -= 16777216; // Shift 2's complement value into standard signed space
		}
		dpVal = (float)differanceDP;
		dpVal = (dpVal * 0.000149012) - 625.0000559;
		*pressure = (uint16_t)(dpVal*100);
	}
	return ERROR_OK;
}

uint8_t GetSensorDP2_XGZP6891D( int *pressure)
{
	unsigned long differanceDP=0;
	float dpVal = 0;
	uint8_t temp=0, data[2]={0x30,0x0A};
	uint8_t data1[3]={0x00,0x00,0x00};
	
	if( SW_I2C4WriteBytes( SENS_ADD_XGZP6891D, 2, &data[0]))
	return ERROR_DP2_SENS_I2C;

	//_delay_ms(20);
	
	do
	{
		data[0]=0x30;
		SW_I2C4WriteReadBytes( SENS_ADD_XGZP6891D, 1, &data[0], 1, &data[1]);
		_delay_ms(5);
		if(++temp>4) break;
		
	}while(data[1] & 0x08);
	
	data[0]=0x06;
	if( SW_I2C4WriteReadBytes( SENS_ADD_XGZP6891D, 1, &data[0], 3, &data1[0]))
	return ERROR_DP2_SENS_I2C;
	
	differanceDP = data1[0];
	differanceDP<<=8;
	differanceDP |= data1[1];
	differanceDP<<=8;
	differanceDP |= data1[2];
	
	differanceDP &= 0xFFFFFF;
	
	if(differanceDP == 0xFFFFFF)
	{
		ERROR_DP2_SENS_I2C;
	}
	else
	{
		if (differanceDP & 0x800000)
		{
			differanceDP -= 16777216; // Shift 2's complement value into standard signed space
		}
		dpVal = (float)differanceDP;
		dpVal = (dpVal * 0.000149012) - 625.0000559;
		*pressure = (uint16_t)(dpVal*100);
	}
	return ERROR_OK;
}

uint8_t GetSensorDP3_XGZP6891D( int *pressure)
{
	unsigned long differanceDP=0;
	float dpVal = 0;
	uint8_t temp=0, data[2]={0x30,0x0A};
	uint8_t data1[3]={0x00,0x00,0x00};
	
	if( SW_I2C2WriteBytes( SENS_ADD_XGZP6891D, 2, &data[0]))
	return ERROR_DP3_SENS_I2C;

	//_delay_ms(20);
	
	do
	{
		data[0]=0x30;
		SW_I2C2WriteReadBytes( SENS_ADD_XGZP6891D, 1, &data[0], 1, &data[1]);
		_delay_ms(5);
		if(++temp>4) break;
		
	}while(data[1] & 0x08);
	
	data[0]=0x06;
	if( SW_I2C2WriteReadBytes( SENS_ADD_XGZP6891D, 1, &data[0], 3, &data1[0]))
	return ERROR_DP3_SENS_I2C;
	
	differanceDP = data1[0];
	differanceDP<<=8;
	differanceDP |= data1[1];
	differanceDP<<=8;
	differanceDP |= data1[2];
	
	differanceDP &= 0xFFFFFF;
	
	if(differanceDP == 0xFFFFFF)
	{
		ERROR_DP3_SENS_I2C;
	}
	else
	{
		if (differanceDP & 0x800000)
		{
			differanceDP -= 16777216; // Shift 2's complement value into standard signed space
		}
		dpVal = (float)differanceDP;
		dpVal = (dpVal * 0.000149012) - 625.0000559;
		*pressure = (uint16_t)(dpVal*100);
	}
	return ERROR_OK;
}


