#ifndef __PRESSURE_SENS__
#define __PRESSURE_SENS__

#include <avr/io.h>

#define DP1_SENSOR_RESOLUTION_OTHER			0x7FFF  // 15 bits
#define DP1_SENSOR_RESOLUTION_SM9543		0x3FFF  // 14 bits
#define DP1_SENSOR_RESOLUTION_XGZP6891D		0x7FFFFF  // 24 bits
#define DP2_SENSOR_RESOLUTION_OTHER			0x7FFF  // 15 bits
#define DP2_SENSOR_RESOLUTION_SM9543		0x3FFF  // 14 bits
#define DP2_SENSOR_RESOLUTION_XGZP6891D		0x7FFFFF  // 24 bits
#define DP3_SENSOR_RESOLUTION_OTHER 		0x7FFF  // 15 bits
#define DP3_SENSOR_RESOLUTION_SM9543	    0x3FFF  // 14 bits
#define DP3_SENSOR_RESOLUTION_XGZP6891D		0x7FFFFF  // 24 bits

uint8_t GetSensorDP1_SM9543( int * pressure );
uint8_t GetSensorDP1_XGZP6891D( int * pressure );

uint8_t GetSensorDP2_SM9543( int * pressure );
uint8_t GetSensorDP2_XGZP6891D( int * pressure );

uint8_t GetSensorDP3_SM9543( int * pressure);
uint8_t GetSensorDP3_XGZP6891D( int * pressure );

// returns ERROR_OK or error code
uint8_t GetSensorDP1( int * pressure );
uint8_t GetSensorDP2( int * pressure );
uint8_t GetSensorDP3( int * pressure );

#endif

