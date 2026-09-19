#ifndef __PRESSURE_SENS__
#define __PRESSURE_SENS__

#include <avr/io.h>

#define DP1_SENSOR_RESOLUTION_OTHER			0x7FFF  // 15 bits
#define DP1_SENSOR_RESOLUTION_SM9543		0x3FFF  // 14 bits
#define DP1_SENSOR_RESOLUTION_XGZP6891D		0x7FFFFF  // 24 bits

//-------------------------------------------------------------------------------------
// Scaling
//
// The 24-bit reading is two's complement with zero at 0x800000, normalised to -1.0 .. +1.0
// by dividing by 8388608, then multiplied by the part's full-scale pressure.
//
// NOTE: the datasheet says the pressure conversion "needs to be converted according to
// the pressure range" and recommends the vendor's own C code.  The linear form below is
// the documented one; if WFH supply a compensated routine, it drops in here.
//-------------------------------------------------------------------------------------
#define WF200DP_ZERO_COUNT			8388608.0f	// 2^23
#define WF200DP_FULL_SCALE_PA		500.0f		// WF200DPZ0.005B -> 0.005 bar -> 500 Pa

uint8_t GetSensorDP1_SM9543( int * pressure );
uint8_t GetSensorDP1_XGZP6891D( int * pressure );
uint8_t GetSensorDP1_WF200DP( int * pressure );

// returns ERROR_OK or error code
uint8_t GetSensorDP1( int * pressure );


#endif

