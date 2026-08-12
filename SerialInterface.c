#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>        
#include <string.h>        
#include "os_api.h"
#include "locked_printf.h"
#include "SerialInterface.h"
#include "UserInterface.h"
#include "HardwareInfo.h"
#include "deviceio.h"
#include "rtc.h"
#include "error.h"
#include "eepromAdr.h"
#include "pid.h"

#ifdef OS_AVRX
#include "AvrXserialIO.h"
#else
#include "FreeRTOSBufferedSerial.h"
#include "timers.h"
#endif /* OS_AVRX */

#define SERIAL_CMD_SOT	         0xFF
#define SERIAL_CMD_EOT	         0xFE

#define SERIAL_RESP_SOT	         0xFD
#define SERIAL_RESP_EOT	         0xFC

#define SERIAL_CMD_SEP			0xEF
#define SERIAL_RESP_SEP         0xEE

/* Message IDs */
#define READ_ALL_RUNTIME_VALUES 0x0
#define READ_DP1_VALUE			0x1
#define READ_DP2_VALUE			0x2
#define READ_TEMP_VALUE         0x3
#define READ_HUMIDITY_VALUE     0x4
#define READ_SYSTEM_ERR         0x6
#define READ_ALARMS             0x7
#define READ_PID_PER_VALUES     0x8
#define READ_DP3_VALUE			0x9


#define START_LOG_READING       0xA    // returns error code and resets the log read pointer
#define ERASE_LOG_READING       0xB    // returns error code, erases the log info.
#define READ_LOG_CURRENT        0xC    // returns error code and actual log data at the pointer
#define READ_LOG_NEXT           0xD    // returns error code and actual log data at the next pointer

#define READ_PARAMETER     		0x10      
#define WRITE_PARAMETER    		0x11    
#define READ_OUTPUT   	   		0x12        // Read Output
#define WRITE_OUTPUT   			0x13        // Write Output
#define READ_INPUT   	   		0x14        // Read Input
#define PUT_DFU_ID				0xAA
#define LCD_CONTROL_ID			0x6F
#define COM_CONTROL_ID			0x70

// Broadcast data IDs
#define ALL_RUNTIME_VALUES_BRDCAST  0x80
#define DP1_BRDCAST					0x81
#define DP2_BRDCAST					0x82
#define TEMPERATURE_BRDCAST         0x83
#define HUMIDITY_BRDCAST            0x84
#define SYSTEMERR_BRDCAST           0x85
#define ALARM_BRDCAST               0x86
#define DP3_BRDCAST					0x87

#define DFU_PASSWORD				3123

/*
Protocol:
All bytes in the packet will have 7 bits value except for Start, stop, deviceID and broadcast ID byte
CheckSum calculation: 
   cksum = 0x55; cksum = (cksum + byte) & 0x7f for all bytes starting from deviceID to end of data.
Receive: 
   0xFF<deviceID><cmdID><data 0.. n><7 bit module 0x55 checksum>0xFE
     Data Formates:
       For parameter read commands data0 is always parameter ID in binary.
       For parameter write commands data0 is always parameter ID in binary followed by ASCII value.
	   For Sensor values no data.
	   For Alarms no data
Send (data response): 
   0xFD<deviceID><cmdID><data0.. n><7 bit module 0x55 checksum>0xFC
       For parameter commands data0 error code followed by binary parameter ID followed by ASCII value.
	   For Sensor values data0 is binary error code followed by ASCII value.
Send (broadcast): 
   0xFD<deviceID><broadcastDataID><data0.. n><7 bit module 0x55 checksum>0xFC
	   For Sensor values data0 is binary error code followed by ASCII value.
	   For Alarms data0 Bit 0 Pressure Abs High Alarm
						Bit 1 Pressure Abs Low Alarm
				  data1 Bit 0 Pressure Diff High Alarm
						Bit 1 Pressure Diff Low Alarm
				  data2 Bit 0 Temp High Alarm
						Bit 1 Temp Low Alarm
				  data3 Bit 0 Humidity High Alarm
						Bit 1 Humidity Low Alarm
*/

static int uart_putchar(char c, FILE *stream);
static void SerialPrint( char * str );
static void SendPacket( uint8_t comId, uint8_t dataId, char * dataVal, uint8_t dataLen );
static int MakeAllRuntimeParaXmitSrting();
 void vBaseTimerCallback( TimerHandle_t xBaseTimer);
 
static uint8_t cmdId[2];
static char rxDataBuffer[2][RX0_BUFSZ];
static char txDataBuffer[TX0_BUFSZ];
static int paraValue;
static uint8_t rxDataLen[2]={0}, rxMode=0, msgRcvd=0, RxTimeout=0;
static uint8_t rxMode1=0, msgRcvd1=0, RxTimeout1=0;

uint8_t u8_resetTimer=0,u8_DFUresetTimer=0,gu8_SetBySerial=0;
	
TimerHandle_t xBaseTimer;

static SENSOR_VALUE senVal;

//static OSSemaMutex SerialDeviceMutex;

static FILE serialStdIo;

const char monthTbl [][4] PROGMEM =
{
   "ERR",
   "JAN",
   "FEB",
   "MAR",
   "APR",
   "MAY",
   "JUN",
   "JUL",
   "AUG",
   "SEP",
   "OCT",
   "NOV",
   "DEC"
};

//=============================================================================================================
ISR(USARTC0_RXC_vect)
{	
	uint8_t ch;

    /* Get the character and post it on the queue of Rxed characters.
    If the post causes a task to wake force a context switch as the woken task
    may have a higher priority than the task we have interrupted. */
    ch = USARTC0_DATA;
	
	if(msgRcvd==1) return;

	if((ch==SERIAL_CMD_SOT) && (!rxMode))
	{
		rxMode=1;
		RxTimeout = 4;
	}
	else if(rxMode==1)
	{
		if((ch==GetParameterValue( DEVICE_ID )) || (ch==0x00))
		{
			rxDataBuffer[0][rxDataLen[0]++]=ch;
			rxMode=2;
			RxTimeout = 4;
		}
		else
		{
			rxMode=0;
			rxDataLen[0]=0;
			RxTimeout = 0;
		}
	}
	else if(rxMode==2)
	{
		cmdId[0] = ch;
		rxDataBuffer[0][rxDataLen[0]++]=ch;
		rxMode=4;
	}
	else if(rxMode==3)
	{
		cmdId[0] = ch;
		rxDataBuffer[0][rxDataLen[0]++]=ch;
		rxMode=4;
	}
	else if(rxMode==4)
	{
		if(ch==SERIAL_CMD_EOT)
		{
			rxMode=0;
			RxTimeout = 0;
			
			uint8_t crcVal;
			crcVal=CalCRC((uint8_t*)&rxDataBuffer[0][0],rxDataLen[0]-1);
			if(rxDataBuffer[0][rxDataLen[0]-1]==crcVal)
			{
				msgRcvd = 1;
			}
			else
			{
				rxDataLen[0]=0;
			}
		}
		else
		{
			rxDataBuffer[0][rxDataLen[0]++]=ch;
			RxTimeout = 4;
		}
	}
	if(rxDataLen[0]>=RX0_BUFSZ) rxDataLen[0]=0;
}

ISR(USARTC0_DRE_vect)
{
	signed char cChar, cTaskWoken = pdFALSE;

	if( xQueueReceiveFromISR( vUSARTC0_TxQueue, &cChar, &cTaskWoken ) == pdTRUE )
	{
		/* Send the next character queued for Tx. */
		USARTC0_DATA = cChar;
	}
	else
	{
		/* Queue empty, nothing to send. */
		USARTC0_CTRLA &= ~(USART_DREINTLVL_gm);			// Disable UDRE interrupt
	}
}

ISR(USARTC0_TXC_vect)
{
	TX0_DIS_RS485;
	USARTC0_CTRLA &= ~(USART_TXCINTLVL_gm);			// Disable UTXC interrupt
}

void UARTC0RxTimeout(void)
{
	if(RxTimeout)
	{
		RxTimeout--;
		if(!RxTimeout)
		{
			msgRcvd=0;
			rxMode=0;
			rxDataLen[0]=0;
		}
	}
}

//=============================================================================================================
ISR(USARTC1_RXC_vect)
{	
	uint8_t ch;

    /* Get the character and post it on the queue of Rxed characters.
    If the post causes a task to wake force a context switch as the woken task
    may have a higher priority than the task we have interrupted. */
    ch = USARTC1_DATA;
	
	if(msgRcvd1==1) return;

	if((ch==SERIAL_CMD_SOT) && (!rxMode1))
	{
		rxMode1=1;
		RxTimeout1 = 4;
	}
	else if(rxMode1==1)
	{
		if((ch==GetParameterValue( DEVICE_ID )) || (ch==0x00))
		{
			rxDataBuffer[1][rxDataLen[1]++]=ch;
			rxMode1=2;
			RxTimeout1 = 4;
		}
		else
		{
			rxMode1=0;
			rxDataLen[1]=0;
			RxTimeout1 = 0;
		}
	}
	else if(rxMode1==2)
	{
		cmdId[1] = ch;
		rxDataBuffer[1][rxDataLen[1]++]=ch;
		rxMode1=4;
	}
	else if(rxMode1==3)
	{
		cmdId[1] = ch;
		rxDataBuffer[1][rxDataLen[1]++]=ch;
		rxMode1=4;
	}
	else if(rxMode1==4)
	{
		if(ch==SERIAL_CMD_EOT)
		{
			rxMode1=0;
			RxTimeout1 = 0;
			
			uint8_t crcVal;
			crcVal=CalCRC((uint8_t*)&rxDataBuffer[1][0],rxDataLen[1]-1);
			if(rxDataBuffer[1][rxDataLen[1]-1]==crcVal)
			{
				msgRcvd1 = 1;
			}
			else
			{
				rxDataLen[1]=0;
			}
		}
		else
		{
			rxDataBuffer[1][rxDataLen[1]++]=ch;
			RxTimeout1 = 4;
		}
	}
	if(rxDataLen[1]>=RX1_BUFSZ) rxDataLen[1]=0;
}

ISR(USARTC1_DRE_vect)
{
	signed char cChar, cTaskWoken = pdFALSE;

	if( xQueueReceiveFromISR( vUSARTC1_TxQueue, &cChar, &cTaskWoken ) == pdTRUE )
	{
		/* Send the next character queued for Tx. */
		USARTC1_DATA = cChar;
	}
	else
	{
		/* Queue empty, nothing to send. */
		USARTC1_CTRLA &= ~(USART_DREINTLVL_gm);			// Disable UDRE interrupt
	}
}

ISR(USARTC1_TXC_vect)
{
	TX1_DIS_RS485;
	USARTC1_CTRLA &= ~(USART_TXCINTLVL_gm);			// Disable UTXC interrupt
}

void UARTC1RxTimeout(void)
{
	if(RxTimeout1)
	{
		RxTimeout1--;
		if(!RxTimeout1)
		{
			msgRcvd1=0;
			rxMode1=0;
			rxDataLen[1]=0;
		}
	}
}

uint8_t CheckSerialSet(void)
{
	return gu8_SetBySerial;
}

void SerialInterfaceTask( void )
{
	uint8_t ch, i=0, Ind = 0;
	int16_t tempshort = 0;
				 
	if((msgRcvd==1) || (msgRcvd1==1))
	{
		if (msgRcvd==1) Ind=0;
		else Ind=1;
		
		if(gu8_IsCOMDisable)
		{
			if(cmdId[Ind]==COM_CONTROL_ID)
			{
				
			}
			else
			{
				msgRcvd=0;
				rxMode=0;
				rxDataLen[0]=0;
				
				msgRcvd1=0;
				rxMode1=0;
				rxDataLen[1]=0;
				
				return;
			}
		}
		
        // Valid packet is detected. Process it.
        //OSSemaTakeEver(SerialDeviceMutex);
        switch( cmdId[Ind] )
        {
			case PUT_DFU_ID:
			 
				tempshort = findValue(&rxDataBuffer[Ind][2],rxDataLen[Ind]-3);
				if(tempshort==DFU_PASSWORD)
				{
					OSWriteEEPromWord((unsigned int *)DFU_NUMBER_LOGIC, 0xABCD);
					SendPacket(Ind, PUT_DFU_ID, txDataBuffer, i);
					u8_DFUresetTimer = 3;
					 
					//software_reset();
				}
			 
			break;
			
			case LCD_CONTROL_ID:
			
				if((rxDataBuffer[Ind][2]=='0') || (rxDataBuffer[Ind][2]=='1'))
				{
					gu8_IsLCDDisable=rxDataBuffer[Ind][2]-'0';
					OSWriteEEPromWord((unsigned int *)LCD_CONTROL_ADDR, (unsigned int)gu8_IsLCDDisable);
					SendPacket(Ind, LCD_CONTROL_ID, txDataBuffer, i);
					u8_resetTimer = 3;
				}
			
			break;
			
			case COM_CONTROL_ID:
			
				if((rxDataBuffer[Ind][2]=='0') || (rxDataBuffer[Ind][2]=='1'))
				{
					gu8_IsCOMDisable=rxDataBuffer[Ind][2]-'0';
					OSWriteEEPromWord((unsigned int *)COM_CONTROL_ADDR, (unsigned int)gu8_IsCOMDisable);
					SendPacket(Ind, COM_CONTROL_ID, txDataBuffer, i);
				}
			
			break;
			
		    case READ_ALL_RUNTIME_VALUES:
            i = MakeAllRuntimeParaXmitSrting();
 		        SendPacket(Ind, READ_ALL_RUNTIME_VALUES, txDataBuffer, i);
			 	break;
		    case READ_DP1_VALUE:
				if (IsDP1Enabled())
				{
				    GetPareValue(DP1_VAL_INDEX, &senVal);
				    txDataBuffer[0] = senVal.errorCode;
					locked_sprintf_P( txDataBuffer+1, PSTR("%d"), senVal.convertedValue );
				    SendPacket(Ind, READ_DP1_VALUE, txDataBuffer, strlen(txDataBuffer+1)+1);
				}
				else
				{
				    txDataBuffer[0] = ERROR_CMD_INVALID;
				    SendPacket(Ind, READ_DP1_VALUE, txDataBuffer, 1);
				}
			 	break;
		    case READ_DP2_VALUE:
				if (IsDP2Enabled())
				{
					GetPareValue(DP2_VAL_INDEX, &senVal);
				    txDataBuffer[0] = senVal.errorCode;
				    locked_sprintf_P( txDataBuffer+1, PSTR("%d"), senVal.convertedValue );
					SendPacket(Ind, READ_DP2_VALUE, txDataBuffer, strlen(txDataBuffer+1)+1);
				}
				else
				{
				    txDataBuffer[0] = ERROR_CMD_INVALID;
				    SendPacket(Ind, READ_DP2_VALUE, txDataBuffer, 1);
				}
			 	break;
			case READ_DP3_VALUE:
				if (IsDP3Enabled())
				{
					GetPareValue(DP3_VAL_INDEX, &senVal);
					txDataBuffer[0] = senVal.errorCode;
					locked_sprintf_P( txDataBuffer+1, PSTR("%d"), senVal.convertedValue );
					SendPacket(Ind, READ_DP3_VALUE, txDataBuffer, strlen(txDataBuffer+1)+1);
				}
				else
				{
					txDataBuffer[0] = ERROR_CMD_INVALID;
					SendPacket(Ind, READ_DP3_VALUE, txDataBuffer, 1);
				}
				break;
		    case READ_TEMP_VALUE:
				if (IsTemperatureEnabled())
				{
                GetPareValue(TEMPERATURE_VAL_INDEX, &senVal);//GetTemperature( &senVal );
				    txDataBuffer[0] = senVal.errorCode;
				    locked_sprintf_P( txDataBuffer+1, PSTR("%d"), senVal.convertedValue );
				    SendPacket(Ind, READ_TEMP_VALUE, txDataBuffer, strlen(txDataBuffer+1)+1);
				}
				else
				{
				    txDataBuffer[0] = ERROR_CMD_INVALID;
				    SendPacket(Ind, READ_TEMP_VALUE, txDataBuffer, 1);
				}
			 	break;
		    case READ_HUMIDITY_VALUE:
				if (IsHumidityEnabled())
				{
                GetPareValue(HUMIDITY_VAL_INDEX, &senVal);//GetHumidity( &senVal );
				    txDataBuffer[0] = senVal.errorCode;
				    locked_sprintf_P( txDataBuffer+1, PSTR("%d"), senVal.convertedValue );
				    SendPacket(Ind, READ_HUMIDITY_VALUE, txDataBuffer, strlen(txDataBuffer+1)+1);
				}
				else
				{
				    txDataBuffer[0] = ERROR_CMD_INVALID;
				    SendPacket(Ind, READ_HUMIDITY_VALUE, txDataBuffer, 1);
				}
			 	break;
		    case READ_SYSTEM_ERR:
                txDataBuffer[0] = GetSystemError();
 			    SendPacket(Ind, READ_SYSTEM_ERR, txDataBuffer, 1);
				break;
		    case READ_ALARMS:
			    tempshort = GetAlarms();
                txDataBuffer[0] = tempshort & 0x3;
	            tempshort >>= 2;
                txDataBuffer[1] = tempshort & 0x3;
	            tempshort >>= 2;
                txDataBuffer[2] = tempshort & 0x3;
	            tempshort >>= 2;
                txDataBuffer[3] = tempshort & 0x3;
				tempshort >>= 2;
				txDataBuffer[4] = tempshort & 0x3;
                SendPacket(Ind, READ_ALARMS, txDataBuffer, 5 );
				break;

            case READ_PID_PER_VALUES:
			    paraValue = GetAbsPresPIDPercent();
			    txDataBuffer[0] = ((char *)&paraValue)[0];
			    txDataBuffer[1] = ((char *)&paraValue)[1];					
				paraValue = GetDiffPresPIDPercent();
			    txDataBuffer[2] = ((char *)&paraValue)[0];
			    txDataBuffer[3] = ((char *)&paraValue)[1];					
                paraValue = GetTempPIDPercent();
			    txDataBuffer[4] = ((char *)&paraValue)[0];
			    txDataBuffer[5] = ((char *)&paraValue)[1];					
                paraValue = GetRHPIDPercent();
			    txDataBuffer[6] = ((char *)&paraValue)[0];
			    txDataBuffer[7] = ((char *)&paraValue)[1];					
                SendPacket(Ind, READ_PID_PER_VALUES, txDataBuffer, 8 );
				break;

		    case READ_INPUT:

                GetInput( (uint8_t*)&txDataBuffer[1] );  // return value
  		   		txDataBuffer[0] = ERROR_OK;
                SendPacket(Ind, READ_INPUT, txDataBuffer, 2 );
				break;

		    case READ_OUTPUT:

                GetOutput( (uint8_t*)&txDataBuffer[1] );  // return value
  		   		txDataBuffer[0] = ERROR_OK;
                SendPacket(Ind, READ_OUTPUT, txDataBuffer, 2 );
				break;

		    case WRITE_OUTPUT:

 			    SetOutput( (uint8_t)rxDataBuffer[Ind][3], (uint8_t*)&txDataBuffer[1] );  // set value, return value
 				txDataBuffer[0] = ERROR_OK;
               	SendPacket(Ind, WRITE_OUTPUT, txDataBuffer, 2 );
				OSWriteEEPromWord((unsigned int *)EA_OUTPUT, rxDataBuffer[Ind][3]);
				break;


		    case READ_PARAMETER:
					
				if( rxDataLen[Ind] < (uint8_t)1 )
				{
    		        txDataBuffer[0] = ERROR_PARA_INVALID;
	    			SendPacket(Ind, READ_PARAMETER, txDataBuffer, 1);
		    		break;
			    }
					
				switch((uint8_t)rxDataBuffer[Ind][2])
				{
					case IP1_HIGH_NAME:
					case IP1_LOW_NAME:
					case IP2_HIGH_NAME:
					case IP2_LOW_NAME:
					case IP3_HIGH_NAME:
					case IP3_LOW_NAME:
					case IP4_HIGH_NAME:
					case IP4_LOW_NAME:
					case OP1_HIGH_NAME:
					case OP1_LOW_NAME:
					case OP2_HIGH_NAME:
					case OP2_LOW_NAME:
					case OP3_HIGH_NAME:
					case OP3_LOW_NAME:
					case OP4_HIGH_NAME:
					case OP4_LOW_NAME:
						
						//ch = GetParameterStr( (uint8_t)rxDataBuffer[Ind][2], (uint8_t*)&rxDataBuffer[Ind][3] );
						memcpy(&rxDataBuffer[Ind][3],&gu8_inOutput[(uint8_t)rxDataBuffer[Ind][2]-IP1_HIGH_NAME][0],8);
						ch = ERROR_OK;
							
						txDataBuffer[0] = ch;
						txDataBuffer[1] = rxDataBuffer[Ind][2];
						if( ch != ERROR_OK )
						{
							SendPacket(Ind, READ_PARAMETER, txDataBuffer, 2);
						}
						else
						{
							memcpy(&txDataBuffer[2],&gu8_inOutput[(uint8_t)rxDataBuffer[Ind][2]-IP1_HIGH_NAME][0],8);
							SendPacket(Ind, READ_PARAMETER, txDataBuffer, 10);
						}
						
					break;
					default:
						
						gu8_SetBySerial=1;
						ch = GetParameter( rxDataBuffer[Ind][2], &paraValue );
						gu8_SetBySerial=0;
							
						txDataBuffer[0] = ch;
						txDataBuffer[1] = rxDataBuffer[Ind][2];
						if( ch != ERROR_OK )
						{
							SendPacket(Ind, READ_PARAMETER, txDataBuffer, 2);
						}
						else
						{
							locked_sprintf_P( txDataBuffer+2, PSTR("%d"), paraValue );
							SendPacket(Ind, READ_PARAMETER, txDataBuffer, strlen(txDataBuffer+2)+2);
						}
						
					break;
				}
				break;
		    case WRITE_PARAMETER:
                if( rxDataLen[Ind] < (uint8_t)2 )
				{
  					txDataBuffer[0] = ERROR_PARA_INVALID;
	    			SendPacket(Ind, WRITE_PARAMETER, txDataBuffer, 1);
		    		break;
			    }
				  
				switch((uint8_t)rxDataBuffer[Ind][2])
				{
					case IP1_HIGH_NAME:
					case IP1_LOW_NAME:
					case IP2_HIGH_NAME:
					case IP2_LOW_NAME:
					case IP3_HIGH_NAME:
					case IP3_LOW_NAME:
					case IP4_HIGH_NAME:
					case IP4_LOW_NAME:
					case OP1_HIGH_NAME:
					case OP1_LOW_NAME:
					case OP2_HIGH_NAME:
					case OP2_LOW_NAME:
					case OP3_HIGH_NAME:
					case OP3_LOW_NAME:
					case OP4_HIGH_NAME:
					case OP4_LOW_NAME:
					  
						memset(&gu8_inOutput[(uint8_t)rxDataBuffer[Ind][2]-IP1_HIGH_NAME][0],0,10);
						memcpy(&gu8_inOutput[(uint8_t)rxDataBuffer[Ind][2]-IP1_HIGH_NAME][0],&rxDataBuffer[Ind][3],8);					  
						ch = SetParameterStr( (uint8_t)rxDataBuffer[Ind][2], (uint8_t*)&gu8_inOutput[(uint8_t)rxDataBuffer[Ind][2]-IP1_HIGH_NAME][0] );
						  
						txDataBuffer[0] = ch;
						txDataBuffer[1] = rxDataBuffer[Ind][2];
						if( ch != ERROR_OK )
						{
							SendPacket(Ind, WRITE_PARAMETER, txDataBuffer, 2);
						}
						else
						{
							memcpy(&txDataBuffer[2],&rxDataBuffer[Ind][3],8);
							SendPacket(Ind, WRITE_PARAMETER, txDataBuffer, 10);
						}
					  
					break;
					  
					default:
					  
						switch((uint8_t)rxDataBuffer[Ind][2])
						{
							case DP1_UPPER_ALARM_ON_LIMIT:
							case DP1_UPPER_ALARM_OFF_LIMIT:
							case DP1_LOWER_ALARM_OFF_LIMIT:
							case DP1_LOWER_ALARM_ON_LIMIT:
							case DP2_UPPER_ALARM_ON_LIMIT:
							case DP2_UPPER_ALARM_OFF_LIMIT:
							case DP2_LOWER_ALARM_OFF_LIMIT:
							case DP2_LOWER_ALARM_ON_LIMIT:
							case DP3_UPPER_ALARM_ON_LIMIT:
							case DP3_UPPER_ALARM_OFF_LIMIT:
							case DP3_LOWER_ALARM_OFF_LIMIT:
							case DP3_LOWER_ALARM_ON_LIMIT:
							case TEMP_UPPER_ALARM_ON_LIMIT:
							case TEMP_UPPER_ALARM_OFF_LIMIT:
							case TEMP_LOWER_ALARM_OFF_LIMIT:
							case TEMP_LOWER_ALARM_ON_LIMIT:
							case RH_UPPER_ALARM_ON_LIMIT:
							case RH_UPPER_ALARM_OFF_LIMIT:
							case RH_LOWER_ALARM_OFF_LIMIT:
							case RH_LOWER_ALARM_ON_LIMIT:
								gu16_AlarmLimitCheckDisableTimer=300;
							break;
						}
						paraValue = atoi( &rxDataBuffer[Ind][3] );
						gu8_SetBySerial=1;
						ch = SetParameter( rxDataBuffer[Ind][2], &paraValue );
						gu8_SetBySerial=0;
						txDataBuffer[0] = ch;
						txDataBuffer[1] = rxDataBuffer[Ind][2];
						if( ch != ERROR_OK )
						SendPacket(Ind, WRITE_PARAMETER, txDataBuffer, 2);
						else
						{
							locked_sprintf_P( txDataBuffer+2, PSTR("%d"), paraValue );
							SendPacket(Ind, WRITE_PARAMETER, txDataBuffer, strlen(txDataBuffer+2)+2);
						}
						
					break;
				}

				break;
			default:
		        txDataBuffer[0] = ERROR_CMD_INVALID;
				SendPacket(Ind, cmdId[Ind], txDataBuffer, 1);
				break;
		}
        //OSSemaGive(SerialDeviceMutex);
		if(Ind==0)
		{
			msgRcvd = 0;
		}
		else 
		{
			msgRcvd1 = 0;
		}
		
		rxDataLen[Ind] = 0;
    }
}

/* Define a callback function that will be used by multiple timer
 instances.  The callback function does nothing but count the number
 of times the associated timer expires, and stop the timer once the
 timer has expired 10 times.  The count is saved as the ID of the
 timer. */
 void vBaseTimerCallback( TimerHandle_t xBaseTimer )
 {

    /* Optionally do something if the pxTimer parameter is NULL. */
    //configASSERT( xBaseTimer );

	xTimerStop( xBaseTimer, 0 );
	
	msgRcvd=0;
	rxMode=0;
	rxDataLen[0]=0;
	
	msgRcvd1=0;
	rxMode1=0;
	rxDataLen[1]=0;
 }


uint8_t CalCRC(uint8_t *ptr,uint16_t NoOfByte)
{
	uint32_t Total=0x00000055;
	uint16_t i=0;
	
	for (i=0;i<NoOfByte;i++)
	{
		Total += (unsigned long)*(ptr + i);
	}
	
	if(Total > 0x0000007F) Total &= 0x0000007F;

	return ((uint8_t)Total);
}


static void SendPacket( uint8_t comId, uint8_t cId, char * val, uint8_t len )
{
	//uint8_t buff1[3],buff2[2];
   
	uint8_t ckSum = 0x55, i;
	uint8_t devId = GetParameterValue( DEVICE_ID );

	//   TX_ENB_RS485;
	ckSum = ( ckSum + devId ) & 0x7F;
	ckSum = ( ckSum + cId ) & 0x7F;
	for(i = len; i; i--)
	ckSum = (ckSum + val[i-1]) & 0x7F;

	if(!comId)
	{
		TX0_ENB_RS485;
		put_char(&USARTC0, SERIAL_RESP_SOT );
		put_char(&USARTC0, devId );
		put_char(&USARTC0, cId );
		for( i = 0; i < len; i++) put_char(&USARTC0, val[i] );
		put_char(&USARTC0, ckSum );
		put_char(&USARTC0, SERIAL_RESP_EOT );
		TX0_DIS_RS485;
	}
	else
	{
		TX1_ENB_RS485;
		put_char(&USARTC1, SERIAL_RESP_SOT );
		put_char(&USARTC1, devId );
		put_char(&USARTC1, cId );
		for( i = 0; i < len; i++) put_char(&USARTC1, val[i] );
		put_char(&USARTC1, ckSum );
		put_char(&USARTC1, SERIAL_RESP_EOT );
		TX1_DIS_RS485;
	}
	
	//buff1[0] = SERIAL_RESP_SOT;
	//buff1[1] = devId;
	//buff1[2] = cId;
	//
	//buff2[0] = ckSum;
	//buff2[1] = SERIAL_RESP_EOT;
   
    //TX0_ENB_RS485;
   	//for(i = 0; i < 3; i++) xQueueSend( vUSARTC0_TxQueue, &buff1[i], 0);
   	//for(i = 0; i < len; i++) xQueueSend( vUSARTC0_TxQueue, &val[i], 0);
   	//for(i = 0; i < 2; i++) xQueueSend( vUSARTC0_TxQueue, &buff2[i], 0);
   	//USARTC0_CTRLA |= (USART_DREINTLVL_HI_gc);
	
	//for(i = 0; i < 3; i++) xQueueSend( vUSARTC1_TxQueue, &buff1[i], 0);
	//for(i = 0; i < len; i++) xQueueSend( vUSARTC1_TxQueue, &val[i], 0);
	//for(i = 0; i < 2; i++) xQueueSend( vUSARTC1_TxQueue, &buff2[i], 0);
	//USARTC1_CTRLA |= (USART_DREINTLVL_HI_gc);
}

static TIME curTime;
static DATE curDate;
static void SerialPrint( char * str )
{
   uint8_t mon;
   if( RTCGetTime( &curTime ) == (uint8_t)ERROR_OK && RTCGetDate( &curDate ) == (uint8_t)ERROR_OK )
   {
	  mon = (curDate.month >> 4) * 10 + (curDate.month & 0xf );
	  if( mon < (uint8_t)1 || mon > (uint8_t)12 )
	     mon = 0;
   
//      TX_ENB_RS485;
      locked_printf_P( PSTR("Module:%03d %c%c%S%c%c %c%c:%c%c:%c%c %s\r"), 
                              GetParameterValue( DEVICE_ID ), 
	                          (curDate.date >> 4) + '0',
	                          (curDate.date & 0xf ) + '0',
							  monthTbl[mon],
	                          (curDate.year >> 4) + '0',
	                          (curDate.year & 0xf ) + '0',
	  	                      (curTime.hours >> 4) + '0',
	  					      (curTime.hours & 0xf ) + '0',
                              (curTime.minutes >> 4) + '0',
	                          (curTime.minutes & 0xf ) + '0',
	                          (curTime.seconds >> 4) + '0',
	                          (curTime.seconds & 0xf ) + '0',
						      str );
//      TX_DIS_RS485;
   }
   else
   {
//      TX_ENB_RS485;
      locked_printf_P( PSTR("Module:%03d %s\r"), GetParameterValue( DEVICE_ID ), str );
//      TX_DIS_RS485;
   }
}



void SerialBroadcast(uint8_t port)
{
   if( IsWirelessBroadcastEnabled() )
   {
      UIResourceLock();
      //OSSemaTakeEver(SerialDeviceMutex);

	if(port==0)
		SendPacket(0, ALL_RUNTIME_VALUES_BRDCAST, txDataBuffer, MakeAllRuntimeParaXmitSrting());
	else if(port==1)
		SendPacket(1, ALL_RUNTIME_VALUES_BRDCAST, txDataBuffer, MakeAllRuntimeParaXmitSrting());
	else if(port==2)
	{
		SendPacket(0, ALL_RUNTIME_VALUES_BRDCAST, txDataBuffer, MakeAllRuntimeParaXmitSrting());
		SendPacket(1, ALL_RUNTIME_VALUES_BRDCAST, txDataBuffer, MakeAllRuntimeParaXmitSrting());
	}

/*
      if (IsDP1Enabled())
      {
         GetPareValue(DP1_VAL_INDEX, &senVal);
         txDataBuffer[0] = senVal.errorCode;
		 locked_sprintf_P( txDataBuffer+1, PSTR("%d"), senVal.convertedValue );
         SendPacket(Ind, DP1_BRDCAST, txDataBuffer, strlen(txDataBuffer+1)+1);
      }
      if (IsDP2Enabled())
      {
         GetPareValue(DP2_VAL_INDEX, &senVal);
         txDataBuffer[0] = senVal.errorCode;
		 locked_sprintf_P( txDataBuffer+1, PSTR("%d"), senVal.convertedValue );
         SendPacket(Ind, DP2_BRDCAST, txDataBuffer, strlen(txDataBuffer+1)+1 );
      }
	  if (IsDP3Enabled())
	  {
		  GetPareValue(DP3_VAL_INDEX, &senVal);
		  txDataBuffer[0] = senVal.errorCode;
		  locked_sprintf_P( txDataBuffer+1, PSTR("%d"), senVal.convertedValue );
		  SendPacket(Ind, DP3_BRDCAST, txDataBuffer, strlen(txDataBuffer+1)+1 );
	  }
      if (IsTemperatureEnabled())
      {
         GetPareValue(TEMPERATURE_VAL_INDEX, &senVal);
         txDataBuffer[0] = senVal.errorCode;
   		locked_sprintf_P( txDataBuffer+1, PSTR("%d"), senVal.convertedValue );
         SendPacket(Ind, TEMPERATURE_BRDCAST, txDataBuffer, strlen(txDataBuffer+1)+1 );
      }
      if (IsHumidityEnabled())
      {
         GetPareValue(HUMIDITY_VAL_INDEX, &senVal);
         txDataBuffer[0] = senVal.errorCode;
         locked_sprintf_P( txDataBuffer+1, PSTR("%d"), senVal.convertedValue );
         SendPacket(Ind, HUMIDITY_BRDCAST, txDataBuffer, strlen(txDataBuffer+1)+1 );
      }
      if( systemError != (uint8_t)ERROR_OK )
      {
         txDataBuffer[0] = systemError;
         SendPacket(Ind, SYSTEMERR_BRDCAST, txDataBuffer, 1 );
      }
*/
      //OSSemaGive(SerialDeviceMutex);
      UIResourceUnlock();
   }
}


void SerialPrinter()
{
   if( IsSerialPrinterEnabled() )
   {
      UIResourceLock();
      //OSSemaTakeEver(SerialDeviceMutex);

      if (IsDP1Enabled())
      {
         GetPareValue(DP1_VAL_INDEX, &senVal);//GetDP1( &senVal );
         SerialPrint( FormatDP1OneLine( txDataBuffer, senVal.errorCode, senVal.convertedValue ));
      }
      if (IsDP2Enabled())
      {
         GetPareValue(DP2_VAL_INDEX, &senVal);//GetDP2( &senVal );
         SerialPrint( FormatDP2OneLine( txDataBuffer, senVal.errorCode, senVal.convertedValue ));
      }
	  if (IsDP3Enabled())
	  {
		  GetPareValue(DP3_VAL_INDEX, &senVal);//GetDP3( &senVal );
		  SerialPrint( FormatDP3OneLine( txDataBuffer, senVal.errorCode, senVal.convertedValue ));
	  }
      if (IsTemperatureEnabled())
      {
         GetPareValue(TEMPERATURE_VAL_INDEX, &senVal);//GetTemperature( &senVal );
         SerialPrint( FormatTemperatureOneLine ( txDataBuffer, senVal.errorCode, senVal.convertedValue, 2 ));
      }
      if (IsHumidityEnabled())
      {
         GetPareValue(HUMIDITY_VAL_INDEX, &senVal);//GetHumidity( &senVal );
         SerialPrint( FormatHumidityOneLine( txDataBuffer, senVal.errorCode, senVal.convertedValue, 2 ));
      }
      if( systemError != (uint8_t)ERROR_OK )
      {
         SerialPrint( FormatSystemError( txDataBuffer, systemError ));
      }

      //OSSemaGive(SerialDeviceMutex);
      UIResourceUnlock();
   }
}


//void SerialBroadcastAlarm( uint8_t alarmData )
//{
   //if( IsWirelessBroadcastEnabled())  // Broadcast enabled
   //{
      ////OSSemaTakeEver(SerialDeviceMutex);
////      TX_ENB_RS485;
      //txDataBuffer[0] = alarmData & 0x3;
	  //alarmData >>= 2;
      //txDataBuffer[1] = alarmData & 0x3;
	  //alarmData >>= 2;
      //txDataBuffer[2] = alarmData & 0x3;
	  //alarmData >>= 2;
      //txDataBuffer[3] = alarmData & 0x3;
      //SendPacket(Ind, ALARM_BRDCAST, txDataBuffer, 4 );
////      TX_DIS_RS485;
      ////OSSemaGive(SerialDeviceMutex);
      //SerialBroadcast();
   //}
//}


void SerialPrintAlarm( uint8_t alarmId, uint8_t state )
{
   const char * statStr = state ? PSTR( "ON" ) : PSTR( "OFF" ); 
   const char * module = PSTR( "Module:" ); 
   const char * alarm = PSTR( "ALARM:" ); 
   const char * upper = PSTR( "UPPER" );
   const char * lower = PSTR( "LOWER" );
   const char * pAbs = PSTR( "P-ABS" );
   const char * pDiff = PSTR( "P-DIFF" );
   const char * temp = PSTR( "TEMP" );
   const char * humi = PSTR( "HUMI" );
   const char * fmt = PSTR( "%S%03d %S %S %S %S\r" );
   unsigned char devId;

   if( IsSerialPrinterEnabled())
   {
      //OSSemaTakeEver(SerialDeviceMutex);
	  devId = GetParameterValue( DEVICE_ID );
//      TX_ENB_RS485;
      switch( alarmId )
      {
         case ALARM_UPPER_DP1:
            locked_printf_P( fmt, module, devId, alarm, pAbs, upper, statStr );
            break;
         case ALARM_LOWER_DP1:
            locked_printf_P( fmt, module, devId, alarm, pAbs, lower, statStr );
            break;
         case ALARM_UPPER_DP2:
            locked_printf_P( fmt, module, devId, alarm, pDiff, upper, statStr );
            break;
         case ALARM_LOWER_DP2:
            locked_printf_P( fmt, module, devId, alarm, pDiff, lower, statStr );
            break;
		 case ALARM_UPPER_DP3:
			locked_printf_P( fmt, module, devId, alarm, pDiff, upper, statStr );
			break;
		 case ALARM_LOWER_DP3:
			locked_printf_P( fmt, module, devId, alarm, pDiff, lower, statStr );
			 break;
         case ALARM_UPPER_TEMPERATURE:
            locked_printf_P( fmt, module, devId, alarm, temp, upper, statStr );
            break;
         case ALARM_LOWER_TEMPERATURE:
            locked_printf_P( fmt, module, devId, alarm, temp, lower, statStr );
            break;
         case ALARM_UPPER_HUMIDITY:
            locked_printf_P( fmt, module, devId, alarm, humi, upper, statStr );
            break;
         case ALARM_LOWER_HUMIDITY:
            locked_printf_P( fmt, module, devId, alarm, humi, lower, statStr );
            break;
      }
      //OSSemaGive(SerialDeviceMutex);
//      TX_DIS_RS485;
      SerialPrinter();
   }
}

void USART_SetBaudrate(USART_t * usart, int16_t bsel,int8_t bscale)
{
	usart->BAUDCTRLA = (bsel & 0xff) << USART_BSEL_gp;
	usart->BAUDCTRLB = ((bscale) << USART_BSCALE_gp) | ((bsel >> 8) << USART_BSEL_gp);
}

void InitUSART(USART_t * usart, PORT_t * port, uint8_t rxpin, uint8_t txpin, uint16_t ubrr, uint8_t noOfbits, uint8_t parity, uint8_t stopbit)
{
	int16_t brt_bsel=0;
	int8_t brt_bscale=0;
	
	//-----------------------------
	// USART setup ( RXD0, TXD0 )

	/* (TXD0) as output. */
	port->DIRSET = txpin;
	
	/* (RXD0) as input. */
	port->DIRCLR = rxpin;
	
	//disable while setting baud rate
	usart->CTRLA=0;
	usart->CTRLC=0;
	
	//for Fosc=32 MHz
	switch(ubrr)
	{
		case 1200:		brt_bscale = -1; 		brt_bsel = 3331;	break;
		case 2400:		brt_bscale = -2; 		brt_bsel = 3329;	break;
		case 4800:		brt_bscale = -3; 		brt_bsel = 3325;	break;
		case 9600:		brt_bscale = -4;		brt_bsel = 3317;	break;
		case 14400:		brt_bscale = -4; 		brt_bsel = 2206;	break;
		case 19200:		brt_bscale = -5; 		brt_bsel = 3301;	break;
		case 28800:		brt_bscale = -5; 		brt_bsel = 2190;	break;
		case 38400:		brt_bscale = -6; 		brt_bsel = 3269;	break;
		case 57600:		brt_bscale = -6;		brt_bsel = 2158; 	break;
	}
	
	USART_SetBaudrate(usart, brt_bsel, brt_bscale);
	
	switch(noOfbits)
	{
		case DATABIT_5:																	break;
		case DATABIT_6:		usart->CTRLC |= USART_CHSIZE0_bm;							break;
		case DATABIT_7:		usart->CTRLC |= USART_CHSIZE1_bm;							break;
		case DATABIT_8:		usart->CTRLC |= (USART_CHSIZE1_bm | USART_CHSIZE0_bm);		break;
		default:    																	break;	// 8 bit
	}

	switch(parity)
	{
		case SERIAL_PARITY_EVEN:	usart->CTRLC |= USART_PMODE1_bm;							break;
		case SERIAL_PARITY_ODD:		usart->CTRLC |= (USART_PMODE1_bm | USART_PMODE0_bm);		break;
		case SERIAL_PARITY_NONE:																break;
	}
	
	switch(stopbit)
	{
		case STOPBIT_2:	usart->CTRLC |= USART_SBMODE_bm;		break;
		case STOPBIT_1:											break;
		default:												break;	//1 bit
	}

	usart->CTRLB = USART_RXEN_bm | USART_TXEN_bm;
	//USARTC0_CTRLA = (USART_RXCINTLVL_HI_gc | USART_TXCINTLVL_HI_gc | USART_DREINTLVL_HI_gc);
	usart->CTRLA = USART_RXCINTLVL_HI_gc;
}


void SerialInterfaceInit()
{
   uint16_t baudrate;
   uint16_t databits;
   uint8_t noOfbits, parity, stopbit;

   TX0_SETPORT_DIR;
   TX1_SETPORT_DIR;
   TX0_DIS_RS485;
   TX1_DIS_RS485;
   
   //OSSemaCreateMutex( SerialDeviceMutex );
   //OSSemaGive(SerialDeviceMutex);

   switch( GetParameterValue( SERIAL_BAUD_RATE_VAL ))
   {
      case SERIAL_BAUD_1200: baudrate = 1200;       break;
      case SERIAL_BAUD_2400: baudrate = 2400;           break;
      case SERIAL_BAUD_4800: baudrate = 4800;           break;
      case SERIAL_BAUD_14400: baudrate = 14400;         break;
      case SERIAL_BAUD_19200: baudrate = 19200;         break;
      case SERIAL_BAUD_28800: baudrate = 28800;         break;
      case SERIAL_BAUD_38400: baudrate = 38400;         break;
      case SERIAL_BAUD_57600: baudrate = 57600;         break;
      case SERIAL_BAUD_9600: 
      default:               baudrate = 9600;           break;
   }
   databits = GetParameterValue( SERIAL_DATA_STOP_PARITY_VAL );
   
   noOfbits = (databits & SERIAL_DATA_BITS_MASK) >> SERIAL_DATA_BITS_SHIFT;
   parity = (databits & SERIAL_PARITY_MASK) >> SERIAL_PARITY_SHIFT;
   stopbit = (databits & SERIAL_STOP_BIT_MASK) >> SERIAL_STOP_BIT_SHIFT;
   
   InitUSART(&USARTC0, &PORTC, PIN2_bm, PIN3_bm, baudrate, noOfbits, parity, stopbit);
   InitUSART(&USARTC1, &PORTC, PIN6_bm, PIN7_bm, baudrate, noOfbits, parity, stopbit);
   //InitUSART(&USARTD0, &PORTD, PIN2_bm, PIN3_bm, baudrate, noOfbits, parity, stopbit);
   //InitUSART(&USARTE0, &PORTE, PIN2_bm, PIN3_bm, baudrate, noOfbits, parity, stopbit);

   //vUSARTC0_RxQueue = xQueueCreate(RX0_BUFSZ, (unsigned portBASE_TYPE ) sizeof(signed char));
   //vUSARTC0_TxQueue = xQueueCreate(TX0_BUFSZ, (unsigned portBASE_TYPE ) sizeof(signed char));
   //vUSARTC1_RxQueue = xQueueCreate(RX1_BUFSZ, (unsigned portBASE_TYPE ) sizeof(signed char));
   //vUSARTC1_TxQueue = xQueueCreate(TX1_BUFSZ, (unsigned portBASE_TYPE ) sizeof(signed char));

   // Setup stdout to use serial port
   fdev_setup_stream( &serialStdIo, uart_putchar, NULL, _FDEV_SETUP_WRITE );
   stdout = &serialStdIo;

   if( IsSerialHandshackEnabled())
   {  // Need to run thread only if Command/Response protocol is selected.
		/* Spawn the producer and consumer. */
		//xTaskCreate( SerialInterfaceTask, "SI", SERTASK_STACK_SIZE, ( void * ) 0, SERTASK_PRIORITY, NULL );		
   }
}


static int uart_putchar(char c, FILE *stream)
{
   put_char(&USARTC0, c);
   return 0;
}

static int MakeAllRuntimeParaXmitSrting()
{
   uint8_t i = 0;
  uint16_t tempshort = 0;
  	
   i = 0;
   if (IsDP1Enabled())
   {
      GetPareValue(DP1_VAL_INDEX, &senVal);//GetDP1( &senVal );
      txDataBuffer[i++] = senVal.errorCode;
      if( senVal.errorCode == 0 )
	   {
         locked_sprintf_P( &txDataBuffer[i], PSTR("%d"), senVal.convertedValue );
		 i += strlen(&txDataBuffer[i]);
	   }
   }
	else
	{
       txDataBuffer[i++] = ERROR_CMD_INVALID;
	}
	txDataBuffer[i++] = SERIAL_RESP_SEP;

   if (IsDP2Enabled())
   {
      GetPareValue(DP2_VAL_INDEX, &senVal);//GetDP2( &senVal );
      txDataBuffer[i++] = senVal.errorCode;
	   if( senVal.errorCode == 0 )
	   {
         locked_sprintf_P( &txDataBuffer[i], PSTR("%d"), senVal.convertedValue );
		 i += strlen(&txDataBuffer[i]);
	   }
   }
	else
	{
       txDataBuffer[i++] = ERROR_CMD_INVALID;
	}
	txDataBuffer[i++] = SERIAL_RESP_SEP;

	if (IsDP3Enabled())
	{
		GetPareValue(DP3_VAL_INDEX, &senVal);//GetDP3( &senVal );
		txDataBuffer[i++] = senVal.errorCode;
		if( senVal.errorCode == 0 )
		{
			locked_sprintf_P( &txDataBuffer[i], PSTR("%d"), senVal.convertedValue );
			i += strlen(&txDataBuffer[i]);
		}
	}
	else
	{
		txDataBuffer[i++] = ERROR_CMD_INVALID;
	}
	txDataBuffer[i++] = SERIAL_RESP_SEP;

    if (IsTemperatureEnabled())
    {
           GetPareValue(TEMPERATURE_VAL_INDEX, &senVal);
       txDataBuffer[i++] = senVal.errorCode;
	   if( senVal.errorCode == 0 )
	   {
          locked_sprintf_P( &txDataBuffer[i], PSTR("%d"), senVal.convertedValue );
		  i += strlen(&txDataBuffer[i]);
	   }
    }
	else
	{
       txDataBuffer[i++] = ERROR_CMD_INVALID;
	}
	txDataBuffer[i++] = SERIAL_RESP_SEP;
	
	if (IsTemperature2Enabled())
	{
		GetPareValue(TEMPERATURE2_VAL_INDEX, &senVal);
		txDataBuffer[i++] = senVal.errorCode;
		if( senVal.errorCode == 0 )
		{
			locked_sprintf_P( &txDataBuffer[i], PSTR("%d"), senVal.convertedValue );
			i += strlen(&txDataBuffer[i]);
		}
	}
	else
	{
		txDataBuffer[i++] = ERROR_CMD_INVALID;
	}
	txDataBuffer[i++] = SERIAL_RESP_SEP;

    if (IsHumidityEnabled())
    {
           GetPareValue(HUMIDITY_VAL_INDEX, &senVal);
       txDataBuffer[i++] = senVal.errorCode;
	   if( senVal.errorCode == 0 )
	   {
          locked_sprintf_P( &txDataBuffer[i], PSTR("%d"), senVal.convertedValue );
		  i += strlen(&txDataBuffer[i]);
	   }
    }
	else
	{
       txDataBuffer[i++] = ERROR_CMD_INVALID;
	}
	txDataBuffer[i++] = SERIAL_RESP_SEP;
	
	 if (IsTemperature2Enabled())
	 {
		 GetPareValue(HUMIDITY2_VAL_INDEX, &senVal);
		 txDataBuffer[i++] = senVal.errorCode;
		 if( senVal.errorCode == 0 )
		 {
			 locked_sprintf_P( &txDataBuffer[i], PSTR("%d"), senVal.convertedValue );
			 i += strlen(&txDataBuffer[i]);
		 }
	 }
	 else
	 {
		 txDataBuffer[i++] = ERROR_CMD_INVALID;
	 }
	 txDataBuffer[i++] = SERIAL_RESP_SEP;
	 
	//if (IsTemperatureEnabled() && IsHumidityEnabled())
	//{
		//txDataBuffer[i++] = PreTdErr;
		//if( PreTdErr == 0 )
		//{
			//locked_sprintf_P( &txDataBuffer[i], PSTR("%d"), PreTd );
			//i += strlen(&txDataBuffer[i]);
		//}
	//}
	//else
	//{
		//txDataBuffer[i++] = ERROR_CMD_INVALID;
	//}
	//txDataBuffer[i++] = SERIAL_RESP_SEP;
	
	//if (IsTemperature2Enabled())
	//{
		//txDataBuffer[i++] = PostTdErr;
		//if( PostTdErr == 0 )
		//{
			//locked_sprintf_P( &txDataBuffer[i], PSTR("%d"), PostTd );
			//i += strlen(&txDataBuffer[i]);
		//}
	//}
	//else
	//{
		//txDataBuffer[i++] = ERROR_CMD_INVALID;
	//}
	//txDataBuffer[i++] = SERIAL_RESP_SEP;
	
	//if (IsTemperatureEnabled() && IsHumidityEnabled() && IsTemperature2Enabled())
	//{
		//txDataBuffer[i++] = TDdiffErr;
		//if( TDdiffErr == 0 )
		//{
			//locked_sprintf_P( &txDataBuffer[i], PSTR("%d"), TDdiff );
			//i += strlen(&txDataBuffer[i]);
		//}
	//}
	//else
	//{
		//txDataBuffer[i++] = ERROR_CMD_INVALID;
	//}
	//txDataBuffer[i++] = SERIAL_RESP_SEP;
	
	 
   txDataBuffer[i++] = GetSystemError();
   txDataBuffer[i++] = SERIAL_RESP_SEP;
   tempshort = GetAlarms();
   txDataBuffer[i++] = tempshort & 0x3;
   tempshort >>= 2;
   txDataBuffer[i++] = tempshort & 0x3;
   tempshort >>= 2;
   txDataBuffer[i++] = tempshort & 0x3;
   tempshort >>= 2;
   txDataBuffer[i++] = tempshort & 0x3;
   tempshort >>= 2;
   txDataBuffer[i++] = tempshort & 0x3;
   tempshort >>= 2;
   txDataBuffer[i++] = tempshort & 0x3;
   txDataBuffer[i++] = SERIAL_RESP_SEP;

   paraValue = GetAbsPresPIDPercent();
   txDataBuffer[i++] = ((char *)&paraValue)[0];
   txDataBuffer[i++] = ((char *)&paraValue)[1];					
   paraValue = GetDiffPresPIDPercent();
   txDataBuffer[i++] = ((char *)&paraValue)[0];
   txDataBuffer[i++] = ((char *)&paraValue)[1];					
   paraValue = GetTempPIDPercent();
   txDataBuffer[i++] = ((char *)&paraValue)[0];
   txDataBuffer[i++] = ((char *)&paraValue)[1];					
   paraValue = GetRHPIDPercent();
   txDataBuffer[i++] = ((char *)&paraValue)[0];
   txDataBuffer[i++] = ((char *)&paraValue)[1];					
   txDataBuffer[i++] = SERIAL_RESP_SEP;
   
   GetInput( (uint8_t *)&txDataBuffer[i++] );		// Input 
   GetOutput( (uint8_t *)&txDataBuffer[i++] );		// Output
   txDataBuffer[i++] = SERIAL_RESP_SEP;
   
   txDataBuffer[i++] = PRES_SENS_SM9543;
   txDataBuffer[i++] = (uint8_t)GetParameterValue(DP1_UNIT);
   paraValue =  GetParameterValue(DP1_RANGE);
   txDataBuffer[i++] = ((char *)&paraValue)[0];
   txDataBuffer[i++] = ((char *)&paraValue)[1];
   txDataBuffer[i++] = SERIAL_RESP_SEP;
   
   txDataBuffer[i++] = PRES_SENS_SM9543;
   txDataBuffer[i++] = (uint8_t)GetParameterValue(DP2_UNIT);
   paraValue =  GetParameterValue(DP2_RANGE);
   txDataBuffer[i++] = ((char *)&paraValue)[0];
   txDataBuffer[i++] = ((char *)&paraValue)[1];
   txDataBuffer[i++] = SERIAL_RESP_SEP;
   
   txDataBuffer[i++] = PRES_SENS_SM9543;
   txDataBuffer[i++] = (uint8_t)GetParameterValue(DP3_UNIT);
   paraValue =  GetParameterValue(DP3_RANGE);
   txDataBuffer[i++] = ((char *)&paraValue)[0];
   txDataBuffer[i++] = ((char *)&paraValue)[1];
   	
   return i;
}

//void software_reset(void)
//{
	//CPU_CCP  = CCP_IOREG_gc;
	//RST.CTRL = RST_SWRST_bm ;
//}

short findValue(char *ptr,unsigned char NoOfDigit)
{
	unsigned short Value=0,value1=1;
	unsigned char minus=0;
	
	if(*ptr=='-') 
	{
		ptr += (NoOfDigit-1);
		NoOfDigit--;
		minus=1;
	}
	else
	{
		ptr += (NoOfDigit-1);
	}
	
	
	while(NoOfDigit)
	{
		//*ptr -= '0';
		Value += ((unsigned short)(*ptr - '0') * value1);
		
		ptr--;
		NoOfDigit--;
		
		value1 *= 10;
	}
	
	if(minus) Value *= (-1);
	
	return Value;
}

