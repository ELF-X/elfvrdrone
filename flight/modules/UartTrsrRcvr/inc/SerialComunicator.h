#ifndef _SERIALCOMUNICATOR_H_
#define _SERIALCOMUNICATOR_H_

#include "pios.h"
#include <pios_stm32.h>


#define INBUF_SIZE 64

#define MSP_SET_RAW_RC_TINY			150
#define MSP_ARMING               	161
#define MSP_ARM                  	151
#define MSP_DISARM               	152
#define MSP_BATTERY_VOLTAGE		 	153
#define MSP_GET_VERSION				154
#define MSP_CALIBRATION				155
#define MSP_ALTITUDE				156
#define MSP_GET_FLIGHT_STATUS		157

#define RECV_BUFFER_LENGTH		20
#define SEND_BUFFER_LENGTH		20

extern void recievedData(const unsigned char *pData, int32_t len);
extern uint32_t read32();
extern uint16_t read16();
extern uint8_t read8();

extern void sendCommand(unsigned char cmd);
extern void sendData(unsigned char cmd, unsigned char *dat, int16_t len);
extern void processData(const unsigned char *pData, int16_t len);

#endif


