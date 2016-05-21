/**
 ******************************************************************************
 * @brief BleRcvr test
 *
 * @file       UartRcvr.c
 * @author     Richile.
 * @brief      UartRcvr test
 *
 *****************************************************************************/

#include <openpilot.h>
#include "hwsettings.h"
#include "taskinfo.h"
#include "UartTrsrRcvr.h"
#include "SerialComunicator.h"
#include "flightstatus.h"

#include <stdbool.h>
#include <pios_mem.h>


// Private functions
static void uartRcvrTask(void *parameters);
static void additionalTask( void *parameters);
static void updateSettings();
static void FlightStatusUpdate(UAVObjEvent *ev);
// Private constants

#define UART_RCVR_STACK_SIZE_BYTES		78
#define VERSION_ID						35
#define TASK_PRIORITY        			(tskIDLE_PRIORITY + 1)
#define UART_RCVR_BUF_LEN       		20
#define GRAVITY           				9.805f

static FlightStatusData flightStatus;
static xTaskHandle uartRcvrTaskHandle;
static xTaskHandle additionalTaskHandle;
static xSemaphoreHandle sem_busy;

/* Com port */
static uint32_t usart_port;
static uint8_t uart_rcvr_buf[UART_RCVR_BUF_LEN];
static bool uart_rcvr_enabled = true;

/* Uart rcvr */
static struct uart_rcvr_dev *uart_dev = 0;
static int32_t Uart_Get(uint32_t rcvr_id, uint8_t channel);
const struct pios_rcvr_driver uart_rcvr_driver = {
    .read          = Uart_Get,
};

/**
 * Get channel value 
 * \return channel value
 */
static int32_t Uart_Get(uint32_t rcvr_id, uint8_t channel)
{
	if(channel > BLE_MAX_NUM_CHANNELS || rcvr_id == 0)
		return PIOS_RCVR_INVALID;
	struct uart_rcvr_dev *rcvr_dev = (struct uart_rcvr_dev *)rcvr_id;

	return rcvr_dev->channelVal[channel];
}

/**
 * Start the module
 * \return -1 if initialisation failed
 * \return 0 on success
 */
static int32_t uartRcvrStart(void)
{
    if (uart_rcvr_enabled && usart_port) {
        // Start tasks
        xTaskCreate(uartRcvrTask, "UartRcvr", UART_RCVR_STACK_SIZE_BYTES, NULL, TASK_PRIORITY, &uartRcvrTaskHandle);
		xTaskCreate(additionalTask, "AdditionalTask", UART_RCVR_STACK_SIZE_BYTES, NULL, TASK_PRIORITY, &additionalTaskHandle);
        return 0;
    }

    return -1;
}

/**
 * Initialise the module
 * \return -1 if initialisation failed
 * \return 0 on success
 */
static int32_t uartRcvrInitialize(void)
{
	usart_port     = PIOS_COM_UART_RCVR;
	
    if (uart_rcvr_enabled && usart_port) {
        updateSettings();
		sem_busy = xSemaphoreCreateMutex();
    }

	FlightStatusInitialize();
	FlightStatusConnectCallback(FlightStatusUpdate);
    return 0;
}
MODULE_INITCALL(uartRcvrInitialize, uartRcvrStart);

/**
 * Called if altitudeHoldDesired is updated
 */
static void FlightStatusUpdate(__attribute__((unused)) UAVObjEvent *ev)
{
	FlightStatusGet(&flightStatus);
}

/**
 * Additional task to send voltage of battery, 
 * calibrate and send version id
 */
static void additionalTask(__attribute__((unused)) void *parameters)
{
	portTickType lastSysTime;

    lastSysTime = xTaskGetTickCount();

	uint8_t cnt = 0;

	while(1){
		if(cnt >= 30){ // 900 ms
			cnt = 0;
		}	
		cnt++;
		vTaskDelayUntil(&lastSysTime, 30 / portTICK_RATE_MS);
	}
}

/**
 * Get uart data task
 */
static void uartRcvrTask(__attribute__((unused)) void *parameters)
{
	FlightStatusGet(&flightStatus);
	uint8_t lastArmed = flightStatus.Armed;
	uint8_t uartIsBreaked = 0;
	uint8_t cnt = 0;
	uint32_t stabilizationThrottle = RC_MIN_VALUE;
    while (1) {
        uint32_t rx_bytes = 0;
		if(usart_port){			
        	rx_bytes = PIOS_COM_ReceiveBuffer(usart_port, uart_rcvr_buf, UART_RCVR_BUF_LEN, 500);
		}
		
        if (rx_bytes > 0) {
			cnt = 0;
			uartIsBreaked = 0;
			stabilizationThrottle = RC_MIN_VALUE;
            recievedData(uart_rcvr_buf, (int32_t)rx_bytes);
        }else{
        	cnt++;
			if(cnt >= 2){
				cnt = 2;
				uartIsBreaked = 1;

				if(stabilizationThrottle == RC_MIN_VALUE && flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED && uart_dev->channelVal[THROTTLE] > (RC_MIN_VALUE +40)){
					stabilizationThrottle = uart_dev->channelVal[THROTTLE] - 40;
				}
			}
		}
		if(!uart_dev)
			continue;
		if(uartIsBreaked == 1){
			uart_dev->channelVal[THROTTLE] = stabilizationThrottle;
			uart_dev->channelVal[ROLL] = RC_MID_VALUE;
			uart_dev->channelVal[PITCH] = RC_MID_VALUE;
		}
		
		FlightStatusGet(&flightStatus);
		if(lastArmed != flightStatus.Armed){
			if (sem_busy != NULL && xSemaphoreTake(sem_busy, 1000 / portTICK_RATE_MS) == pdTRUE) {
				if(flightStatus.Armed == FLIGHTSTATUS_ARMED_DISARMED){
					sendCommand(MSP_DISARM);
				}else if(flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMING){
					sendCommand(MSP_ARMING);
				}else if(flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED){
					sendCommand(MSP_ARM);
				}
				xSemaphoreGive(sem_busy);
			}
			lastArmed = flightStatus.Armed;
		}
    }
}

int32_t Write(uint8_t *pData, uint16_t len)
{
	return PIOS_COM_SendBuffer(usart_port, pData, len);
}

uint16_t Read(uint8_t *buf, uint16_t buf_len, uint32_t timeout_ms)
{
	return PIOS_COM_ReceiveBuffer(usart_port, buf, buf_len, timeout_ms);
}



static void updateSettings()
{
    if (usart_port) {
		PIOS_COM_ChangeBaud(usart_port, 115200);
    }
}

uint8_t PIOS_UART_RCVR_Init(uint32_t *uart_rcvr_id)
{
	PIOS_DEBUG_Assert(uart_rcvr_id);

    uart_dev = (struct uart_rcvr_dev *)pios_malloc(sizeof(struct uart_rcvr_dev));
    if (!uart_dev) {
        goto out_fail;
    }
	uart_dev->channelVal[ROLL] = RC_MID_VALUE;
	uart_dev->channelVal[PITCH] = RC_MID_VALUE;
	uart_dev->channelVal[YAW] = RC_MID_VALUE;
	uart_dev->channelVal[THROTTLE] = RC_MIN_VALUE;
	uart_dev->channelVal[AUX1] = RC_MIN_VALUE;
	uart_dev->channelVal[AUX2] = RC_MIN_VALUE;
	uart_dev->channelVal[AUX3] = RC_MIN_VALUE;
	uart_dev->channelVal[AUX4] = RC_MIN_VALUE;

	*uart_rcvr_id = (uint32_t)uart_dev;

    return 0;

out_fail:
    return -1;
}


void OnEvaluateCommand(uint8_t cmd)
{
	switch(cmd){
		case MSP_SET_RAW_RC_TINY:{
			uart_dev->channelVal[ROLL] = RC_MIN_VALUE + read8() * 4;
			uart_dev->channelVal[PITCH] = RC_MIN_VALUE + read8() * 4;
			uart_dev->channelVal[YAW] = RC_MIN_VALUE + read8() * 4;
			uart_dev->channelVal[THROTTLE] = RC_MIN_VALUE + read8() * 4;

			uint32_t temp = uart_dev->channelVal[THROTTLE] - RC_MIN_VALUE;
			float s = ((float)temp)/((float)(RC_MAX_VALUE-RC_MIN_VALUE));
			if(s < 0.05f){
				uart_dev->channelVal[THROTTLE] = RC_MIN_VALUE;
			}else{
				uint32_t scaleValue = 1000.0f*s;
				scaleValue = sqrtf(s*100.0f)*81.0f;
				uart_dev->channelVal[THROTTLE] = RC_MIN_VALUE + 200 + scaleValue;
				if(uart_dev->channelVal[THROTTLE] > RC_MAX_VALUE)
					uart_dev->channelVal[THROTTLE] = RC_MAX_VALUE;
			}

			unsigned char auxChannels = read8();
			unsigned char aux = (auxChannels & 0xc0) >> 6;
			if(aux == 0){
				uart_dev->channelVal[AUX1] = RC_MIN_VALUE;
 			}else if(aux == 1){
				uart_dev->channelVal[AUX1] = RC_MID_VALUE;
			}else{
				uart_dev->channelVal[AUX1] = RC_MAX_VALUE;
			}

			aux = (auxChannels & 0x30) >> 4;
			if(aux == 0){
				uart_dev->channelVal[AUX2] = RC_MIN_VALUE;
			}else if(aux == 1){
				uart_dev->channelVal[AUX2] = RC_MID_VALUE;
			}else{
				uart_dev->channelVal[AUX2] = RC_MAX_VALUE;
			}
     
			aux = (auxChannels & 0x0c) >> 2;
			if(aux == 0){
				uart_dev->channelVal[AUX3] = RC_MIN_VALUE;
			}else if(aux == 1){
				uart_dev->channelVal[AUX3] = RC_MID_VALUE;
			}else{
				uart_dev->channelVal[AUX3] = RC_MAX_VALUE;
			}

			aux = (auxChannels & 0x03);
			if(aux == 0){
				uart_dev->channelVal[AUX4] = RC_MIN_VALUE;
			}else if(aux == 1){
				uart_dev->channelVal[AUX4] = RC_MID_VALUE;
			}else{
				uart_dev->channelVal[AUX4] = RC_MAX_VALUE;
			}
		}break;
		case MSP_ARM:{
			sendCommand(MSP_ARM);
		}break;
		case MSP_DISARM:{
			sendCommand(MSP_DISARM);
		}break;
		case MSP_BATTERY_VOLTAGE:{
			
		}break;
		case MSP_GET_VERSION:{ 
			unsigned char versionId = VERSION_ID;
			sendData(MSP_GET_VERSION, &versionId, sizeof(versionId));
		}break;
		case MSP_CALIBRATION:{

		}break;
		case MSP_GET_FLIGHT_STATUS:{
			FlightStatusGet(&flightStatus);
			if (sem_busy != NULL && xSemaphoreTake(sem_busy, 1000 / portTICK_RATE_MS) == pdTRUE) {
				if(flightStatus.Armed == FLIGHTSTATUS_ARMED_DISARMED){
					sendCommand(MSP_DISARM);
				}else if(flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMING){
					sendCommand(MSP_ARMING);
				}else if(flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED){
					sendCommand(MSP_ARM);
				}
				xSemaphoreGive(sem_busy);
			}
		}break;
		default:break;
	}
}




