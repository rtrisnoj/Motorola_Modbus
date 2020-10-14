/*
 * EchoSensor.cpp
 * Status Message
 * Created: 10/08/2020
 *  Author: Ryan T
 */ 

#include "EchoSensor.h"
#include "sapi.h"

static char		 echostring[32];
static uint32_t	 echocount;

int sendInterval3 = ParamSendInterval();
int sampleRate3 =  ParamSampleRate();

sapi_error_t echo_read_sensor(char *payload, uint8_t *len)
{
	// Assemble the Payload
	char    rsendInterval[] = "0";
	char    rsampleRate[] = "0,"; //if it shows 2, float not connected properly. The value should be 0 or 1.
	
	sprintf(rsendInterval, "%d,", sendInterval3);
	sprintf(rsampleRate, "%d,", sampleRate3);
	
	strcpy(payload, "");
	strcat(payload, "SI:");
	strcat(payload, rsendInterval);
	strcat(payload, ";");
	strcat(payload, "SR:");
	strcat(payload, rsampleRate);
	strcat(payload, ";");

	*len = strlen(payload);
	
	dlog(LOG_DEBUG, "Echo Payload: %s", payload);
    return SAPI_ERR_OK;
}


sapi_error_t echo_write_cfg(char *payload, uint8_t *len)
{
	if (strncmp(payload, "cfg=",4) == 0)
	{
		strcpy(echostring, &payload[4]);
		echocount = 1;
		
		return SAPI_ERR_OK;
	}
	else
	{
		return SAPI_ERR_NOT_IMPLEMENTED;
	}
}


sapi_error_t echo_init_sensor()
{
	strcpy(echostring, "Echo Echo Echo");
	
	echocount = 1;
	
	dlog(LOG_DEBUG, "Initialized Echo Sensor");
	return SAPI_ERR_OK;
}