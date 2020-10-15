/*

Copyright (c) Silver Spring Networks, Inc.
All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the ""Software""), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED AS IS, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Except as contained in this notice, the name of Silver Spring Networks, Inc.
shall not be used in advertising or otherwise to promote the sale, use or other
dealings in this Software without prior written authorization from Silver Spring
Networks, Inc.

*/
#include "TempSensor.h"
#include <Filters.h>
#include "sapi.h"

// DHT11 Sensor Object
#define DHT_TYPE           DHT11
DHT_Unified dht(A1, DHT_TYPE);

// Sensor Context. Contains the unit of measure and alert state.
static temp_ctx_t context;

int counter1 = 0;
char 		payloadFinal[128];


//////////////////////////////////////////////////////////////////////////
//
// Initialization functions. Callback functions. Payload building functions.
//
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//
// Initialize DHT11 temp sensor. Callback called by sapi_init_sensor function.
//
//////////////////////////////////////////////////////////////////////////
sapi_error_t temp_init_sensor()
{
	// Set context defaults and enable the sensor
	context.scalecfg = FAHRENHEIT_SCALE;
	context.alertstate = tsat_disabled;
	temp_sensor_enable();

	// Initialize temperature/humidity sensor
	dht.begin();

	// Log a banner for the sensor with sensor details
	println("DHT11 Sensor Initialized!");

	sensor_t sensor;
	dht.temperature().getSensor(&sensor);
	println("");
	println("------------------------------------");
	print  ("Sensor:       "); println(sensor.name);
	print  ("Driver Ver:   "); printnum(sensor.version);    println("");
	print  ("Unique ID:    "); printnum(sensor.sensor_id);  println("");
	print  ("Max Value:    "); printnum(sensor.max_value);  println(" C");
	print  ("Min Value:    "); printnum(sensor.min_value);  println(" C");
	print  ("Resolution:   "); printnum(sensor.resolution); println(" C");
	println("------------------------------------");        println("");

	return SAPI_ERR_OK;
}


//////////////////////////////////////////////////////////////////////////
//
// Reads a DHT11 sensor. Builds and returns the payload. Callback called on
//  CoAP Get sensor value
//  CoAP Observe notification of sensor value
//
//////////////////////////////////////////////////////////////////////////
sapi_error_t temp_read_sensor(char *payload, uint8_t *len)
{
	float reading = 0.0;
	sapi_error_t rc;
	char buffer[128];

	// Read temp sensor, already in network order
	rc = read_dht11(&reading);
	if (rc != SAPI_ERR_OK)
	{
		return rc;
	}
	//sendInterval1;
	//sampleRate1;
	// Assemble the Payload
	rc = temp_build_payload(buffer, &reading);
	if (rc != SAPI_ERR_OK)
	{
		return rc;
	}
	strcpy(payload, buffer);
	*len = strlen(buffer);
	return rc;
}


//////////////////////////////////////////////////////////////////////////
//
// Reads a DHT11 sensor. Read sensor configuration. Builds and returns the payload. Callback called on
//   CoAP Get configuration value
//
//////////////////////////////////////////////////////////////////////////
sapi_error_t temp_read_cfg(char *payload, uint8_t *len)
{
	// Assemble the Payload
	// Trick - if sensor value is NULL than the payload builder just returns the UOM.
	sapi_error_t rc = temp_build_payload(payload, NULL);
	strcpy(payload, payload);
	*len = strlen(payload);

    return rc;
}


//////////////////////////////////////////////////////////////////////////
//
// Write sensor configuration. Processes payload sent from client. Callback called on
//  CoAP Put configuration value
//
//////////////////////////////////////////////////////////////////////////
sapi_error_t temp_write_cfg(char *payload, uint8_t *len)
{
	if (!strcmp(payload, "cfg=C"))
	{
		context.scalecfg = CELSIUS_SCALE;
	}
	else if (!strcmp(payload, "cfg=F"))
	{
		context.scalecfg = FAHRENHEIT_SCALE;
	}
	// Config not supported
	else
	{
		return SAPI_ERR_NOT_IMPLEMENTED;
	}

	return SAPI_ERR_OK;
}

//////////////////////////////////////////////////////////////////////////
//
// MOTOROLA ACE 3380
// Created 10/14/2020
// By Ryan Trisnojoyo
// PINS LAYOUT
// D2 = RX						RO
// D3 = TX						DI
// D4 = DIGITAL OUTPUT			RE
// D5 = DIGITAL OUTPUT			DE
// Weather Station sensor : 
// RS485_TX = B		--->		A RS485 Weather Station
// RS485_RX = A		--->		B RS485 Weatheer Station
// GND				--->		GND Ultrasnonic Sensor
//////////////////////////////////////////////////////////////////////////
float resultTemp = 0;
float resultUltra = 0;

byte temp_data[40];
byte sendRequest10Data[8]={0x0A,0x03,0x08,0x01,0x00,0x0A,0x97,0x16};				//Send Request for Manufacturer ID
int w = 0;

size_t bytes;

float Send(byte * cmd, byte* ret) {
	// use default send function
	//turn on relay
	digitalWrite(D6, HIGH);
	digitalWrite(D7, LOW);
	delay(5000);
	sendCommand(cmd);
	
	int h = 0;
	// receive answer
	if (Serial3.available()){  //Read return data package (NOTE: Demo is just for your reference, the data package haven't be calibrated yet)
	while(Serial3.read() != 0x0A && h < 1000){
		h = h + 1;
	}
	h = 0;
	ret[0] = 0x0A;
	for(int j=2; j < 26; j++){
		ret[j++]=(Serial3.read());
	}
	
    Serial.println("Data Begin");
    Serial.println(ret[0],HEX); //byte 1    //Slave ID
    Serial.println(ret[2],HEX); //byte 2    //Function Code
    Serial.println(ret[4],HEX); //byte 3    //How many bytes send
    Serial.println(ret[6],HEX); //byte 4    //Hex First Register
    Serial.println(ret[8],HEX); //byte 5    //Hex Second Register
    Serial.println(ret[10],HEX); //byte 6   //3rd Register
	Serial.println(ret[12],HEX); //byte 7   //4th Register
	Serial.println(ret[14],HEX); //byte 8   //5th Register
	Serial.println(ret[16],HEX); //byte 9   //6th Register
	Serial.println(ret[18],HEX); //byte 10   //7th Register
	Serial.println(ret[20],HEX); //byte 11   //8th Register
	Serial.println(ret[22],HEX); //byte 12   //9th Register
	Serial.println(ret[24],HEX); //bytes 13 //10th Register
	
    Serial.println("Data End");

    Serial3.flush();
    Serial.println("Received data Done");
	
	resultTemp = (float(ret[6])*256)+ float(ret[8]);
	Serial.print("Temperature: ");
	Serial.print(resultTemp);
	Serial.println(" C");
	
	/*
    resultTemp = (float(ret[8])*0.48876)-50;
    Serial.print("Temperature: ");
    Serial.print(resultTemp);
    Serial.println(" C");
	
    resultUltra = (float(ret[6]*256)+float(ret[4]))/128;
    Serial.print("Level: ");
    Serial.print(resultUltra);
    Serial.println(" In");
	*/
  }
  else{
    Serial.println("Error reading RS485");
  }
    delay(1000);
	//turn off relay
	digitalWrite(D6, LOW);
	digitalWrite(D7, HIGH);
	
	return resultTemp;
}

/*sendCommand(...)******************************************************************************
 * General function that sends command to RS485 peripheral device
 * <CR> is added to all commands
 * For RS485 communication, RTS pin has to be HIGH to allow writing to peripheral
 **********************************************************************************************/
void sendCommand(byte *cmd) {

//check if transmit the correct data
/*
 Serial.println("Send Command");
 for(int i=0; i < 6; i++){
    Serial.print(cmd[i]);
    Serial.println("");
 }
 */
	// set RTS to HIGH to allow writing to MAX485
 	digitalWrite(D4, HIGH);
 	digitalWrite(D5, HIGH); 
	delay(100);

	for(int i=0; i < 8; i++){
		Serial3.write(cmd[i]); 
  //Serial.print(cmd[i]);
	}	
 // send command
	Serial3.flush(); // Make sure message is fully transferred
	// set RTS to LOW to receive answer
	digitalWrite(D4, LOW);
	digitalWrite(D5, LOW);
 
	delay(50);
}
 
  //Float Code
  int temp_float = 0;
  int calculate_Float(){
	  pinMode(D10, INPUT_PULLUP);
	  temp_float = digitalRead(D10);
	  
	  if (temp_float == 1){
		  temp_float = 0;
	  }
	  else if (temp_float == 0){
		  temp_float = 1;
	  }
	  
	  return temp_float;
  }
//////////////////////////////////////////////////////////////////////////
//
// Code to build the sensor payload. Temp payload is text with this format:
//   <epoch>,<temp>,<uom>
//     <epoch> is a 4 byte (8 hex char) timestamp
//     <temp> is a decimal number
//     <uom> is a char: C or F
//
//  Note that the payload is text. Payloads can also be a byte array of binary data.
//
//////////////////////////////////////////////////////////////////////////
sapi_error_t temp_build_payload(char *buf, float *reading)
{
	int sendInterval2 = ParamSendInterval();
	int sampleRate2 =  ParamSampleRate();
	char 		payload[128]; // REMEMBER!! maximum payload 118 character payload character
	char		temp_payload[128];
	char		reading_buf[32];
	char        datatype_Ultra[] = "3,"; //datatype for LEVEL is 3
	char		datatype_Float[] = "7,"; //datatype for DI state is 7
	char		unitUltra[] = "In";
	char		unitFloat[] = "Al";
	char		unit_buf[4];
	float		ultra_temp = 10.00;
	time_t     	epoch;
	uint32_t	indx;
	char    rultra1[] = "12.00,";
	char    rfloat[] = "2,"; //if it shows 2, float not connected properly. The value should be 0 or 1.
	char    temp_epoch[20];
	
	strcpy(temp_payload, "");
	strcpy(payload, "");

	Serial3.flush();
	
	ultra_temp = Send(sendRequest10Data, temp_data);
	sprintf(rultra1, "%.2f,", ultra_temp);
	sprintf(rfloat, "%d,", calculate_Float());

	// Create string containing the UNIX epoch
	epoch = get_rtc_epoch();
	sprintf(temp_epoch, "%d,", epoch);
	
	
	//construct ultrasonic data payload
	sprintf(temp_payload, "%d,", epoch);
	strcat(temp_payload, datatype_Ultra);
	strcat(temp_payload, rultra1);
	strcpy(unit_buf, unitUltra);
	strcat(temp_payload, unit_buf);
	strcat(temp_payload, ";");

	//construct float data payload
	strcat(temp_payload,temp_epoch);
	strcat(temp_payload, datatype_Float);
	strcat(temp_payload, rfloat);
	strcpy(unit_buf, unitFloat);
	strcat(temp_payload, unit_buf);
	strcat(temp_payload,";");
	strcpy(payload, temp_payload);
	/*
	dlog(LOG_DEBUG, "Temp Payload: %s", payload);
	return SAPI_ERR_OK;
	*/

	if ((counter1 >= (sendInterval2 / sampleRate2 - 1)) || (temp_float == 1) ){
		strcat(payloadFinal, payload);
		strcpy(buf, payloadFinal); //copy to final buf and ready to be sent
		counter1= 0;
		dlog(LOG_DEBUG, "Temp Payload Final: %s", payloadFinal);
		//empty the final payload
		strcpy(payloadFinal, "");
		return SAPI_ERR_OK;
	}
	else {
		counter1 = counter1 + 1; //+1 counter
		strcat(payloadFinal, payload);
		//dlog(LOG_DEBUG, "Temp Payload: %s", payload);
		return SAPI_ERR_OK;
	}
	
}



//////////////////////////////////////////////////////////////////////////
//
// Sensor (hardware) access functions.
//
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//
// Read the temp value from the DHT11 sensor.
//
//////////////////////////////////////////////////////////////////////////
sapi_error_t read_dht11(float *reading)
{
	sapi_error_t rc = SAPI_ERR_OK;
	float re = INVALID_TEMP;

	// Get temperature event
	sensors_event_t event;
	dht.temperature().getEvent(&event);

	// Check for NaN
	if (isnan(event.temperature))
	{
		re = NO_SENSOR_TEMP;
		rc = SAPI_ERR_OK;
	}
	else
	{
		re = event.temperature;
		rc = SAPI_ERR_OK;
	}

	// Reading is in C. Convert to F if needed.
	if (context.scalecfg == FAHRENHEIT_SCALE)
	{
		// Convert from Celsius to Fahrenheit
		re *= 1.8;
		re += 32;
	}

	// Assign output
	*reading = re;
	return rc;
}


//////////////////////////////////////////////////////////////////////////
//
// Context and Alert functions. Support for sensor enable and disable. Support for alerts.
//
//////////////////////////////////////////////////////////////////////////

sapi_error_t temp_sensor_enable(void)
{
	context.enable = 1;
	context.alertstate = tsat_cleared;
	return SAPI_ERR_OK;
}


sapi_error_t temp_sensor_disable(void)
{
	context.enable = 0;
	context.alertstate = tsat_disabled;
	return SAPI_ERR_OK;
}
