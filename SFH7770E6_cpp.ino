/*
* SFH7770E6.cpp
*
*  Created on: Jan 19, 2014
*      Author: mlw
*/

#include "SFH7770E6.h"
#include <Wire.h>

#if 1
#define CLASS_PRE
SFH7770E6Data   _ProxData;
#else
#define CLASS_PRE SFH_7770_E6::
#endif

/***************************************************************************
PRIVATE FUNCTIONS
***************************************************************************/

/**************************************************************************/
/*!
@brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
void CLASS_PRE write8(byte address, byte reg, byte value)
{
	Wire.beginTransmission(address);
	#if ARDUINO >= 100
	Wire.write((uint8_t)reg);
	Wire.write((uint8_t)value);
	#else
	Wire.send(reg);
	Wire.send(value);
	#endif
	Wire.endTransmission();
}

/**************************************************************************
*
*   @brief  Abstract away platform differences in Arduino wire library
*
**************************************************************************/
byte CLASS_PRE read8(byte address, byte reg)
{
	byte value;

	Wire.beginTransmission(address);
	#if ARDUINO >= 100
	Wire.write((uint8_t)reg);
	#else
	Wire.send(reg);
	#endif
	Wire.endTransmission();
	Wire.requestFrom(address, (byte)1);
	#if ARDUINO >= 100
	value = Wire.read();
	#else
	value = Wire.receive();
	#endif
	Wire.endTransmission();

	return value;
}

int CLASS_PRE readCnt(byte address, byte reg, int count, byte *data)
{
	int len = 0;

	Wire.beginTransmission(address);
	#if ARDUINO >= 100
	Wire.write((uint8_t)reg);
	#else
	Wire.send(reg);
	#endif
	Wire.endTransmission();
	Wire.requestFrom(address, (byte) count);
	
	while( Wire.available())
	{
		#if ARDUINO >= 100
		data[len] = Wire.read();
		#else
		data[len] = Wire.receive();
		#endif
		len++;
	}
	Wire.endTransmission();

	return len;
}

/***************************************************************************
CONSTRUCTOR
***************************************************************************/

/***************************************************************************
PUBLIC FUNCTIONS
***************************************************************************/

/**************************************************************************/
/*!
@brief  Setups the HW
*/
/**************************************************************************/
bool CLASS_PRE begin()
{
	int x;
	
	// Enable I2C
	Wire.begin();

	Serial.print("\e[12;1H");
	write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_INTEGRATION_TIME_ACCESS, 1);
	for( x = 0x20 ; x < 0x30 ; x++ )
	{
		byte value;
		
		value = read8(SFH7770E6_ADDRESS, x);
		
		Serial.print("reg 0x");
		Serial.print(x, HEX);
		Serial.print(" value = ");
		Serial.println(value, HEX);
	}

	// Enable the proximity detector in free running mode.
	setPsMode(SFH7770E6_MODE_FREE_RUNNING);
	ledSelect(SFH7770E6_LED_SELECT_LED_ALL);
	
	return true;
}

bool CLASS_PRE begin(SFH7770E6LedCurrent current)
{
	// Enable I2C
	Wire.begin();

	// Enable the proximity detector in free running mode.
	setPsMode(SFH7770E6_MODE_FREE_RUNNING);
	ledSelect(SFH7770E6_LED_SELECT_LED_ALL);

	/* set the led current to the value passed */

	setLedCurrent(1, current);
	setLedCurrent(2, current);
	setLedCurrent(3, current);
	
	return true;
}

bool CLASS_PRE begin(SFH7770E6LedCurrent current, SFH7770E6PsIntegrationTime time)
{
	// Enable I2C
	Wire.begin();

	/* set the led current to the value passed */

	setLedCurrent(1, current);
	setLedCurrent(2, current);
	setLedCurrent(3, current);

	/* set the proximity integration time passed */

	setPsIntegrationTime(time);

	// Enable the proximity detector in free running mode.
	setPsMode(SFH7770E6_MODE_FREE_RUNNING);
	ledSelect(SFH7770E6_LED_SELECT_LED_ALL);

	return true;
}


/**************************************************************************/
/*!
@brief  Gets the most recent proximity data
*/
/**************************************************************************/
void CLASS_PRE getData(SFH7770E6Data *proxData) {
	SFH7770E6DataStatus dataStatus;
	SFH7770E6Mode mode;
	int cnt = 0;

	/* Clear the data */
	memset(proxData, 0, sizeof(SFH7770E6Data));

	// if we are free running check to see what data is available and update the
	// global copies
	
	mode = (SFH7770E6Mode) read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_PS_MODE);
	
	if( mode == SFH7770E6_MODE_TRIGGERED )
	{
		// trigger a read of proximity
		
		write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_MCU_TRIGGER, SFH7770E6_TRIGGER_PS);
	}
	
	// look to see if there is any proximity data available
	// we are going to read all the data bytes at one time
	
	if( (cnt = readCnt(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_ALS_DATA_LSB, sizeof(SFH7770E6Data), (byte *) &_ProxData)) != sizeof(SFH7770E6Data))
	{
		// complain for now
		Serial.print("\e[10;1H");
		Serial.print("Only read ");
		Serial.print(cnt);
		Serial.print(" of ");
		Serial.println(sizeof(SFH7770E6Data));
		return;
	}
	
	dataStatus = (SFH7770E6DataStatus) _ProxData.status;

	/* what registers have data to read */

	if( dataStatus & (SFH7770E6_PS_LED_1_DATA | SFH7770E6_PS_LED_2_DATA | SFH7770E6_PS_LED_3_DATA) )
	{
		// there is new data somewhere

		if( dataStatus & SFH7770E6_PS_LED_1_DATA )
		{
			proxData->led1 = _ProxData.led1;
		}

		if( dataStatus & SFH7770E6_PS_LED_2_DATA )
		{
			proxData->led2 = _ProxData.led2;
		}

		if( dataStatus & SFH7770E6_PS_LED_3_DATA )
		{
			proxData->led3 = _ProxData.led3;
		}
	}
	
	proxData->status = _ProxData.status;
	proxData->intControl = _ProxData.intControl;
}

void CLASS_PRE ledSelect(SFH7770E6LedSelect leds)
{
	switch(leds)
	{
		case SFH7770E6_LED_SELECT_LED_1:
		case SFH7770E6_LED_SELECT_LED_2:
		case SFH7770E6_LED_SELECT_LED_3:
		case SFH7770E6_LED_SELECT_LED_ALL:
		write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_LED_1_2_CURRENT, leds);
		break;
		default:
		break;
	}
}

/**************************************************************************/
/*!
@brief  Set the LED current
*/
/**************************************************************************/
void setLedCurrent(byte ledNum, SFH7770E6LedCurrent current)
{
	byte value;
	
	if( (ledNum == 1) || (ledNum == 2) )
	{
		value = read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_LED_1_2_CURRENT);
		if( ledNum == 1)
		{
			value = (value & 0xf8) + current;
		}
		else
		{
			value = (value & 0xc7) + (current << 3);
		}

		write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_LED_1_2_CURRENT, value);
	}
	else if( ledNum == 3 )
	{
		value = read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_LED_3_CURRENT);
		value = (value & 0xf8) + current;
		write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_LED_3_CURRENT, value);
	}
}

/**************************************************************************/
/*!
@brief  Get the LED current
*/
/**************************************************************************/
byte getLedCurrent(byte ledNum)
{
	byte value;
	
	if( (ledNum == 1) || (ledNum == 2))
	{
		value = read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_LED_1_2_CURRENT);
		
		if( ledNum == 2 )
		{
			value >>= 3;
		}
	}
	else
	{
		value = read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_LED_3_CURRENT);
	}
	
	return(value & 0x07 );
}

/**************************************************************************/
/*!
@brief  Set the proximity integration time
*/
/**************************************************************************/
void setPsIntegrationTime(SFH7770E6PsIntegrationTime time)
{
	byte value;
	
	/* enable the integration time access */

	while(	read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_INTEGRATION_TIME_ACCESS) == 0)
	{
		Serial.print("\e[6;1H int access == 0");
		write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_INTEGRATION_TIME_ACCESS, 0x1);
	}
	
	Serial.print("\e[8;1H int access == 1");
	/* set the proximity sensor integration time */
	value = read8(SFH7770E6_ADDRESS, 0x27);
	Serial.print("\e[7;1H int time == ");
	Serial.println(value);
	
	write8(SFH7770E6_ADDRESS, 0x27, 5);

	/* disable the integration time access */

    write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_INTEGRATION_TIME_ACCESS, 0);
}

/**************************************************************************/
/*!
@brief  Get the proximity integration time
*/
/**************************************************************************/
byte getPsIntegrationTime(void)
{
	byte value;
	
	write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_INTEGRATION_TIME_ACCESS, 0x1);
	value = read8(SFH7770E6_ADDRESS, 0x27);
	write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_INTEGRATION_TIME_ACCESS, 0x0);
	return value;
}

/**************************************************************************/
/*!
@brief  Gets the part number and revision
*/
/**************************************************************************/
byte CLASS_PRE getPartRev(void) {
	return read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_PART_REV);
}

/**************************************************************************/
/*!
@brief  Gets the manufacture number
*/
/**************************************************************************/
byte CLASS_PRE getMfgr(void) {
	return read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_MFGR);
}

/**************************************************************************/
/*!
@brief  Set the proximity mode
*/
/**************************************************************************/
void setPsMode(SFH7770E6Mode mode)
{
	write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_PS_MODE, mode);
}

/**************************************************************************/
/*!
@brief  Get the proximity mode
*/
/**************************************************************************/
SFH7770E6Mode getPsMode(void)
{
	return (SFH7770E6Mode) read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_PS_MODE);
}

/**************************************************************************/
/*!
@brief  Set the ambient light sensor mode
*/
/**************************************************************************/
void setAlsMode(SFH7770E6Mode mode)
{
	write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_ALS_MODE, mode);
}

/**************************************************************************/
/*!
@brief  Get the ambient light sensor mode
*/
/**************************************************************************/
SFH7770E6Mode getAlsMode(void)
{
	return (SFH7770E6Mode) read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_ALS_MODE);
}

/**************************************************************************/
/*!
@brief  Set the LED PS threshold
*/
/**************************************************************************/
void CLASS_PRE setPsThreshold(byte ledNum, byte threshold)
{
	
	if( ledNum & LED_1)
	{
		write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_PS_LED_1_THRES, threshold);
	}

	if( ledNum & LED_2 )
	{
		write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_PS_LED_2_THRES, threshold);
	}

	if( ledNum & LED_3 )
	{
		write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_PS_LED_3_THRES, threshold);
	}
}

/**************************************************************************/
/*!
@brief  Get the LED PS threshold
*/
/**************************************************************************/
byte CLASS_PRE getPsThreshold(byte ledNum)
{
	
	if( ledNum & LED_1)
	{
		return read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_PS_LED_1_THRES);
	}
	else if( ledNum & LED_2 )
	{
		return read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_PS_LED_2_THRES);
	}
	else if( ledNum & LED_3 )
	{
		return read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_PS_LED_3_THRES);
	}
	
	return(0);
}

/**************************************************************************/
/*!
@brief  Set the ALS Upper threshold
*/
/**************************************************************************/
void CLASS_PRE setAlsUpperThreshold(byte ledNum, word threshold)
{
	write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_ALS_UPPER_THRES_LSB, threshold & 0xff);
	write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_ALS_UPPER_THRES_MSB, (threshold >> 8) & 0xff);
}

/**************************************************************************/
/*!
@brief  Get the ALS Upper threshold
*/
/**************************************************************************/
word CLASS_PRE getAlsUpperThreshold(byte ledNum)
{
	word value;
	
	value = (word) read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_ALS_UPPER_THRES_LSB);
	value |= ((word) read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_ALS_UPPER_THRES_LSB)) << 8;
	return value;
}

/**************************************************************************/
/*!
@brief  Set the ALS Lower threshold
*/
/**************************************************************************/
void CLASS_PRE setAlsLowerThreshold(byte ledNum, word threshold)
{
	write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_ALS_LOWER_THRES_LSB, threshold & 0xff);
	write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_ALS_LOWER_THRES_MSB, (threshold >> 8) & 0xff);
}

/**************************************************************************/
/*!
@brief  Get the ALS Lower threshold
*/
/**************************************************************************/
word CLASS_PRE getAlsLowerThreshold(byte ledNum)
{
	word value;
	
	value = (word) read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_ALS_LOWER_THRES_LSB);
	value |= ((word) read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_ALS_LOWER_THRES_LSB)) << 8;
	return value;
}


void printLeadingZero(byte val)
{
	if( val < 10 )
	{
		Serial.print("00");
	}
	else if( val < 100 )
	{
		Serial.print("0");
	}
	
	Serial.print(val);
}

void setup()
{
	int x;
	
	Serial.begin(115200);
	Serial.print("\e[2J\e[1;1H");
	begin();
	/* set the led current to the value passed */

	setLedCurrent(1, SFH7770E6_LED_CURRENT_200MA);
	setLedCurrent(2, SFH7770E6_LED_CURRENT_200MA);
	setLedCurrent(3, SFH7770E6_LED_CURRENT_200MA);
	
	Serial.print("\e[1;1H");
	Serial.print("The part # and rev ");
	Serial.println(getPartRev());
	Serial.print("The MFGR ");
	Serial.println(getMfgr());
//	setPsIntegrationTime(SFH7770E6_PS_INT_2500US);

}

void loop()
{
	SFH7770E6Data proxData;
	byte value;
	int i;
	
	getData(&proxData);

	Serial.print("\e[3;1H");
	Serial.print("Led1[");
	printLeadingZero(proxData.led1);
	Serial.print("] Led2[");
	printLeadingZero(proxData.led2);
	Serial.print("] Led3[");
	printLeadingZero(proxData.led3);
	Serial.println("]     ");
	
	
	Serial.print("\e[4;1H");

	for( i = 1 ; i <= 3 ; i++)
	{
		Serial.print("Led");
		Serial.print(i);
		Serial.print("Cur[");
		value = getLedCurrent(i);
		Serial.print(value);
		Serial.print("] ");
	}
	Serial.println("     ");
	
	Serial.print("\e[5;1H");
	Serial.print("Ps integration time ");
	Serial.println(getPsIntegrationTime());
	
	delay(250);
}


