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

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
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
  // Enable I2C
  Wire.begin();

  // Enable the proximity detector in free running mode.
  setPsMode(SFH7770E6_MODE_TRIGGERED);
  ledSelect(SFH7770E6_LED_SELECT_LED_ALL);
  // set 200ma current
   setLedCurrent(LED_1, SFH7770E6_LED_CURRENT_200MA);
   setLedCurrent(LED_2, SFH7770E6_LED_CURRENT_200MA);
   setLedCurrent(LED_3, SFH7770E6_LED_CURRENT_200MA);
  
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
  
  dataStatus = (SFH7770E6DataStatus) read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_DATA_STATUS);

  /* what registers have data to read */

  if( dataStatus & (SFH7770E6_PS_LED_1_DATA | SFH7770E6_PS_LED_2_DATA | SFH7770E6_PS_LED_3_DATA) )
  {
	  // there is new data somewhere

	  if( dataStatus & SFH7770E6_PS_LED_1_DATA )
	  {
		  _ProxData.led1 = read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_PS_LED_1_DATA);
	  }

	  if( dataStatus & SFH7770E6_PS_LED_2_DATA )
	  {
		  _ProxData.led2 = read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_PS_LED_2_DATA);
	  }

	  if( dataStatus & SFH7770E6_PS_LED_3_DATA )
	  {
		  _ProxData.led3 = read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_PS_LED_3_DATA);
	  }
  }

  // return the values we keep in memory
  *proxData = _ProxData;
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

void CLASS_PRE setLedCurrent(byte led, SFH7770E6LedCurrent current)
{
    byte value;
    
    switch(led)
    {
      case LED_1:
        value = read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_LED_1_2_CURRENT);
        value &= 0xf8;
        value |= current & 0x7;
        write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_LED_1_2_CURRENT, value);      
        break;
      case LED_2:
        value = read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_LED_1_2_CURRENT);
        value &= 0xc7;
        value |= (current & 0x7) << 3;
        write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_LED_1_2_CURRENT, value);      
        break;
      case LED_3:
        write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_LED_3_CURRENT, current & 0x7);      
        break;
      default:
        break;
    }   
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
void CLASS_PRE setPsThreshold(byte ledNum, byte threshold) {
  
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
byte CLASS_PRE getPsThreshold(byte ledNum) {
  
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
void CLASS_PRE setAlsUpperThreshold(byte ledNum, word threshold) {

    write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_ALS_UPPER_THRES_LSB, threshold & 0xff);
    write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_ALS_UPPER_THRES_MSB, (threshold >> 8) & 0xff);
}

/**************************************************************************/
/*!
    @brief  Get the ALS Upper threshold
*/
/**************************************************************************/
word CLASS_PRE getAlsUpperThreshold(byte ledNum) {
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
void CLASS_PRE setAlsLowerThreshold(byte ledNum, word threshold) {

    write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_ALS_LOWER_THRES_LSB, threshold & 0xff);
    write8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_ALS_LOWER_THRES_MSB, (threshold >> 8) & 0xff);
}

/**************************************************************************/
/*!
    @brief  Get the ALS Lower threshold
*/
/**************************************************************************/
word CLASS_PRE getAlsLowerThreshold(byte ledNum) {
    word value;
    
    value = (word) read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_ALS_LOWER_THRES_LSB);
    value |= ((word) read8(SFH7770E6_ADDRESS, SFH7770E6_REGISTER_ALS_LOWER_THRES_LSB)) << 8;
    return value;
}

void setup()
{
  Serial.begin(57600);
  begin();
  Serial.print("The part # and rev ");
  Serial.println(getPartRev());
  Serial.print("The MFGR ");
  Serial.println(getMfgr());
}

void loop()
{
  SFH7770E6Data proxData;
  
  getData(&proxData);
  
  Serial.print("Led1[");
  Serial.print(proxData.led1);
  Serial.print("] Led2[");
  Serial.print(proxData.led2);
  Serial.print("] Led3[");
  Serial.print(proxData.led3);
  Serial.println("]");

  delay(100);
}
  
  
