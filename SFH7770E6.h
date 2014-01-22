/*
 * SFH7770E6.h
 *
 *  Created on: Jan 19, 2014
 *      Author: mlw
 */
#ifndef SFH7770E6_H_
#define SFH7770E6_H_

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif



//#define byte unsigned char
//#define word unsigned short

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define SFH7770E6_ADDRESS          (0x38)         // 0011100x
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    typedef enum
    {                                                     // DEFAULT    TYPE
      SFH7770E6_REGISTER_INTEGRATION_TIME_ACCESS   = 0x20,   // 00000111   rw
      SFH7770E6_REGISTER_ALS_INTEGRATION_TIME      = 0x26,   // 00000000   r
      SFH7770E6_REGISTER_PS_INTEGRATION_TIME       = 0x27,   // 00000000   r
      SFH7770E6_REGISTER_RESET                     = 0x80,
      SFH7770E6_REGISTER_ALS_MODE                  = 0x80,   // same as previous
      SFH7770E6_REGISTER_PS_MODE                   = 0x81,
      SFH7770E6_REGISTER_LED_1_2_CURRENT           = 0x82,
      SFH7770E6_REGISTER_LED_SELECT                = 0x82,   // same as previous
      SFH7770E6_REGISTER_LED_3_CURRENT             = 0x83,
      SFH7770E6_REGISTER_MCU_TRIGGER               = 0x84,
      SFH7770E6_REGISTER_PS_TIME_INTERVAL          = 0x85,
      SFH7770E6_REGISTER_ALS_TIME_INTERVAL         = 0x86,
      SFH7770E6_REGISTER_PART_REV                  = 0x8A,
      SFH7770E6_REGISTER_MFGR                      = 0x8B,
      SFH7770E6_REGISTER_ALS_DATA_LSB              = 0x8C,
      SFH7770E6_REGISTER_ALS_DATA_MSB              = 0x8D,
      SFH7770E6_REGISTER_DATA_STATUS               = 0x8E,
      SFH7770E6_REGISTER_PS_LED_1_DATA             = 0x8F,
      SFH7770E6_REGISTER_PS_LED_2_DATA             = 0x90,
      SFH7770E6_REGISTER_PS_LED_3_DATA             = 0x91,
      SFH7770E6_REGISTER_INT_CONTROL_STATUS        = 0x92,
      SFH7770E6_REGISTER_PS_LED_1_THRES            = 0x93,
      SFH7770E6_REGISTER_PS_LED_2_THRES            = 0x94,
      SFH7770E6_REGISTER_PS_LED_3_THRES            = 0x95,
      SFH7770E6_REGISTER_ALS_UPPER_THRES_LSB       = 0x96,
      SFH7770E6_REGISTER_ALS_UPPER_THRES_MSB       = 0x97,
      SFH7770E6_REGISTER_ALS_LOWER_THRES_LSB       = 0x98,
      SFH7770E6_REGISTER_ALS_LOWER_THRES_MSB       = 0x99
    } SFH7770E6Registers_t;


/*=========================================================================*/

/*=========================================================================
	MODE SETTING
	-----------------------------------------------------------------------*/
	typedef enum
	{
	  SFH7770E6_MODE_STANDBY                       = 0x00,
	  SFH7770E6_MODE_TRIGGERED                     = 0x02,
	  SFH7770E6_MODE_FREE_RUNNING                  = 0x03
	} SFH7770E6Mode;

/*=========================================================================*/

/*=========================================================================
	RESET SETTING
	-----------------------------------------------------------------------*/
	typedef enum
	{
	  SFH7770E6_STATE_RUNNING                      = 0x00,
	  SFH7770E6_STATE_RESET                        = 0x04
	} SFH7770E6State;

/*=========================================================================*/

/*=========================================================================
	LED SELECT
	-----------------------------------------------------------------------*/
	typedef enum
	{
	  SFH7770E6_LED_SELECT_LED_1                   = 0x00,
	  SFH7770E6_LED_SELECT_LED_2                   = 0x40,
	  SFH7770E6_LED_SELECT_LED_3                   = 0x80,
	  SFH7770E6_LED_SELECT_LED_ALL                 = 0xC0
	} SFH7770E6LedSelect;

/*=========================================================================*/

/*=========================================================================
	LED CURRENT, for led2 Shift up by 3 bits
	-----------------------------------------------------------------------*/
	typedef enum
	{
	  SFH7770E6_LED_CURRENT_5MA                    = 0x00,
	  SFH7770E6_LED_CURRENT_10MA                   = 0x01,
	  SFH7770E6_LED_CURRENT_20MA                   = 0x02,
	  SFH7770E6_LED_CURRENT_50MA                   = 0x03,
	  SFH7770E6_LED_CURRENT_100MA                  = 0x04,
	  SFH7770E6_LED_CURRENT_150MA                  = 0x05,
	  SFH7770E6_LED_CURRENT_200MA                  = 0x06,
	} SFH7770E6LedCurrent;

/*=========================================================================*/

/*=========================================================================
	TRIGGER FROM MCU
	-----------------------------------------------------------------------*/
	typedef enum
	{
	  SFH7770E6_TRIGGER_PS                         = 0x01,
	  SFH7770E6_TRIGGER_ALS                        = 0x02
	} SFH7770E6Trigger;

/*=========================================================================*/

/*=========================================================================
	PROXIMITY TIME INTERVAL
	-----------------------------------------------------------------------*/
	typedef enum
	{
	  SFH7770E6_PS_TIME_10MS                       = 0x00,
	  SFH7770E6_PS_TIME_20MS                       = 0x01,
	  SFH7770E6_PS_TIME_30MS                       = 0x02,
	  SFH7770E6_PS_TIME_50MS                       = 0x03,
	  SFH7770E6_PS_TIME_70MS                       = 0x04,
	  SFH7770E6_PS_TIME_100MS                      = 0x05,
	  SFH7770E6_PS_TIME_200MS                      = 0x06,
	  SFH7770E6_PS_TIME_500MS                      = 0x07,
	  SFH7770E6_PS_TIME_1000MS                     = 0x08,
	  SFH7770E6_PS_TIME_2000MS                     = 0x09
	} SFH7770E6PsTimeInterval;

/*=========================================================================*/

/*=========================================================================
	AMBIENT LIGHT SENSE TIME INTERVAL
	-----------------------------------------------------------------------*/
	typedef enum
	{
	  SFH7770E6_ALS_TIME_100MS                      = 0x00,
	  SFH7770E6_ALS_TIME_200MS                      = 0x01,
	  SFH7770E6_ALS_TIME_500MS                      = 0x02,
	  SFH7770E6_ALS_TIME_1000MS                     = 0x03,
	  SFH7770E6_ALS_TIME_2000MS                     = 0x04
	} SFH7770E6AlsTimeInterval;

/*=========================================================================*/

/*=========================================================================
	DATA STATUS
	-----------------------------------------------------------------------*/
	typedef enum
	{
	  SFH7770E6_PS_LED_1_DATA                       = 0x01,
	  SFH7770E6_PS_LED_1_THRES                      = 0x02,
	  SFH7770E6_PS_LED_2_DATA                       = 0x04,
	  SFH7770E6_PS_LED_2_THRES                      = 0x08,
	  SFH7770E6_PS_LED_3_DATA                       = 0x10,
	  SFH7770E6_PS_LED_3_THRES                      = 0x20,
	  SFH7770E6_ALS_DATA                            = 0x40,
	  SFH7770E6_ALS_THRES                           = 0x80
	} SFH7770E6DataStatus;

/*=========================================================================*/

/*=========================================================================
	INTERRUPT STATUS AND CONTROL
	-----------------------------------------------------------------------*/
	typedef enum
	{
	  SFH7770E6_INT_MODE_HI_Z                       = 0x00,
	  SFH7770E6_INT_MODE_PS                         = 0x01,
	  SFH7770E6_INT_MODE_ALS                        = 0x02,
	  SFH7770E6_INT_MODE_PS_ALS                     = 0x03,
	  SFH7770E6_INT_LEVEL_LOW                       = 0x00,
	  SFH7770E6_INT_LEVEL_HI                        = 0x04,
	  SFH7770E6_INT_MODE_LATCHED                    = 0x00,
	  SFH7770E6_INT_MODE_NOT_LATCHED                = 0x08,
	  SFH7770E6_INT_FROM_ALS                        = 0x00,
	  SFH7770E6_INT_FROM_LED_1                      = 0x20,
	  SFH7770E6_INT_FROM_LED_2                      = 0x40,
	  SFH7770E6_INT_FROM_LED_3                      = 0x60,
	  SFH7770E6_INT_MASK                            = 0x60,
	} SFH7770E6IntStatusControl;

/*=========================================================================*/

/*=========================================================================
    INTERNAL PROXIMTIY DATA TYPE
    -----------------------------------------------------------------------*/
    typedef struct SFH7770E6Data_s
    {
        byte led1;
        byte led2;
        byte led3;
        word als;
    } SFH7770E6Data;
/*=========================================================================*/

/*=========================================================================
    CHIP ID
    -----------------------------------------------------------------------*/
    #define SFH7770E6_ID                     (0b10010111)
/*=========================================================================*/

/*=========================================================================
    LEDS
    -----------------------------------------------------------------------*/
    #define LED_1                            (1 << 0)
    #define LED_2                            (1 << 1)
    #define LED_3                            (1 << 2)
/*=========================================================================*/


#if 0
/* Unified sensor driver for the accelerometer */
class SFH_7770_E6
{
  public:
    bool begin(void);
    void getData(SFH7770E6Data *);
    void ledSelect(SFH7770E6LedSelect);
    byte getPartRev(void);
    byte getMfgr(void);
    void setPsMode(SFH7770E6Mode);
    SFH7770E6Mode getPsMode(void);
    void setAlsMode(SFH7770E6Mode);
    SFH7770E6Mode getAlsMode(void);
    void setPsThreshold(byte, byte);
    byte getPsThreshold(byte);
    void setAlsThreshold(byte, word);
    word getAlsThreshold(byte);

  private:
    SFH7770E6Data   _ProxData;   // Last read proximity and ambient values

    void write8(byte address, byte reg, byte value);
    byte read8(byte address, byte reg);
};
#endif
#endif /* SFH7770E6_H_ */
