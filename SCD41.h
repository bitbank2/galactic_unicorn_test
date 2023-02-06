//
// SCD41 CO2 sensor library
//
// Written by Larry Bank - 5/16/2022
// Copyright (c) 2022 BitBank Software, Inc.
// bitbank@pobox.com
//
//
#ifndef __SCD41__
#define __SCD41__

#include "BitBang_I2C.h"

#define SCD41_SUCCESS 0
#define SCD41_ERROR -1
#define SCD41_INVALID_PARAM -2

// 16-bit I2C commands
#define SCD41_CMD_START_PERIODIC_MEASUREMENT              0x21b1
#define SCD41_CMD_START_LP_PERIODIC_MEASUREMENT           0x21ac
#define SCD41_CMD_SINGLE_SHOT_MEASUREMENT                 0x219d
#define SCD41_CMD_READ_MEASUREMENT                        0xec05 // execution time: 1ms
#define SCD41_CMD_STOP_PERIODIC_MEASUREMENT               0x3f86 // execution time: 500ms
#define SCD41_CMD_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED  0x2416 // execution time 1ms
#define SCD41_CMD_GET_DATA_READY_STATUS                   0xe4b8 // execution time: 1ms
#define SCD41_CMD_POWERDOWN                               0x36e0 // execution time: 1ms
#define SCD41_CMD_WAKEUP                                  0x36f6 // execution time: 20ms

enum {
   SCD41_MODE_PERIODIC=0,
   SCD41_MODE_LP_PERIODIC,
   SCD41_MODE_SINGLE_SHOT
};

class SCD41
{
  public:
    int init(int iSDA=-1, int iSCL=-1, bool bBitBang=false, int32_t iSpeed=100000L);
    int start(int iMode = SCD41_MODE_PERIODIC);
    int stop();
    void wakeup();
    void getSample(); // trigger + read the latest data
    int temperature(); // temperature (C) as an int 10x (e.g. 25.5 = 255)
    int humidity(); // humidity as an int 10x (e.g. 50.5% = 505)
    int co2(); // CO2 as an int
    void shutdown(); // turn off sensor (if possible)
    uint8_t computeCRC8(uint8_t data[], uint8_t len);
    void sendCMD(uint16_t u16Cmd);
    void sendCMD(uint16_t u16Cmd, uint16_t u16Parameter);
    uint16_t readRegister(uint16_t u16Register);

  private:
    uint32_t _iCO2, _iHumidity;
    int32_t _iTemperature;
    int _iType; // sensor type (SCD40/SCD41)
    int _iAddr; // I2C address of device
    int _iMode;
    BBI2C _bbi2c;
};

#endif // __SCD41__
