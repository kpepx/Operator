#ifndef AllFiveEncoder_h
#define AllFiveEncoder_h

#include "Arduino.h"

class AllFiveEncoder
{
  public:
    void Config(bool LowP, bool SPD, uint8_t OVS, uint16_t RES);
    void Read();
    uint16_t* GetBuffer();
    void Print();
    void CheckAvailable();
    void CheckAvailablePrint();
  private:
    uint16_t MAG[5];
    bool GNDisReady= false;
    bool DCOILPisReady= false;
    bool DCOILNisReady= false;
    bool DVCC_SEisReady= false;
    bool VCCisReady= false;
};

#endif
