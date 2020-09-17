#ifndef Encoder_h
#define Encoder_h

#include "Arduino.h"

/*  
 *   Address ดูจากขา 2 ต่อ R 4.7k กับอะไร เช่น ต่อgndให้เขียน GND(ตัวพิมพ์ใหญ่เท่านั้น)  มี 5 address
 *   การตั้งค่า encoder : ถ้าต้องการตั้งค่า void ที่นำหน้าด้วย Set ต้องเขียน Config() ต่อด้วยเสมอเพื่อส่งคำสั่งไปให้ ic
*/

class Encoder
{
  public:
    Encoder(String Address);
    void Config();
    void SetEncoderSleep(bool TF);
    void SetLinearOn(bool TF);
    void SetCounterOn(bool TF);
    void SetLowPowerModeOn(bool TF);
    void SetSpeedModeOn(bool TF);
    void SetOversampling(int ovs);
    void SetResolution(int Res);
    void Read();
    int GetRawMagneticAngle();
    int GetMagneticAngle();
    float GetFloatMagneticAngle();
    int GetIncrementLinearCounter();
    int GetConfigurationRegister();
    int GetResolutionRegister();
    uint8_t _CONF0 = 0;
  private:
    uint8_t _size = 10;
    uint8_t _Address = 0x59;
    uint8_t _CONF = 0;
    uint8_t _SLPbit = 0;
    uint8_t _LINbit = 0;
    uint8_t _CNTbit = 0;
    uint8_t _PWRbit = 0;
    uint8_t _SPDbit = 0;
    uint8_t _OVCSbit1 = 1;
    uint8_t _OVCSbit2 = 0;
    int _RES = 360;
    uint16_t _MAG = 0;
    uint32_t _ILC = 0;
    uint16_t _RES_read = 0;
    uint8_t _CSSEND = 0;
};

#endif
