
#include "Arduino.h"
#include "Encoder.h"
#include "Wire.h"

/* version 1.1 edited on 27/7/2020 7:38 PM */

Encoder::Encoder(String Address)
{
    if(Address == "GND"){_Address = 0x59;}
    if(Address == "DCOILP"){_Address = 0x5A;}
    if(Address == "DCOILN"){_Address = 0x5B;}
    if(Address == "DVCC_SE"){_Address = 0x5C;}
    if(Address == "VCC"){_Address = 0x5D;}
}
void Encoder::SetEncoderSleep(bool TF)
{
    if(TF == true){_SLPbit = 1;}
    if(TF == false){_SLPbit = 0;}
}

void Encoder::SetLinearOn(bool TF)
{
    if(TF == true){_LINbit = 1;}
    if(TF == false){_LINbit = 0;}
}

void Encoder::SetCounterOn(bool TF)
{
    if(TF == true){_CNTbit = 1;}
    if(TF == false){_CNTbit = 0;}
}
void Encoder::SetLowPowerModeOn(bool TF)
{
    if(TF == true){_PWRbit = 1;}
    if(TF == false){_PWRbit = 0;}
}
void Encoder::SetSpeedModeOn(bool TF)
{
    if(TF == true){_SPDbit = 1;}
    if(TF == false){_SPDbit = 0;}
}
void Encoder::SetOversampling(int ovs)
{
    if(ovs == 2){_OVCSbit1 = 0;_OVCSbit2 = 0;}
    if(ovs == 4){_OVCSbit1 = 0;_OVCSbit2 = 1;}
    if(ovs == 8){_OVCSbit1 = 1;_OVCSbit2 = 0;}
    if(ovs == 32){_OVCSbit1 = 1;_OVCSbit2 = 1;}
    else{_OVCSbit1 = 1;_OVCSbit2 = 0;}
}

void Encoder::SetResolution(int Res)
{
    if(Res>=32768){_RES = 32768;}
    else{_RES = Res;}
}

void Encoder::Config()
{
    uint8_t RESL = _RES;
    uint8_t RESH = _RES>>8;
    uint8_t CONF = (_SLPbit<<7)|(_LINbit<<5)|(_CNTbit<<4)|(_PWRbit<<3)|(_SPDbit<<2)|(_OVCSbit1<<1)|_OVCSbit2;
    uint16_t CS = 0xFF - (CONF+RESH+RESL) + 0x01;
    Wire.beginTransmission(_Address);
    Wire.write(CONF);
    Wire.write(RESH);
    Wire.write(RESL);
    Wire.write(CS);
    Wire.endTransmission();
    delay(150);
}

void Encoder::Read()
{
    Wire.beginTransmission(_Address);
    Wire.requestFrom(_Address, _size);
    if (10 <= Wire.available()){
        _MAG = (Wire.read() | ((int16_t)Wire.read()<<8));
        _ILC = (Wire.read() | Wire.read()<<8 | Wire.read()<<16| Wire.read()<<32);
        _CONF = ((int8_t)Wire.read());
        _RES_read = (Wire.read() | Wire.read()<<8);
    }
    Wire.endTransmission();
}

int Encoder::GetRawMagneticAngle()
{
  return _MAG;
}
float Encoder::GetFloatMagneticAngle()
{
  float p = (float)_MAG / ((float)_RES/360.0f);
  return p;
}
int Encoder::GetMagneticAngle()
{
  int p = (int)_MAG / ((int)_RES/360);
  return p;
}
int Encoder::GetIncrementLinearCounter()
{
  return _ILC;
}
int Encoder::GetConfigurationRegister()
{
  return _CONF;
}
int Encoder::GetResolutionRegister()
{
  return _RES_read;
}
