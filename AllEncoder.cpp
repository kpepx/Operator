#include "Arduino.h"
#include "AllEncoder.h"
#include "Encoder.h"
#include "Wire.h"

int magneticAngle[5];
/* version 1.1 edited on 27/7/2020 7:38 PM */
Encoder encoder1("GND");
Encoder encoder2("DCOILP");
Encoder encoder3("DCOILN");
Encoder encoder4("DVCC_SE");
Encoder encoder5("VCC");

AllEncoder::AllEncoder(int sampling, int resolution)
{
  encoder1.SetOversampling(sampling);
  encoder1.SetResolution(resolution);
  
  encoder2.SetOversampling(sampling);
  encoder2.SetResolution(resolution);
  
  encoder3.SetOversampling(sampling);
  encoder3.SetResolution(resolution);
  
  encoder4.SetOversampling(sampling);
  encoder4.SetResolution(resolution);
  
  encoder5.SetOversampling(sampling);
  encoder5.SetResolution(resolution);
  
}

void AllEncoder::Config(){
  encoder1.Config();
  encoder2.Config();
  encoder3.Config();
  encoder4.Config();
  encoder5.Config();
}

void AllEncoder::Read(){
  encoder1.Read();
  encoder2.Read();
  encoder3.Read();
  encoder4.Read();
  encoder5.Read();
}

void AllEncoder::GetValue(){
  magneticAngle[0] = encoder1.GetMagneticAngle();
  magneticAngle[1] = encoder2.GetMagneticAngle();
  magneticAngle[2] = encoder3.GetMagneticAngle();
  magneticAngle[3] = encoder4.GetMagneticAngle();
  magneticAngle[4] = encoder5.GetMagneticAngle();
}

int* AllEncoder::GetBuffer(){
  GetValue();
  return magneticAngle;
}
