#include "Arduino.h"
#include "Encoder.h"
#include "AllFiveEncoder.h"
#include "Wire.h"

Encoder encoder11("GND");
Encoder encoder12("DCOILP");
Encoder encoder13("DCOILN");
Encoder encoder14("DVCC_SE");
Encoder encoder15("VCC");

void AllFiveEncoder::Config(bool LowP, bool SPD, uint8_t OVS, uint16_t RES){
  encoder11.SetLowPowerModeOn(LowP);
  encoder11.SetSpeedModeOn(SPD);
  encoder11.SetOversampling(OVS);
  encoder11.SetResolution(RES);
  encoder11.Config();
  
  encoder12.SetLowPowerModeOn(LowP);
  encoder12.SetSpeedModeOn(SPD);
  encoder12.SetOversampling(OVS);
  encoder12.SetResolution(RES);
  encoder12.Config();
  
  encoder13.SetLowPowerModeOn(LowP);
  encoder13.SetSpeedModeOn(SPD);
  encoder13.SetOversampling(OVS);
  encoder13.SetResolution(RES);
  encoder13.Config();
  
  encoder14.SetLowPowerModeOn(LowP);
  encoder14.SetSpeedModeOn(SPD);
  encoder14.SetOversampling(OVS);
  encoder14.SetResolution(RES);
  encoder14.Config();
  
  encoder15.SetLowPowerModeOn(LowP);
  encoder15.SetSpeedModeOn(SPD);
  encoder15.SetOversampling(OVS);
  encoder15.SetResolution(RES);
  encoder15.Config();
}

void AllFiveEncoder::Read(){
  if(GNDisReady==1){
    encoder11.Read();
    MAG[0] = int(encoder11.GetMagneticAngle());
  }
  if(DCOILPisReady==1){
    encoder12.Read();
    MAG[1] = int(encoder12.GetMagneticAngle());
    }
  if(DCOILNisReady==1){
    encoder13.Read();
    MAG[2] = int(encoder13.GetMagneticAngle());
  }
  if(DVCC_SEisReady==1){
    encoder14.Read();
    MAG[3] = int(encoder14.GetMagneticAngle());
  }
  if(VCCisReady==1){
    encoder15.Read();
    MAG[4] = int(encoder15.GetMagneticAngle());
    }
}

uint16_t* AllFiveEncoder::GetBuffer(){
  return MAG;
  }
  
void AllFiveEncoder::Print(){
  encoder11.Read();
  Serial.print("enc1: ");
  Serial.print(MAG[0]);
  Serial.print(" enc2: ");
  Serial.print(MAG[1]);
  Serial.print(" enc3: ");
  Serial.print(MAG[2]);
  Serial.print(" enc4: ");
  Serial.print(MAG[3]);
  Serial.print(" enc5: ");
  Serial.println(MAG[4]);
  }

void AllFiveEncoder::CheckAvailable(){
  Wire.beginTransmission (89);
  if (Wire.endTransmission () == 0){GNDisReady= true;}
  Wire.beginTransmission (90);
  if (Wire.endTransmission () == 0){DCOILPisReady= true;}
  Wire.beginTransmission (91);
  if (Wire.endTransmission () == 0){DCOILNisReady= true;}
  Wire.beginTransmission (92);
  if (Wire.endTransmission () == 0){DVCC_SEisReady= true;}
  Wire.beginTransmission (93);
  if (Wire.endTransmission () == 0){VCCisReady= true;}
  delay(10);
  }

  void AllFiveEncoder::CheckAvailablePrint(){
  Serial.println("Address Checking....");
  Serial.print("Address GND(0x59): ");
  Wire.beginTransmission (89);
  if (Wire.endTransmission () == 0){Serial.println("Available");GNDisReady= true;}
  else{Serial.println("Not Found");}
  Serial.print("Address DCOILP, Pin 20 (0x5A): ");
  Wire.beginTransmission (90);
  if (Wire.endTransmission () == 0){Serial.println("Available");DCOILPisReady= true;}
  else{Serial.println("Not Found");}
  Serial.print("Address DCOILN, Pin 19 (0x5B): ");
  Wire.beginTransmission (91);
  if (Wire.endTransmission () == 0){Serial.println("Available");DCOILNisReady= true;}
  else{Serial.println("Not Found");}
  Serial.print("Address DVCC_SE, Pin 13 (0x5C): ");
  Wire.beginTransmission (92);
  if (Wire.endTransmission () == 0){Serial.println("Available");DVCC_SEisReady= true;}
  else{Serial.println("Not Found");}
  Serial.print("Address VCC(0x5D): ");
  Wire.beginTransmission (93);
  if (Wire.endTransmission () == 0){Serial.println("Available");VCCisReady= true;}
  else{Serial.println("Not Found");}
  Serial.println("Done.");
  delay(2000);
  }
