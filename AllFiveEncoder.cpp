#include "Arduino.h"
#include "Encoder.h"
#include "AllFiveEncoder.h"
#include "Wire.h"

Encoder encoder1("GND");
Encoder encoder2("DCOILP");
Encoder encoder3("DCOILN");
Encoder encoder4("DVCC_SE");
Encoder encoder5("VCC");

void AllFiveEncoder::Config(bool LowP, bool SPD, uint8_t OVS, uint16_t RES){
  encoder1.SetLowPowerModeOn(LowP);
  encoder1.SetSpeedModeOn(SPD);
  encoder1.SetOversampling(OVS);
  encoder1.SetResolution(RES);
  encoder1.Config();
  
  encoder2.SetLowPowerModeOn(LowP);
  encoder2.SetSpeedModeOn(SPD);
  encoder2.SetOversampling(OVS);
  encoder2.SetResolution(RES);
  encoder2.Config();
  
  encoder3.SetLowPowerModeOn(LowP);
  encoder3.SetSpeedModeOn(SPD);
  encoder3.SetOversampling(OVS);
  encoder3.SetResolution(RES);
  encoder3.Config();
  
  encoder4.SetLowPowerModeOn(LowP);
  encoder4.SetSpeedModeOn(SPD);
  encoder4.SetOversampling(OVS);
  encoder4.SetResolution(RES);
  encoder4.Config();
  
  encoder5.SetLowPowerModeOn(LowP);
  encoder5.SetSpeedModeOn(SPD);
  encoder5.SetOversampling(OVS);
  encoder5.SetResolution(RES);
  encoder5.Config();
}

void AllFiveEncoder::Read(){
  if(GNDisReady==1){
    encoder1.Read();
    MAG[0] = int(encoder1.GetMagneticAngle());
  }
  if(DCOILPisReady==1){
    encoder2.Read();
    MAG[1] = int(encoder2.GetMagneticAngle());
    }
  if(DCOILNisReady==1){
    encoder3.Read();
    MAG[2] = int(encoder3.GetMagneticAngle());
  }
  if(DVCC_SEisReady==1){
    encoder4.Read();
    MAG[3] = int(encoder4.GetMagneticAngle());
  }
  if(VCCisReady==1){
    encoder5.Read();
    MAG[4] = int(encoder5.GetMagneticAngle());
    }
}

uint16_t* AllFiveEncoder::GetBuffer(){
  return MAG;
  }
  
void AllFiveEncoder::Print(){
  encoder1.Read();
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
