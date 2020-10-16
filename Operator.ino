#include  <SoftwareSerial.h>
#include "Wire.h"
#include "AllEncoder.h"
#include "Drive_MOTOR.h"
#include "Serial_Motor.h"

const uint8_t scl = 22;
const uint8_t sda = 21;

char r;
int vr = 2;
boolean callback = false;
int encoder[11];
int ver[11];
AllEncoder Encoder(3200, 360);
Serial_Motor sHand(115200);
Drive_MOTOR Drive;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR serialReceive() {
  portENTER_CRITICAL(&timerMux);
  //  while (Serial.available()) {
  //    r = Serial.read();
  //    drive.read2(r);
  //  }
  callback = true;
  portEXIT_CRITICAL(&timerMux);
}

void runMotor() {
    Encoder.Read();
    int* en = Encoder.GetBuffer();
//    Serial.print(en[0]);
//    Serial.print(", ");
//    Serial.print(en[1]);
//    Serial.print(", ");
//    Serial.print(en[2]);
//    Serial.print(", ");
//    Serial.print(en[3]);
//    Serial.print(", ");
//    Serial.print(en[4]);
//    Serial.println();
  //  if(encoder.GetMagneticAngle()<=360){
  //  enco = Encoder.GetBuffer()[4];
  //    }
  //  byte Data[3];
  //  Data[0] = 0xFF;
  //  Data[1] = enco >> 8;
  //  Data[2] = enco;
  ////  Data[3] = y >> 8;
  ////  Data[4] = y;
  //  Serial.write(Data, sizeof(Data));
  encoder[0] = 0;
  encoder[1] = 30;
  encoder[2] = en[4];
  encoder[3] = en[3];
  encoder[4] = 120;
  encoder[5] = 150;
  encoder[6] = 180;
  encoder[7] = 190;
  encoder[8] = 210;
  encoder[9] = 240;
  encoder[10] = 270;
//  encoder[11] = 300;
  for (int i = 0; i < 11; i++) {
//    encoder[i] = 100;
    ver[i] = int(analogRead(vr) / 11.4);
  }
  Drive.DriveCompute(encoder, ver);
  Drive.start();
}

String x = "";

void setup() {
  //Operator
//  Serial.begin(115200, SERIAL_8N1 , 16, 17);
  Serial.begin(115200);
//  Wire.begin(sda, scl);
//  Wire.setClock(400000);
//  Encoder.Config();
//  Drive.begin();
//  pinMode(vr, INPUT);
  timer = timerBegin(0, 0.001, true);
  timerAttachInterrupt(timer, &serialReceive, true);
  timerAlarmWrite(timer, 2000, true);
  timerAlarmEnable(timer);
}

void loop() {
  if (callback == true) {
    sHand.SetPos(1,1001);
//    Drive.readCurrent();
//    Drive.currentAll();
//    runMotor();
    callback = false;
  }
}
