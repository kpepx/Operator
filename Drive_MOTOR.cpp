#include "Drive_MOTOR.h"
#include "Adafruit_TLC5947.h"
#include "MAX11131.h"
#include "pid.h"
#include  <SoftwareSerial.h>
#include <SPI.h>

#define tx_pin  17 //1,17
#define rx_pin  16 //3,16
SoftwareSerial Serial1(rx_pin, tx_pin); // RX, TX
int startbit = 0xFF;

#define NUM_TLC5974 1
#define mosi   13 //mosi
#define clock   14 //clk
#define latch   25 // xlat
Adafruit_TLC5947 tlc = Adafruit_TLC5947(NUM_TLC5974, clock, mosi, latch);

uint8_t spi_cs = 5; // Generic: Arduino 10-pin header D10
// example code declare GPIO interface pins
uint8_t CNVST_pin = 26; // Digital Trigger Input to MAX11131 device
// uint8_t REF_plus_pin = Px_x_PortName_To_Be_Determined; // Reference Input to MAX11131 device
// uint8_t REF_minus_slash_AIN15_pin = Px_x_PortName_To_Be_Determined; // Reference Input to MAX11131 device
uint8_t EOC_pin = 27; // Digital Event Output from MAX11131 device
//// example code declare device instance
MAX11131 g_MAX11131_device(spi_cs, CNVST_pin, EOC_pin, MAX11131::MAX11131_IC);

const int num_motor = 11;

//pid
int kp = 50;
int ki = 2;
int kd = 10;
int gain = 99;

int collectpwm[num_motor][2];

//current
int currentDrive[num_motor][10];
int presum[num_motor];

PID Motor1 = PID(kp, ki, kd);
PID Motor2 = PID(kp, ki, kd);
PID Motor3 = PID(kp, ki, kd);
PID Motor4 = PID(kp, ki, kd);
PID Motor5 = PID(kp, ki, kd);
PID Motor6 = PID(kp, ki, kd);
PID Motor7 = PID(kp, ki, kd);
PID Motor8 = PID(kp, ki, kd);
PID Motor9 = PID(kp, ki, kd);
PID Motor10 = PID(kp, ki, kd);
PID Motor11 = PID(kp, ki, kd);

uint8_t count = 0;
byte dat[5];

boolean Drive_MOTOR::begin() {
  g_MAX11131_device.Init();
  tlc.begin();
}

void Drive_MOTOR::DriveCompute(int input[], int setpoint[]) {
  int motor = 0;
  switch (motor) {
    case 0:
      Motor1.compute(input[0], setpoint[0]);
      Motor2.compute(input[1], setpoint[1]);
      Motor3.compute(input[2], setpoint[2]);
      Motor4.compute(input[3], setpoint[3]);
      Motor5.compute(input[4], setpoint[4]);
      Motor6.compute(input[5], setpoint[5]);
      Motor7.compute(input[6], setpoint[6]);
      Motor8.compute(input[7], setpoint[7]);
      Motor9.compute(input[8], setpoint[8]);
      Motor10.compute(input[9], setpoint[9]);
      Motor11.compute(input[10], setpoint[10]);
      DrivePID(1, Motor1.Out());
      DrivePID(2, Motor2.Out());
      DrivePID(3, Motor3.Out());
      DrivePID(4, Motor4.Out());
      DrivePID(5, Motor5.Out());
      DrivePID(6, Motor6.Out());
      DrivePID(7, Motor7.Out());
      DrivePID(8, Motor8.Out());
      DrivePID(9, Motor9.Out());
      DrivePID(10, Motor10.Out());
      DrivePID(11, Motor11.Out());
      DriveTLC();
      break;
//    case 1:
    default:
      break;
  }

}

void Drive_MOTOR::DrivePID(int num, int output) {
  int Factor = 16;
  int out = abs(int(output * Factor));
  if (output > 0) {
    collectpwm[num - 1][0] = 1;
  }
  if (output < 0) {
    collectpwm[num - 1][0] = 0;
  }
  collectpwm[num - 1][1] = out;
}

void Drive_MOTOR::DriveTLC() {
  for (int i = 1; i < num_motor + 1; i++) {
    int pos = collectpwm[i - 1][0];
    int pwm = collectpwm[i - 1][1];
    if (pos == 0) {
      tlc.setPWM((i * 2) - 2, 0);
      tlc.setPWM((i * 2) - 1, pwm);
    }
    else if (pos == 1) {
      tlc.setPWM((i * 2) - 1, 0);
      tlc.setPWM((i * 2) - 2, pwm);
    }
  }
}

void Drive_MOTOR::start() {
  tlc.write();
  Motor1.start();
  Motor2.start();
  Motor3.start();
  Motor4.start();
  Motor5.start();
  Motor6.start();
  Motor7.start();
  Motor8.start();
  Motor9.start();
  Motor10.start();
  Motor11.start();
}

void Drive_MOTOR::stop() {
  Motor1.stop();
  Motor2.stop();
  Motor3.stop();
  Motor4.stop();
  Motor5.stop();
  Motor6.stop();
  Motor7.stop();
  Motor8.stop();
  Motor9.stop();
  Motor10.stop();
  Motor11.stop();
  for (int i = 1; i < num_motor + 1; i++) {
    tlc.setPWM((i * 2) - 1, 4095);
    tlc.setPWM((i * 2) - 2, 4095);
  }
  tlc.write();
}

void Drive_MOTOR::setpwm(int num, int pwm) {

}

void Drive_MOTOR::setlimit(int input) {

}

void Drive_MOTOR::readCurrent() {
  // this code repeats forever
  // Measure ADC channels in sequence from AIN0 to channelNumber_0_15.
  // @param[in] g_MAX11131_device.channelNumber_0_15: AIN Channel Number
  // @param[in] g_MAX11131_device.PowerManagement_0_2: 0=Normal, 1=AutoShutdown, 2=AutoStandby
  // @param[in] g_MAX11131_device.chan_id_0_1: ADC_MODE_CONTROL.CHAN_ID
  int channelId_0_15 = 11;
  g_MAX11131_device.channelNumber_0_15 = channelId_0_15;
  g_MAX11131_device.PowerManagement_0_2 = 0;
  g_MAX11131_device.chan_id_0_1 = 1;
  g_MAX11131_device.NumWords = g_MAX11131_device.ScanStandardExternalClock();
  // Read raw ADC codes from device into AINcode[] and RAW_misoData16[]
  // @pre one of the MAX11311_Scan functions was called, setting g_MAX11131_device.NumWords
  g_MAX11131_device.ReadAINcode();
  //    for (int i = 0; i < 11; i++) {
  //      currentDrive[i][0] = g_MAX11131_device.AINcode[i];
  //    }
  for (int i = 0; i < num_motor; i++) {
    currentDrive[i][count] = g_MAX11131_device.AINcode[i];
  }
  count += 1;
}

int Drive_MOTOR::showCurrent(int Current, int Output) {
  byte Data[6];
  Data[0] = 0xFF;
  Data[1] = Current >> 8;
  Data[2] = Current;
  Data[3] = Output >> 8;
  Data[4] = Output;
  Data[5] = (Data[1] + Data[2] + Data[3] + Data[4] + 1) & 0xFF;
  Serial.write(Data, sizeof(Data));
}

void Drive_MOTOR::filter(int motor, int Output) {
  if (count == 10) {
    count = 0;
  }
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += currentDrive[motor - 1][i];
  }
  sum = sum / 10;
  presum[motor - 1] = (int)(((sum * (100 - gain)) + (presum[motor - 1]  *  gain)) / 100);
  showCurrent(presum[motor - 1], Output);
}

void Drive_MOTOR::currentAll() {
  filter(1, 1);
  filter(2, 2);
  filter(3, 3);
  filter(4, 4);
  filter(5, 5);
  filter(6, 6);
  filter(7, 7);
  filter(8, 8);
  filter(9, 9);
  filter(10, 10);
  filter(11, 11);
}

//void Drive_MOTOR::writes(int16_t num, int16_t pos, int16_t pwm) {
//  byte Data[5];
//  Data[0] = startbit;
//  Data[1] = num * 0x10 + pos;
//  Data[2] = (pwm >> 8);
//  Data[3] = pwm;
//  Data[4] = (Data[1] + Data[2] + Data[3] + 1) & 0xFF;
//  //****************send*****************//
//  Serial.print(num);
//  Serial.print(",");
//  Serial.println(pos);
//  Serial1.begin(115200);
//  Serial1.write(Data, sizeof(Data));
//  //*************************************//
//}

//void Drive_MOTOR::write2(int16_t num, int16_t pos) {
//  byte Data[4];
//  Data[0] = startbit;
//  Data[1] = num * 0x10 + (pos >> 8);
//  Data[2] = pos;
//  Data[3] = (Data[1] + Data[2] + 1) & 0xFF;
//  //****************send*****************//
//  Serial1.begin(115200);
//  Serial1.write(Data, sizeof(Data));
//  //  Serial.write(Data, sizeof(Data));
//  //*************************************//
//}
//
//void Drive_MOTOR::read(byte Data) {
//  dat[count] = Data;
//  count++;
//  if (count == 5) {
//    if (dat[0] == startbit) {
//      byte check = (dat[1] + dat[2] + dat[3] + 1) & 0xFF;
//      if (check == dat[4]) {
//        int motor = (dat[1] & 0xF0) >> 4;
//        int pos = dat[1] & 0x0F;
//        int pwm = (dat[2] << 8) + dat[3];
//        //        Drive(motor, pos, pwm);
//        if (motor == 1) {
//          readCurrent();
//        }
//        count = 0;
//      }
//      else {
//        count = 0;
//      }
//    }
//    else {
//      count = 0;
//    }
//  }
//  if (dat[0] != startbit) {
//    count = 0;
//  }
//}
//
//void Drive_MOTOR::read2(byte Data) {
//  dat[count] = Data;
//  count++;
//  if (count == 4) {
//    if (dat[0] == startbit) {
//      byte check = (dat[1] + dat[2] + 1) & 0xFF;
//      if (check == dat[3]) {
//        int motor = (dat[1] & 0xF0) >> 4;
//        int pos = ((dat[1] & 0x0F) << 8) + dat[2];
//        //        DrivePID(motor, pos, 180);
//        //        DrivePID2(motor, pos, 180);
//        //        DrivePID3(motor, pos, 180);
//        //        DrivePID(motor, currentDrive[motor-1], 50);
//        //          Drive(motor,1,4095);
//        //          Drive(motor,0,4095);
//        count = 0;
//      }
//      else {
//        count = 0;
//      }
//    }
//    else {
//      count = 0;
//    }
//  }
//  if (dat[0] != startbit) {
//    count = 0;
//  }
//}
