#include "Arduino.h"
#include "pid.h"

PID::PID(int p, int i, int d)
    :
    on(ON) {
      tune(p,i,d);
    }

void PID::tune(int p, int i, int d) {
  kp = p;
  ki = i;
  kd = d;
}

bool PID::compute(int input, int setpoint) {
  if (!on) return false;
  //insert frequncy time
  int Factor = 16;
  unsigned long now = millis();
  unsigned long timeChange = now - time_prev;
  double error = setpoint - input;
  integral += ki * error; //intrgral
  input_change = (input - input_prev);
  //  Serial.println(Output);
  if (integral > 255) {
    integral = 255;
  }
  if (integral < -255) {
    integral = -255;
  }
  Output = (kp * error + integral + kd * input_change) / 100;
  if (Output > 255) {
    Output = 255;
  }
  if (Output < -255) {
    Output = -255;
  }
  input_prev = input;
  time_prev = now;
  return true;
}

int PID::Out(){
  return Output;
}

void PID::start() {
  if (!on) {
    on = ON;
  }
}

void PID::stop() {
  on = OFF;
}
