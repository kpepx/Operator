#ifndef _PID_H
#define _PID_H

#pragma once
const bool OFF = 0;
const bool ON = 1;

class PID {
    bool on;
    int motor, input, setpoint, input_prev, Output, integral, input_change;
    unsigned long time_prev;
    double last_error;
    int kp, ki, kd;

  public:
  
    // by default off
    PID(int p, int i, int d);
    void tune(int p, int i, int d);
    bool compute(int input, int setpoint);
    int Out();
    void start();
    void stop();
};
#endif
