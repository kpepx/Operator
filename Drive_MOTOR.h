#ifndef _DRIVE_MOTOR_H
#define _DRIVE_MOTOR_H

#include <Arduino.h>

class Drive_MOTOR
{
private:
    int16_t _num, _pos, _pwm;
public:
    boolean begin();
    
    void writes(int16_t num, int16_t pos, int16_t pwm);
    void write2(int16_t num, int16_t pos);
    void read(byte Data);
    void read2(byte Data);
    
    void DriveCompute(int input[], int setpoint[]);
    void DrivePID(int num, int output);
    void DriveTLC();
    void motor(int num);
    void start();
    void stop();

    void setpwm(int num, int pwm);
    void setlimit(int input);

    void currentAll();
    void readCurrent();
    int showCurrent(int Current, int Output);
    void filter(int motor, int Output);
};
#endif
