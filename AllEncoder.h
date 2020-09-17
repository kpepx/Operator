#ifndef AllEncoder_h
#define AllEncoder_h

#include "Arduino.h"

/*
     Address ดูจากขา 2 ต่อ R 4.7k กับอะไร เช่น ต่อgndให้เขียน GND(ตัวพิมพ์ใหญ่เท่านั้น)  มี 5 address
     การตั้งค่า encoder : ถ้าต้องการตั้งค่า void ที่นำหน้าด้วย Set ต้องเขียน Config() ต่อด้วยเสมอเพื่อส่งคำสั่งไปให้ ic
*/

class AllEncoder
{
  public:
    AllEncoder(int sampling, int resolution);
    void Config();
    void Read();
    void GetValue();
    int* GetBuffer();
};

#endif
