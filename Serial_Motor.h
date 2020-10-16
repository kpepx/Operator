#ifndef _SERIAL_MOTOR_H
#define _SERIAL_MOTOR_H

#include <Arduino.h>
#include <SoftwareSerial.h>

// #define DATA_DEBUG				//Command this line to disable some of pc.printf in DYNAMIXEL

#define SHIFT_TO_LSB(w)			(uint8_t((uint16_t(w)) & 0x00ff))
#define SHIFT_TO_MSB(w)			(uint8_t((uint16_t(w) >> 8) & 0x00ff))

#define HEADER		    0xff

//MODE
#define WRITE           0x01
#define READ            0x02

//REGISTER
#define POSITION        0x01
#define ANGLE           0x02
#define VOLTAGE         0x03

class Serial_Motor
{
private:
//    SoftwareSerial sHAND; // RX, TX
    static const int nBytes = 10;
    void sendIPacket();
	void getRPacket();
    uint8_t iPacket[nBytes];
    uint8_t rPacket[nBytes];
    uint8_t parameter[2];
    uint8_t targetID;
    uint8_t targetInst;		//instruction for targeted DYNAMIXEL
	uint16_t targetAddr;	//address for instruction
	uint16_t crc;
	uint16_t iPacketLength;	//Instruction Packet Length
	uint16_t rPacketLength;
    uint16_t crc;
    unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
public:
    Serial_Motor(uint32_t baudrate);

    void SetPos(uint8_t id,uint16_t pos);
    void GetPos(uint8_t id);
    void iRead(uint8_t length, uint16_t address);
	void iWrite(uint8_t length, uint16_t address, uint8_t param[]);
};
#endif
