#ifndef MSGDEFS_H
#define MSGDEFS_H

typedef struct __attribute__((packed)) OutMessage_t
{
	uint8_t start[2];
	double dT;
	double cT;
	double ekfVelN;
	double ekfVelE;
	double ekfVelD;
	double ekfPosN;
	double ekfPosE;
	double ekfPosD;
	double ekfRoll;
	double ekfPitch;
	double ekfYaw;
	double ekfLat;
	double ekfLon;
	double ekfAlt;
	double cmpVelN;
	double cmpVelE;
	double cmpVelD;
	double cmpPosN;
	double cmpPosE;
	double cmpPosD;
	double cmpRoll;
	double cmpPitch;
	double cmpYaw;
	double cmpLat;
	double cmpLon;
	double cmpAlt;
	double gpsLat;
	double gpsLon;
	double gpsAlt;
	double accelX;
	double accelY;
	double accelZ;
	double gyroX;
	double gyroY;
	double gyroZ;
	double magX;
	double magY;
	double magZ;
}OutMessage_t;

typedef union OutMessageU_t
{
	OutMessage_t msg;
	char data[sizeof(OutMessage_t)];
}OutMessageU_t;

















#endif
