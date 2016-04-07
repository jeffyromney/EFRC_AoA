#ifndef MSGDEFS_H
#define MSGDEFS_H

typedef struct __attribute__((packed)) OutMessage_t
{
	uint8_t start[2];
	float dT;
	double cT;
	float ekfVelN;
	float ekfVelE;
	float ekfVelD;
	float ekfPosN;
	float ekfPosE;
	float ekfPosD;
	float ekfRoll;
	float ekfPitch;
	float ekfYaw;
	double ekfLat;
	double ekfLon;
	float ekfAlt;
	float cmpVelN;
	float cmpVelE;
	float cmpVelD;
	float cmpPosN;
	float cmpPosE;
	float cmpPosD;
	float cmpRoll;
	float cmpPitch;
	float cmpYaw;
	double cmpLat;
	double cmpLon;
	float cmpAlt;
	double gpsLat;
	double gpsLon;
	float gpsAlt;
	float accelX;
	float accelY;
	float accelZ;
	float gyroX;
	float gyroY;
	float gyroZ;
	float magX;
	float magY;
	float magZ;
}OutMessage_t;

typedef union OutMessageU_t
{
	OutMessage_t msg;
	char data[sizeof(OutMessage_t)];
}OutMessageU_t;




#endif
