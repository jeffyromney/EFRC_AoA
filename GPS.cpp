#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <stdint.h>
#include <fcntl.h>



struct termios  config;

typedef union {
  struct  __attribute__((packed)){
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t reserved1;
    uint8_t numSV;
    int32_t lon; // x1E7
    int32_t lat; // x1E7
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN; // mm/s
    int32_t velE; // mm/s
    int32_t velD; // mm/s
    int32_t gSpeed; //mm/s
    int32_t headMot; // x1E5
    uint32_t sAcc; // mm/s
    uint32_t headAcc; // x1E5
    uint16_t pDOP; // x100
    uint8_t reserved2[6];
    int32_t headVeh; // x1E5
    uint8_t reserved3[4];
  }parsed;
  uint8_t rxBuf[92];
} pvt;


pvt inData;
uint8_t msgID;
uint8_t msgClass;
uint16_t msgLength;
uint16_t msgChecksum;

int gpsFD;
int numRead;
int latestRead;


int main(){

  gpsFD = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
  if(gpsFD == -1)
    printf(" COMPORT FAILED TO OPEN ");

  if(tcgetattr(gpsFD, &config) < 0)
    printf(" FAILED TO GET CONFIG DATA\n");
  config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                     INLCR | PARMRK | INPCK | ISTRIP | IXON);
  config.c_oflag = 0;
  config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  config.c_cflag &= ~(CSIZE | PARENB);
  config.c_cflag |= CS8;
  config.c_cc[VMIN]  = 1;
  config.c_cc[VTIME] = 0;
  if(cfsetispeed(&config, B9600) < 0 || cfsetospeed(&config, B9600) < 0) {
    printf("ERROR SETTING BAUD\n");
  }
  if(tcsetattr(gpsFD, TCSAFLUSH, &config) < 0) {
    printf("COULDN\'T APPLY CONFIG\n");
  }
  int state = 0;
  unsigned char c;
  while (true) {
    switch (state){
      case 0: //waiting for header byte 1
        if(read(gpsFD,&c,1) > 0){
          if(c == 0xB5)
            state = 1;
        }
      break;
      case 1: //waiting for header byte 2
        if(read(gpsFD,&c,1) > 0){
          if(c == 0x62)
            state = 2;
          else if (c != 0xB5)
            state = 0;
        }
      break;
      case 2: //waiting for class
        if(read(gpsFD,&c,1) > 0){
          msgClass = c;
          state = 3;
        }
      break;
      case 3: //waiting for ID
        if(read(gpsFD,&c,1) > 0){
          msgID = c;
          state = 4;
        }
      break;
      case 4: //waiting for length
        if(read(gpsFD,&c,1) > 0){
          msgLength = c;
          state = 9;
        }
      break;
      case 9: //waiting for length byte2
        if(read(gpsFD,&c,1) > 0){
          msgLength |= (c << 8);
          state = 5;
        }
      break;
      case 5: //waiting for payload
        numRead = 0;
        while(numRead < msgLength){
          latestRead = read(gpsFD,&inData.rxBuf[numRead],msgLength-numRead);//read msgLength bytes
          if(latestRead > 0){
            numRead += latestRead;
          }
        }
        state = 6;
      break;
      case 6: //waiting for checksum byte 1
        if(read(gpsFD,&c,1) > 0){
          msgChecksum = c << 8;
          state = 7;
        }
      break;
      case 7: //waiting for checksum byte 2
        if(read(gpsFD,&c,1) > 0){
          msgChecksum |= c;
          state = 8;
        }
      break;
      case 8: //Done receiving message. Process now
        if((msgClass == 1) && (msgID == 7) && (msgLength == 92)){
          printf(" %4d %2d %2d %2d %2d %2d  --  Lat: % 2.8f Lon: % 3.8f Alt: % 4.2f velD: % 3.3f Fix Type: %1d Valid: %d\n",
			(inData.parsed.year),
                        (inData.parsed.month),
                        (inData.parsed.day),
                        (inData.parsed.hour),
                        (inData.parsed.minute),
                        (inData.parsed.second),
			((float)inData.parsed.lat)*1E-7,
			((float)inData.parsed.lon)*1E-7,
			((float)inData.parsed.height)*1E-3,
			((float)inData.parsed.velD)*1E-3,
			(inData.parsed.fixType),
			(inData.parsed.valid));
        }
        else
          printf("%d\n", msgLength);
        state = 0;
      break;






    }
  }

}
