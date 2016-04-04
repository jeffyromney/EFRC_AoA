#ifndef GPS_H
#define GPS_H



void readGpsData();
void gpsParser();

//GPS Variables
float gpsGndSpd = 0;
float gpsCourse = 0;
float gpsVelD = 0;
float posNED[3] = {0,0,0};
bool newDataGps = false;
struct gpsDataStruct {
        float lat;
        float lng;
        float alt;
        unsigned long time;
        float course;
        float gndSpd;
        float velN;
        float velE;
        float velD;
        int status;
};
// Queue to hold incoming gpsData.
//std::queue<gpsDataStruct> gpsInputQueue;

struct termios  configGPS;
struct __attribute__((packed)) pvtMessage{
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
}gps_t;

typedef union {
  pvtMessage parsed;
  uint8_t rxBuf[92];
} pvt;




#endif
