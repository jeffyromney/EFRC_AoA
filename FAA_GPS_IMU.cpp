#include <stdlib.h>
#include <wiringPi.h>
#include <stdio.h> 
#include <cmath> 
#include <wiringPiI2C.h> 
#include <time.h>
#include <thread>
#include <queue>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <netdb.h>
#include <stdint.h>
#include <string.h>
#include <string>
#include <csignal>





int loopCount = 0;

int press_address = 40; //1001000 written as decimal number 
int reading = 0; 
int mask = 0x3FFF; //63; //(0 0 1 1 1 1 1 1 ) 
int maskstatus = 192; //(1 1 0 0 0 0 0 0 ) 
int Status; 

float pfwdnoload, p45noload; 
float pfwd, p45; 
float pfwd_p45, Alpha; // pfwd_p45 is Pfwd/P45 
float pfwdcorr, p45corr; 
float A, B;

int pressFD;
int ardFD;

const int unit1 = 0;
const int unit2 = 1;
unsigned long cT ;
double eT ;
unsigned long pT ;
unsigned int loopPeriod = 10;


//Arduino Stuff
struct arduinoDataStruct{
  float alpha;
  float pfwd;
  float p45;
};
std::queue<arduinoDataStruct> arduinoInputQueue;
unsigned char rxBuf[7];
unsigned int rxBufIndexer = 0;
struct termios  config;

//GPS Stuff
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

pvtMessage gpsData;

pvt inData;
uint8_t msgID;
uint8_t msgClass;
uint16_t msgLength;
uint16_t msgChecksum;

int gpsFD;
int gpsNumRead;
int gpsLatestRead;

//IMU Stuff
int imuFD;
   // IMU Data
float gyroData [3];
float accelData [3];
float magData [3];
float attData [3];
int imu_address = 0x29;



//Socket Stuff
int sockfd, portno, n;
struct sockaddr_in serv_addr;
struct hostent *server;

//File Output
FILE * rawSerialFile;
FILE * datalogFile;



float calcAltitude(float pressure){
  float A = pressure/101325;
  float B = 1/5.25588;
  float C = pow(A,B);
  C = 1 - C;
  C = C /0.0000225577; 
  return C;
}

float MS4525Getpressure(){
  int reading = wiringPiI2CReadReg16(pressFD,0);
  reading = ((reading & 0x00ff)<<8) | ((reading & 0xff00)>>8);
  reading = reading & mask;
  return reading;
}

// runs as a thread to continuously check serial port and parse new data
void arduinoReadFunc(){
  arduinoDataStruct inputStruct;
  int state = 0;
  unsigned char c;
  while(true){
    switch(state){
      case 0://waiting for the newline char
        if (read(ardFD,&c,1) > 0){
          fwrite(&c, sizeof(char), 1, rawSerialFile);
          if(c == '\n')
            state = 1;//received newline. start parsing
        }
        else{
          usleep(1000);
        }
      break;
      case 1:
        int numRead = 0;
        while(numRead < 7){
          int latestRead = read(ardFD,&rxBuf[numRead],7-numRead);//read 7 bytes
          if(latestRead > 0){
            fwrite(&rxBuf[numRead], sizeof(char), latestRead, rawSerialFile);
            numRead += latestRead;
          }
          else{
            usleep(1000);
          }
        }
        int tmpAlpha = ((rxBuf[0] << 8) | rxBuf[1]);
	if(tmpAlpha > 2500){
          tmpAlpha -= 65536;
        }
          inputStruct.alpha = ((float)tmpAlpha)/50.0;
          inputStruct.pfwd  = ((float)((rxBuf[2] << 8) | rxBuf[3]));
          inputStruct.p45   = ((float)((rxBuf[4] << 8) | rxBuf[5]));
          arduinoInputQueue.push(inputStruct);
        state = 0;
      break;
    }// switch end
  }// while end
}// function end


//runs as thread to continuously get and update GPS Data
//Unlike the Arduino data, the GPS data is not queued because it runs at a much slower rate. We will likely be getting many (about 5) readings per solution
void gpsReadFunction(){

  int state = 0;
  unsigned char c;
  while (true) {
    switch (state){
      case 0: //waiting for header byte 1
        if(read(gpsFD,&c,1) > 0){
          if(c == 0xB5)
            state = 1;
        }
        else{
          usleep(1000);
        }
      break;
      case 1: //waiting for header byte 2
        if(read(gpsFD,&c,1) > 0){
          if(c == 0x62)
            state = 2;
          else if (c != 0xB5)
            state = 0;
        }
        else{
          usleep(1000);
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
        gpsNumRead = 0;
        while(gpsNumRead < msgLength){
          gpsLatestRead = read(gpsFD,&inData.rxBuf[gpsNumRead],msgLength-gpsNumRead);//read msgLength bytes
          if(gpsLatestRead > 0){
           gpsNumRead += gpsLatestRead;
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
          gpsData = inData.parsed;
//          printf(" %4d %2d %2d %2d %2d %2d  --  Lat: % 2.8f Lon: % 3.8f Alt: % 4.2f velD: % 3.3f Fix Type: %1d Valid: %d\n",
//                        (inData.parsed.year),
//                        (inData.parsed.month),
//                        (inData.parsed.day),
//                        (inData.parsed.hour),
//                        (inData.parsed.minute),
//                        (inData.parsed.second),
//                        ((float)inData.parsed.lat)*1E-7,
//                        ((float)inData.parsed.lon)*1E-7,
//                        ((float)inData.parsed.height)*1E-3,
//                        ((float)inData.parsed.velD)*1E-3,
//                        (inData.parsed.fixType),
//                        (inData.parsed.valid));
        }
        state = 0;
      break;
    }//end switch
  }//end while
}//end gpsReadFunction


//Get Roll Value from IMU
float imuGetRoll(){
  short reading = wiringPiI2CReadReg16(imuFD, 0x1C);
  // correct for msb glitches
  if (reading < -2880)
    reading += 0x8000;
  else if (reading > 2880)
    reading -= 0x8000;
  float roll = -((float)reading)/16.0;
  return roll;
}

//Get Pitch Value from IMU
float imuGetPitch(){
  short reading = wiringPiI2CReadReg16(imuFD, 0x1E);
  // correct for msb glitches
  if (reading < -2880)
    reading += 0x8000;
  else if (reading > 2880)
    reading -= 0x8000;
  float pitch = ((float)reading)/16.0;
  return pitch;
}

//Get Heading Value from IMU
float imuGetHeading(){
  short reading = wiringPiI2CReadReg16(imuFD, 0x1A);
  reading &= ~0x8000;
  float heading = ((float)reading)/16.0;
  return heading;
}

//Get Accelerometer Values from IMU
void imuGetAccel(){
  int baseAddr = 0x08;
  for (int i=0;i<3;i++){
    short reading = wiringPiI2CReadReg16(imuFD,(baseAddr + (i*2)));
    // correct for msb glitches
    if (reading < -16500)
      reading += 0x8000;
    else if (reading > 16500)
      reading -= 0x8000;
    accelData[i] = ((float)reading)/100.0;
  }
  return;
}

//Get Gyroscope Values from IMU
void imuGetGyro(){
  int baseAddr = 0x14;
  for (int i=0;i<3;i++){
    short reading = wiringPiI2CReadReg16(imuFD,(baseAddr + (i*2)));
    //correct for msb glitches
    if (reading < -16500)
      reading += 0x8000;
    else if (reading > 16500)
      reading -= 0x8000;
    gyroData[i] = ((float)reading)/16.0;
  }
  return ;
}

//Get Magnetometer Values from IMU
void imuGetMag(){
  int baseAddr = 0x0E;
  for (int i=0;i<3;i++){
    short reading = wiringPiI2CReadReg16(imuFD,(baseAddr + (i*2)));
    //correct for msb glitches
    if (reading < -16500)
      reading += 0x8000;
    else if (reading > 16500)
      reading -= 0x8000;
    magData[i] = ((float)reading)/16.0;
  }
  return ;
}


void imuGetAttitude(){
  attData[0] = imuGetRoll();
  attData[1] = imuGetPitch();
  attData[2] = imuGetHeading();
}











void error(const char *msg)
{
    perror(msg);
    exit(0);
}

void signalHandler( int signum )
{
    printf(" INTERUPT SIGNAL: %d ", signum);
    // cleanup and close up stuff here  
    // terminate program  
  close(sockfd);
  portno = 5005;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error(" ERROR opening socket ");
  server = gethostbyname("155.31.242.65");
  if (server == NULL) {
    fprintf(stderr," ERROR, no such host ");
  }
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr,
    (char *)&serv_addr.sin_addr.s_addr,
    server->h_length);
  serv_addr.sin_port = htons(portno);
  if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
    printf(" ERROR connecting ");

}

int main(void){
  printf("MAIN START");

  rawSerialFile = fopen("rawSerial.txt", "wb");
  datalogFile = fopen("datalog.txt", "wb");
  signal (SIGPIPE, signalHandler);
  portno = 5005;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error(" ERROR opening socket ");
  server = gethostbyname("155.31.242.65");
  if (server == NULL) {
    fprintf(stderr,"ERROR, no such host");
  }
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr,
    (char *)&serv_addr.sin_addr.s_addr,
    server->h_length);
  serv_addr.sin_port = htons(portno);
  if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
    printf(" ERROR connecting ");
  printf("Socket Done");
  pT = millis();
  wiringPiSetup();
  // MS4525 I2C Handle
  pressFD = wiringPiI2CSetup (0x28) ;
  // IMU I2C Handle
  imuFD = wiringPiI2CSetup (imu_address);


  pinMode (unit1, OUTPUT); // set pin 8 control for sensor 1
  pinMode (unit2, OUTPUT); // set pin 9 control for sensor 2
  digitalWrite( unit1, LOW); // set both units off line
  digitalWrite( unit2, LOW);

  printf("Opening Arduino");
  ardFD = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
  if(ardFD == -1)
    printf(" COMPORT FAILED TO OPEN ");

  if(tcgetattr(ardFD, &config) < 0)
    printf(" FAILED TO GET CONFIG DATA\n");
  config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                     INLCR | PARMRK | INPCK | ISTRIP | IXON);
  config.c_oflag = 0;
  config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  config.c_cflag &= ~(CSIZE | PARENB);
  config.c_cflag |= CS8;
  config.c_cc[VMIN]  = 1;
  config.c_cc[VTIME] = 0;
  if(cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0) {
    printf("ERROR SETTING BAUD\n");
  }
  if(tcsetattr(ardFD, TCSAFLUSH, &config) < 0) {
    printf("COULDN\'T APPLY CONFIG\n");
  }
  printf("Done Arduino");


  gpsFD = open("/dev/ttyACM1", O_RDWR | O_NOCTTY | O_NDELAY);
  if(gpsFD == -1)
    printf(" GPS COMPORT FAILED TO OPEN ");

  if(tcgetattr(gpsFD, &configGPS) < 0)
    printf(" FAILED TO GET GPS CONFIG DATA\n");
  configGPS.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                     INLCR | PARMRK | INPCK | ISTRIP | IXON);
  configGPS.c_oflag = 0;
  configGPS.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  configGPS.c_cflag &= ~(CSIZE | PARENB);
  configGPS.c_cflag |= CS8;
  configGPS.c_cc[VMIN]  = 1;
  configGPS.c_cc[VTIME] = 0;
  if(cfsetispeed(&configGPS, B9600) < 0 || cfsetospeed(&configGPS, B9600) < 0) {
    printf("ERROR SETTING GPS BAUD\n");
  }
  if(tcsetattr(gpsFD, TCSAFLUSH, &configGPS) < 0) {
    printf("COULDN\'T APPLY GPS CONFIG\n");
  }
  printf("Done GPS");

  A = -16.908083; // These values must be adjusted for individual
  B = 42.415008; // probes and probe locations

  pfwdnoload = 8147.9; //counts Adjust these values for
  p45noload = 8144.5; //counts the individual sensors

  std::thread arduinoParserThread(arduinoReadFunc);
  std::thread gpsParserThread(gpsReadFunction);
  usleep(500000);
  wiringPiI2CWriteReg8(imuFD,0x3D,00); // set IMU to CONFIG operation mode
  wiringPiI2CWriteReg8(imuFD,0x3B,00); // set Units
  wiringPiI2CWriteReg8(imuFD,0x41,36); // switch x/y axes
  wiringPiI2CWriteReg8(imuFD,0x42,0);  // negate x/y axes
  wiringPiI2CWriteReg8(imuFD,0x3D,12); // set IMU to NDOF_FMC_OFF operation mode

  printf("Running Now");
  while(true){ // Main loop
    cT = millis();
    eT = cT - pT;
    eT *= 1000.0;
//(cT - pT >= loopPeriod && 
    if(!arduinoInputQueue.empty()){
      arduinoDataStruct latestArduinoData = arduinoInputQueue.back();
      while(!arduinoInputQueue.empty()){
        arduinoInputQueue.pop();
//        break;  //uncomment this and change "back" 3 lines above to "front" to use all values from arduino even if old
      }
      pT = cT;
      float hz = 100/eT;
      hz *= 2;

      //Read IMU Data
      imuGetAccel();
      imuGetGyro();
      imuGetMag();
      imuGetAttitude();

      float flightPathAngle = (((float)gpsData.velD*(-1E-3)) / sqrt(pow(((float)gpsData.velN*1E-3),2) + pow(((float)gpsData.velN*1E-3),2)))*57.2958;

      digitalWrite(unit1,HIGH );
      float pfwd = MS4525Getpressure(); // Get Differential Pressure from unit 1
      digitalWrite( unit1, LOW);

      pfwdcorr = pfwd - pfwdnoload; // Account for no load

      digitalWrite(unit2,HIGH );
      float p45 = MS4525Getpressure(); // Get Differential Pressure from unit 2
      digitalWrite( unit2, LOW);

      p45corr = p45 - p45noload; // Account for noload

      pfwd_p45 = pfwdcorr/p45corr;

      Alpha = A*(pfwd_p45) + B;





      char writeBuffer [255];
      int numWritten = sprintf(writeBuffer,"%lu %9.4f %7.2f %7.2f %9.4f %7.2f %7.2f %12.8f %12.8f %7.1f %6.3f %6.3f %6.3f %02u:%02u:%02u % 6.1f %6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f\n",
              cT, Alpha, pfwd, p45, latestArduinoData.alpha, latestArduinoData.pfwd, latestArduinoData.p45,((float)gpsData.lat)*1E-7, ((float)gpsData.lon)*1E-7, ((float)gpsData.height)*0.00328084,
              ((float)gpsData.velN*1E-3)*1.94384, ((float)gpsData.velE)*1E-3*1.94384, ((float)gpsData.velD)*1E-3*1.94384, gpsData.hour,gpsData.minute, gpsData.second, ((float)gpsData.gSpeed)*1E-3*1.94384, ((float)gpsData.headMot)*1E-5,
              accelData[0], accelData[1], accelData[2], gyroData[0], gyroData[1], gyroData[2], magData[0], magData[1], magData[2], attData[0], attData[1],attData[2], flightPathAngle);
      try{
        fwrite(writeBuffer, sizeof(char), numWritten, datalogFile);
        write(sockfd,&writeBuffer,numWritten);
      }
      catch(int e){
        printf("\nException Thrown: %d\n",e);
      }
//      printf("\n%6lu %9.4f %7.2f %7.2f    %9.4f %7.2f %7.2f",cT, Alpha, pfwd, p45, latestArduinoData.alpha, latestArduinoData.pfwd, latestArduinoData.p45);
      printf("\n%lu %9.4f %7.2f %7.2f %9.4f %7.2f %7.2f %12.8f %12.8f %7.1f %6.3f %6.3f %6.3f %02u:%02u:%02u % 6.1f %6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f",
              cT, Alpha, pfwd, p45, latestArduinoData.alpha, latestArduinoData.pfwd, latestArduinoData.p45,((float)gpsData.lat)*1E-7, ((float)gpsData.lon)*1E-7, ((float)gpsData.height)*0.00328084,
              ((float)gpsData.velN*1E-3)*1.94384, ((float)gpsData.velE)*1E-3*1.94384, ((float)gpsData.velD)*1E-3*1.94384, gpsData.hour,gpsData.minute, gpsData.second, ((float)gpsData.gSpeed)*1E-3*1.94384, ((float)gpsData.headMot)*1E-5,
              accelData[0], accelData[1], accelData[2], gyroData[0], gyroData[1], gyroData[2], magData[0], magData[1], magData[2], attData[0], attData[1],attData[2], flightPathAngle);
    }
    else{
      usleep(1000);
    }
  }
  arduinoParserThread.join();
  gpsParserThread.join();
  fclose(rawSerialFile);
  fclose(datalogFile);
}
