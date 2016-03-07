#include <cstdlib>
#include <wiringPi.h>
#include <stdio.h>
#include<cmath>
#include <wiringPiI2C.h>
#include <time.h>

//Pressure Conversion Variables
int reading = 0;
int mask = 63; //(0 0 1 1 1 1 1 1 )
int maskstatus = 192; //(1 1 0 0 0 0 0 0 )

//Pressure Variables
float pfwdnoload, p45noload;
float pfwd, p45;
float pfwd_p45, Alpha; // pfwd_p45 is Pfwd/P45
float pfwdcorr, p45corr;
float A, B;


// I2C Handles and device addresses
int pressFD;
int imuFD;
int press_address = 0x28;
int imu_address = 0x29;

// Multiplexer Pin assignments
const int unit1 = 0;
const int unit2 = 1;

// Timing Variables
unsigned long cT ;
unsigned long eT ;
unsigned long pT ;
unsigned long pAttT;
unsigned long pAccelT;
unsigned long pGyroT;
unsigned long pMagT;
unsigned long pOutputT;
unsigned long pAlphaT;

// These periods determine rate at which data is read and output
unsigned int attPeriod = 10;
unsigned int accelPeriod = 10;
unsigned int gyroPeriod = 10;
unsigned int magPeriod = 10;
unsigned int outputPeriod = 10;
unsigned int alphaPeriod = 10;

// IMU Data
float gyroData [3];
float accelData [3];
float magData [3];
float attData [3];


//Get Roll Value from IMU
float imuGetRoll(){
  short reading = wiringPiI2CReadReg16(imuFD, 0x1C);
  // correct for msb glitches
  if (reading < -2880)
    reading += 0x8000;
  else if (reading > 2880)
    reading -= 0x8000;
  float roll = ((float)reading)/16.0;
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
    accelData[i] = ((float)reading)/16.0;
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

//Read the Differential pressure value from an MS4525 Pressure Sensor
float MS4525Getpressure(){
  int reading = wiringPiI2CRead(pressFD);
  reading = reading & mask;
  reading = reading << 8;
  reading |= wiringPiI2CRead(pressFD);
  return (float)reading;
}

//One time setup function
//Sets up GPIO pins, i2c bus, serial port
void setup(){
  // Initialize i2c bus
  wiringPiSetup();
  // Pressure Sensor I2C handle
  pressFD = wiringPiI2CSetup (press_address) ;
  // IMU I2C Handle
  imuFD = wiringPiI2CSetup (imu_address);
  // Set IMU to NDOF mode to start readings and fusion
  system("i2cset -y 1 0x29 0x3D 0x0C b");

  //activate multiplexer output pins and deactivate both pressure sensors
  pinMode (unit1, OUTPUT);
  pinMode (unit2, OUTPUT);
  digitalWrite( unit1, LOW);
  digitalWrite( unit2, LOW);

  A = -16.908083; // These values must be adjusted for individual
  B =  42.415008; // probes and probe locations

  pfwdnoload = 8162.9; // sensor bias (unit: counts
  p45noload  = 8186.6; // sensor bias (unit: counts)
  pT = 0;
}

//Forever Loop
void loop(){ // Main loop {
  // Calculate loop speed
  cT = millis();
  eT = cT - pT;
  pT = cT;
  float hz = 1000.0/eT;


  // reads alpha value every alphaPeriod ms
  if (cT - pAlphaT >= alphaPeriod)
  {
    pAlphaT = cT;

    // Activate Pressure sensor 1
    digitalWrite(unit1,HIGH );
    // Read its value
    float pfwd = MS4525Getpressure();
    // Deactivate it
    digitalWrite( unit1, LOW);
    // Correct pressure for sensor bias
    pfwdcorr = pfwd - pfwdnoload;

    // Activate Pressure sensor 2
    digitalWrite(unit2,HIGH );
    // Read it's value
    float p45 = MS4525Getpressure();
    // Deactivate it
    digitalWrite( unit2, LOW);
    // Correct pressure for sensor bias
    p45corr = p45 - p45noload;

    //Calculate angle of attack
    pfwd_p45 = pfwdcorr/p45corr;
    Alpha = A*(pfwd_p45) + B;

  }

  // reads attitude every attPeriod ms
  if(cT - pAttT >= attPeriod)
  {
    attData[0] = imuGetRoll();
    attData[1] = imuGetPitch();
    attData[2] = imuGetHeading();
    pAttT = cT;
  }

  // reads accelerometer every accelPeriod ms
  if(cT - pAccelT >= accelPeriod)
  {
    imuGetAccel();
    pAccelT = cT;
  }

  // reads gyroscope every gyroPeriod ms
  if(cT - pGyroT >= gyroPeriod)
  {
    imuGetGyro();
    pGyroT = cT;
  }

  // reads magnetometer ever magPeriod ms
  if(cT - pMagT >= magPeriod)
  {
    imuGetMag();
    pMagT = cT;
  }

  // outputs data every outputPeriod ms
  if (cT - pOutputT >= outputPeriod)
  {
    pOutputT = cT;
    // format: time alpha roll pitch yaw gx gy gz ax ay az mx my mz
    printf("\n%lu\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f",cT,Alpha,attData[0],attData[1],attData[2],gyroData[0],gyroData[1],gyroData[2],accelData[0],accelData[1],accelData[2],magData[0],magData[1],magData[2]);
  }
}


int main(void){
  // Setup Peripherals
  setup();
  // Run the loop forever
  while(true){
    loop();
  }
}
