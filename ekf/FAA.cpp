 // Name: Bar_and_Diff_pressure_V1_4_2.ino
// Version: 1_4_2
// Date: 29 July 2015
// Author: Francisco Rodriguez
// Calibration: Alpha2 Linear From DAB 5-26/27/2015

// Arduino 1.0+ Only

/* Based largely on code by Jim Lindblom

// Bar_and_Diff_pressure_V1_4_2

Get pressure, altitude, and temperature from the BMP085.
Serial.print it out at 9600 baud to serial monitor.
*/

#include <Wire.h>

#define BMP085_ADDRESS 0x77 // I2C address of BMP085

const unsigned char OSS = 0; // Oversampling Setting

// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
float pressurepsi;
float altitudefeet;
float temperaturec;
int ver = 142;


// Definitions for Delta Pressure sensor

int press_address = 40; //1001000 written as decimal number
int reading = 0;
int mask = 63; //(0 0 1 1 1 1 1 1 )
©2015 David F. Rogers. All rights reserved. 38
int maskstatus = 192; //(1 1 0 0 0 0 0 0 )
int Status;
float deltapressure;
float deltapressure1;
float deltapressure2;

// Definitions for Alpha Calculations

float pfwdnoload, p45noload;
float pfwd, p45;
float pfwd_p45, Alpha; // pfwd_p45 is Pfwd/P45
float pfwdcorr, p45corr, Dtarepfwd,Dtarep45,deltanoloadpfwd;
float deltanoloadp45, psi_count;
float A, B;

// Definitions for LED Control

int led13 = 13;
int num1 = 0;
int led = 1;

int unit = 1; // 1 = unit1 selected, 2 = unit2 selected

// b5 is calculated in bmp085GetTemperature(...)
// the b5 variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...)

long b5;

// Definition of output control

const int unit1 = 8;
const int unit2 = 9;


void setup(){ // Setup loop
	 Serial.begin(9600);
	 Wire.begin();

	 pinMode (unit1, OUTPUT); // set pin 8 control for sensor 1
	 pinMode (unit2, OUTPUT); // set pin 9 control for sensor 2
	 digitalWrite( unit1, LOW); // set both units off line
	 digitalWrite( unit2, LOW);

	 //----------------Calibration FOR ALPHA 2 ------------------

	 //-- ALPHA 2 Right Wing Flight Test Calibration Curve
	 //-- Linear from DAB 5-26/27-2015 Alpha = A*(Pfwd/P45) + B

	 A = -16.908083; // These values must be adjusted for individual
	 B = 42.415008; // probes and probe locations

	 pfwdnoload = 8162.9; //counts Adjust these values for
	 p45noload = 8186.6; //counts the individual sensors

	 //-------NOTE THE BMP085 IS NOT ACTUALLY NEEDED--------------
	 //-------THE SENSOR CAN BE REMOVED FROM THE DFRDAS-----------
	©2015 David F. Rogers. All rights reserved. 39
	 //-------ALONG WITH THE ASSOCIATED CODE----------------------

	 bmp085Calibration();
	 pinMode(led13, OUTPUT);
} // end setup loop

void loop() // Main loop
{
	 float temperature = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
	 float pressure = bmp085GetPressure(bmp085ReadUP());
	 float atm = pressure / 101325; // "Standard Atmosphere Sea Level Pressure"
	 float altitude = calcAltitude(pressure); // Uncompensated calculation - in Meters

	 // Various alternative possibilities for output from the BMP085

	 // Serial.print("Barometric Pressure = ");
	 pressurepsi = pressure * 0.000145037738; // convert from Pa to psi
	 altitudefeet = altitude * 3.280839895; // convert altitude from meters to feet
	 // Serial.print(pressurepsi, 6);
	 // Serial.println("psi");

	 // Serial.print("Barometric Pressure = ");
	 // pressurepsi = pressure * 0.000295; // convert from Pa to inHg
	 // Serial.print(pressurepsi,6);
	 // Serial.println(" inHg");

	 // Serial.print("Altitude = ");
	 // altitude = altitude * 3.2808; // convert from Pa to inHg
	 // Serial.print(altitude,2);
	 // Serial.println(" feet");
	 // -----------------------------------------------------------------------------------
	 // Serial.print(pressurepsi);

	 Serial.print(altitudefeet,2);
	 Serial.print(",");


//*******************THIS DOES NOTHING!!!*************************************************
	 if ( num1 > 5 ) // Discard initial 5 inputs for stability
	 {
		 led =-led;
		 num1 =0;

		 // Multiplex the two MS4525 pressure sensors

		 if (led < 0)
		 {
			digitalWrite(led13,HIGH);
		 }
		 else
		 {
			digitalWrite(led13,LOW);
		 }

	 }
//**************************************************************************************************




	 num1 =num1+1;

	 //-------------------------------------------------
	 // Get Differential Pressure from unit 1

	 digitalWrite(unit1,HIGH );//Allow communication to sensor 1
	 unit=1;
	 float pfwd = MS4525Getpressure(unit); // Get Differential Pressure from unit 1
	 digitalWrite( unit1, LOW);

	 //pfwd =7814.83; // TEST =========================

	 Serial.print( pfwd); // Print raw counts
	 Serial.print(",");

	 pfwdcorr = pfwd - pfwdnoload; // Account for no load

	 // Serial.print( pfwdcorr, 6); // Debuggign
	 // Serial.print(" , ");


	 //-------------------------------------------------
	 // Get Differential Pressure from unit 2
	 digitalWrite(unit2,HIGH );//Allow communication to sensor 2

	 unit=2;
	 float p45 = MS4525Getpressure(unit); // Get Differential Pressure from unit 2
	 digitalWrite( unit2, LOW);

	 //p45 =7926.78; // TEST============================

	 Serial.print( p45); // Print raw counts
	 Serial.print(",");

	 p45corr = p45 - p45noload; // Account for noload

	 // Serial.print( p45corr,6); // Debugging
	 // Serial.print(" , ");

	 digitalWrite(unit2,LOW);

	 Serial.flush(); // Clear serial port
	 // get values again.

	 // pfwdcorr=1; // TEST=============================
	 // p45corr=-2; // TEST=============================


	 pfwd_p45 = pfwdcorr/p45corr;

	 // Serial.print(pfwd_p45,10);
	 // Serial.print(" ------, ");

	 //----------------TEST------------------------------
	 // pfwd_p45 = 2.1;
	 //----------------END TEST--------------------------

	 Serial.print(pfwd_p45,5);
	 Serial.print(" , ");
	 
	 
	 //----- Calculate Alpha From Pfwd/P45 --------------

	 Alpha = A*(pfwd_p45) + B;

	 Serial.print(Alpha);
	 Serial.print(" , ");

	 Serial.print(pfwdnoload);

	 Serial.print(" , ");
	 Serial.print(p45noload);

	 Serial.print(" , V=");
	 Serial.println(ver);

	 // delay(1000); // Debugging
	 // Serial.print("\n");


} // End of main loop

// Stores all of the bmp085’s calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program

void bmp085Calibration()
{
	 ac1 = bmp085ReadInt(0xAA);
	 ac2 = bmp085ReadInt(0xAC);
	 ac3 = bmp085ReadInt(0xAE);
	 ac4 = bmp085ReadInt(0xB0);
	 ac5 = bmp085ReadInt(0xB2);
	 ac6 = bmp085ReadInt(0xB4);
	 b1 = bmp085ReadInt(0xB6);
	 b2 = bmp085ReadInt(0xB8);
	 mb = bmp085ReadInt(0xBA);
	 mc = bmp085ReadInt(0xBC);
	 md = bmp085ReadInt(0xBE);
}

// Calculate temperature in deg C

float bmp085GetTemperature(unsigned int ut){
	 long x1, x2;

	 x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
	 x2 = ((long)mc << 11)/(x1 + md);
	 b5 = x1 + x2;

	 float temp = ((b5 + 8)>>4);
	 temp = temp /10;

	 return temp;
}

// Calculate pressure retrieved.
// Calibration values must be known.
©2015 David F. Rogers. All rights reserved. 42
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned is pressure in units of Pa.

long bmp085GetPressure(unsigned long up){
	 long x1, x2, x3, b3, b6, p;
	 unsigned long b4, b7;

	 b6 = b5 - 4000;

	 // Calculate B3
	 x1 = (b2 * (b6 * b6)>>12)>>11;
	 x2 = (ac2 * b6)>>11;
	 x3 = x1 + x2;
	 b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

	 // Calculate B4
	 x1 = (ac3 * b6)>>13;
	 x2 = (b1 * ((b6 * b6)>>12))>>16;
	 x3 = ((x1 + x2) + 2)>>2;
	 b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

	 b7 = ((unsigned long)(up - b3) * (50000>>OSS));
	 if (b7 < 0x80000000)
		p = (b7<<1)/b4;
	 else
	 p = (b7/b4)<<1;

	 x1 = (p>>8) * (p>>8);
	 x1 = (x1 * 3038)>>16;
	 x2 = (-7357 * p)>>16;
	 p += (x1 + x2 + 3791)>>4;

	 long temp = p;
	 return temp;
}

// Read 1 byte from the BMP085 at ’address’

char bmp085Read(unsigned char address)
{
	 unsigned char data;

	 Wire.beginTransmission(BMP085_ADDRESS);
	 Wire.write(address);
	 Wire.endTransmission();

	 Wire.requestFrom(BMP085_ADDRESS, 1);
	 while(!Wire.available());

	 return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from ’address’
// Second byte will be from ’address’+1

©2015 David F. Rogers. All rights reserved. 43
int bmp085ReadInt(unsigned char address)
{
	 unsigned char msb, lsb;

	 Wire.beginTransmission(BMP085_ADDRESS);
	 Wire.write(address);
	 Wire.endTransmission();

	 Wire.requestFrom(BMP085_ADDRESS, 2);
	 while(Wire.available()<2);
	 msb = Wire.read();
	 lsb = Wire.read();

	 return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value

unsigned int bmp085ReadUT(){
	 unsigned int ut;

	 // Write 0x2E into Register 0xF4
	 // This requests a temperature reading
	 Wire.beginTransmission(BMP085_ADDRESS);
	 Wire.write(0xF4);
	 Wire.write(0x2E);
	 Wire.endTransmission();

	 // Wait at least 4.5ms
	 delay(5); // Wait 5 ms

	 // Read two bytes from registers 0xF6 and 0xF7
	 ut = bmp085ReadInt(0xF6);

	 return ut;
}

// Read the uncompensated pressure value

unsigned long bmp085ReadUP(){

	 unsigned char msb, lsb, xlsb;
	 unsigned long up = 0;

	 // Write 0x34+(OSS<<6) into register 0xF4
	 // Request a pressure reading w/ oversampling setting
	 Wire.beginTransmission(BMP085_ADDRESS);
	 Wire.write(0xF4);
	 Wire.write(0x34 + (OSS<<6));
	 Wire.endTransmission();

	 // Wait for conversion, delay time dependent on OSS
	 delay(2 + (3<<OSS));

	 // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
	 msb = bmp085Read(0xF6);
©2015 David F. Rogers. All rights reserved. 44
	 lsb = bmp085Read(0xF7);
	 xlsb = bmp085Read(0xF8);

	 up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
	 //Serial.print(up);
	 //Serial.print(" , ");
	 return up;
}

void writeRegister(int deviceAddress, byte address, byte val) {
	 Wire.beginTransmission(deviceAddress); // start transmission to device
	 Wire.write(address); // send register address
	 Wire.write(val); // send value to write
	 Wire.endTransmission(); // end transmission
}

int readRegister(int deviceAddress, byte address){

	 int v;
	 Wire.beginTransmission(deviceAddress);
	 Wire.write(address); // register to read
	 Wire.endTransmission();

	 Wire.requestFrom(deviceAddress, 1); // read a byte

	 while(!Wire.available()) {
		// waiting
	 }

	 v = Wire.read();
	 return v;
	 }

float calcAltitude(float pressure){

	 float A = pressure/101325;
	 float B = 1/5.25588;
	 float C = pow(A,B);
	 C = 1 - C;
	 C = C /0.0000225577;

	 return C;
}

// Get Delta Pressure from MS4525 ---------------------------------------

//Reads Differential Pressure from I2C Ms4525 sensor
//
float MS4525Getpressure(int unit){
	 deltapressure =0;

	 //Send a request
	 //Start talking to the device at the specified address
	 Wire.beginTransmission(press_address);
	 //Send a bit asking for register zero, the data register
	 Wire.write(0);
	 //Complete Transmission
©2015 David F. Rogers. All rights reserved. 45
	 Wire.endTransmission();

	 //Request 2 Byte from the specified address
	 Wire.requestFrom(press_address, 2);
	 //wait for response for 2 bytes

	 if(2 <=Wire.available())
	 {
		 reading = Wire.read(); // byte 1
		 //Status = reading & maskstatus; // check status
		 //Status = Status >>6;
		 //Serial.println(Status);

		 //if ( Status <= 0)
		 //{
		 reading = reading & mask;

		 reading = reading << 8; //

		 reading |= Wire.read(); // read byte 2
		 //Serial.print(reading);
		 //Serial.print(",");
		 deltapressure =reading;
		 //deltapressure = deltapressure/16383 - .5;

		 // Serial.println(pressure,4);
		 // }
	 }
	 //delay(50);
	 return(deltapressure);
}
