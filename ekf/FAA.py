# Name: Bar_and_Diff_pressure_V1_4_2.ino
# Version: 1_4_2
# Date: 29 July 2015
# Author: Francisco Rodriguez
# Calibration: Alpha2 Linear From DAB 5-26/27/2015

# Arduino 1.0+ Only

# Based largely on code by Jim Lindblom

# Bar_and_Diff_pressure_V1_4_2

#Get pressure, altitude, and temperature from the BMP085.
#Serial.prit out at 9600 baud to serial monitor.
#*/

import smbus
import time
testing = True

BMP085_ADDRESS = 0x77 # I2C address of BMP085
if not testing:
	bus1 = smbus.SMBus(1)
	bus0 = smbus.SMBus(0)
	import RPi.GPIO as GPIO

pT = 0
cT = 0
eT = 0
ac1 = 0
ac2 = 0
ac3 = 0
ac4 = 10
ac5 = 0
ac6 = 0
b1 = 0
b2 = 0
b3 = 0
b4 = 0
b5 = 0
mb = 0
mc = 0
md = 5


OSS = 0 # Oversampling Setting

# Calibration values
pressurepsi = 0.0
altitudefeet = 0.0
temperaturec = 0.0
ver = 142


# Definitions for Delta Pressure sensor

press_address = 40 #1001000 written as decimal number
reading = 0
mask = 63 #(0 0 1 1 1 1 1 1 )
maskstatus = 192 #(1 1 0 0 0 0 0 0 )

# Definitions for LED Control

ledPin = 13
num1 = 0
led = 1

unit = 1 # 1 = unit1 selected, 2 = unit2 selected

# b5 is calculated in bmp085GetTemperature(...)
# the b5 variable is also used in bmp085GetPressure(...)
# so ...Temperature(...) must be called before ...Pressure(...)

# Definition of output control

unit1 = 8
unit2 = 9


A = -16.908083 # These values must be adjusted for individual
B = 42.415008 # probes and probe locations
pfwdnoload = 8162.9 #counts Adjust these values for
p45noload = 8186.6 #counts the individual sensors


def setup(): # Setup loop
	if not testing:
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(unit1, GPIO.OUT)
		GPIO.setup(unit2, GPIO.OUT)
		GPIO.setup(ledPin, GPIO.OUT)
		GPIO.output( unit1, GPIO.LOW) # set both units off line
		GPIO.output( unit2, GPIO.LOW)

	#----------------Calibration FOR ALPHA 2 ------------------

	#-- ALPHA 2 Right Wing Flight Test Calibration Curve
	#-- Linear from DAB 5-26/27-2015 Alpha = A*(Pfwd/P45) + B

	#-------NOTE THE BMP085 IS NOT ACTUALLY NEEDED--------------
	#-------THE SENSOR CAN BE REMOVED FROM THE DFRDAS-----------
	#-------ALONG WITH THE ASSOCIATED CODE----------------------

	bmp085Calibration()
# end setup loop

def loop(): # Main loop
	temperature = bmp085GetTemperature(bmp085ReadUT()) #MUST be called first
	pressure = bmp085GetPressure(bmp085ReadUP())
	atm = pressure / 101325 # "Standard Atmosphere Sea Level Pressure"
	altitude = calcAltitude(pressure) # Uncompensated calculation - in Meters

	pressurepsi = pressure * 0.000145037738 # convert from Pa to psi
	altitudefeet = altitude * 3.280839895 # convert altitude from meters to feet

	global num1
	global led
#*******************THIS DOES NOTHING!!!*************************************************
	if ( num1 > 5 ): # Discard initial 5 inputs for stability
		led =-led
		num1 =0

		# Multiplex the two MS4525 pressure sensors

		if (led < 0 and not testing):
			GPIO.output(led13,GPIO.HIGH)
		elif not testing:
			GPIO.output(led13,GPIO.LOW)

#**************************************************************************************************




	num1 =num1+1

	#-------------------------------------------------
	# Get Differential Pressure from unit 1
	if not testing:
		GPIO.output(unit1,GPIO.HIGH )#AlGPIO.LOW communication to sensor 1
	unit=1
	pfwd = MS4525Getpressure(unit) # Get Differential Pressure from unit 1
	if not testing:
		GPIO.output( unit1, GPIO.LOW)

	#pfwd =7814.83 # TEST =========================

	pfwdcorr = pfwd - pfwdnoload # Account for no load


	#-------------------------------------------------
	# Get Differential Pressure from unit 2
	if not testing:
		GPIO.output(unit2,GPIO.HIGH )#AlGPIO.LOW communication to sensor 2

	unit=2
	p45 = MS4525Getpressure(unit) # Get Differential Pressure from unit 2
	if not testing:
		GPIO.output( unit2, GPIO.LOW)

	#p45 =7926.78 # TEST============================

	p45corr = p45 - p45noload # Account for noload

	if not testing:
		GPIO.output(unit2,GPIO.LOW)

	# get values again.

	# pfwdcorr=1 # TEST=============================
	# p45corr=-2 # TEST=============================


	pfwd_p45 = pfwdcorr/p45corr

	#----------------TEST------------------------------
	# pfwd_p45 = 2.1
	#----------------END TEST--------------------------	
	
	#----- Calculate Alpha From Pfwd/P45 --------------

	Alpha = A*(pfwd_p45) + B
	global cT
	global eT
	global pT
	cT = time.time()
	eT = cT - pT
	pT = cT
	print(str(altitudefeet) + ' , ' +
	      str(pfwd) + ' , ' +
	      str(p45) + ' , ' +
	      str(pfwd_p45) + ' , ' +
	      str(Alpha) + ' , ' +
	      str(pfwdnoload) + ' , ' +
	      str(p45noload) + ' , ' +
	      str(ver), ' , ' +
	      str(1/eT))
	      

	# delay(1000) # Debugging
	# print("\n")

 # End of main loop

# Stores all of the bmp085s calibration values into global variables
# Calibration values are required to calculate temp and pressure
# This function should be called at the beginning of the program

def bmp085Calibration():
	ac1 = bmp085ReadInt(0xAA)
	ac2 = bmp085ReadInt(0xAC)
	ac3 = bmp085ReadInt(0xAE)
	ac4 = bmp085ReadInt(0xB0)
	ac5 = bmp085ReadInt(0xB2)
	ac6 = bmp085ReadInt(0xB4)
	b1 = bmp085ReadInt(0xB6)
	b2 = bmp085ReadInt(0xB8)
	mb = bmp085ReadInt(0xBA)
	mc = bmp085ReadInt(0xBC)
	md = bmp085ReadInt(0xBE)


# Calculate temperature in deg C

def bmp085GetTemperature(ut):
	x1 = ((ut - ac6)*ac5) >> 15
	x2 = (mc << 11)/(x1 + md)
	b5 = x1 + x2

	temp = ((b5 + 8)>>4)
	temp = temp /10

	return temp

# Calculate pressure retrieved.
# Calibration values must be known.
# b5 is also required so bmp085GetTemperature(...) must be called first.
# Value returned is pressure in units of Pa.

def bmp085GetPressure(up):
	b6 = b5 - 4000

	# Calculate B3
	x1 = (b2 * (b6 * b6)>>12)>>11
	x2 = (ac2 * b6)>>11
	x3 = x1 + x2
	b3 = ((((ac1)*4 + x3)<<OSS) + 2)>>2

	# Calculate B4
	x1 = (ac3 * b6)>>13
	x2 = (b1 * ((b6 * b6)>>12))>>16
	x3 = ((x1 + x2) + 2)>>2
	b4 = (ac4 * (x3 + 32768))>>15

	b7 = ((up - b3) * (50000>>OSS))
	if (b7 < 0x80000000):
		p = (b7<<1)/b4
	else:
		p = (b7/b4)<<1

	x1 = (p>>8) * (p>>8)
	x1 = (x1 * 3038)>>16
	x2 = (-7357 * p)>>16
	p += (x1 + x2 + 3791)>>4

	temp = p
	return temp

# Read 1 byte from the BMP085 at 'address'

def bmp085Read(address):
	if not testing:
		return bus1.read_byte_data(BMP085_ADDRESS,address)
	else:
		return 14

# Read 2 bytes from the BMP085
# First byte will be from 'address'
# Second byte will be from 'address'+1

def bmp085ReadInt(address):
	if not testing:
		msb = bus1.read_byte_data(BMP085_ADDRESS,address)
		lsb = bus1.read_byte_data(BMP085_ADDRESS,address+1)
		return msb<<8 | lsb
	else:
		return 143

# Read the uncompensated temperature value

def bmp085ReadUT():
	# Write 0x2E into Register 0xF4
	# This requests a temperature reading
	if not testing:
		bus1.write(BMP085_ADDRESS,0xF4)
		bus1.write(BMP085_ADDRESS,0x2E)

	# Read two bytes from registers 0xF6 and 0xF7
		ut = bmp085ReadInt(0xF6)
	else:
		ut = 32
	return ut

# Read the uncompensated pressure value

def bmp085ReadUP():
	up = 0

	# Write 0x34+(OSS<<6) into register 0xF4
	# Request a pressure reading w/ oversampling setting
	if not testing:
		bus1.write(BMP085_ADDRESS,0xF4)
		bus1.write(BMP085_ADDRESS,0x34 + (OSS<<6))

	# Wait for conversion, delay time dependent on OSS

	# Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
		msb = bmp085Read(0xF6)
		lsb = bmp085Read(0xF7)
		xlsb = bmp085Read(0xF8)
	else:
		msb = 62
		lsb = 94
		xlsb = 12

	up = ((msb << 16) | (lsb << 8) | xlsb) >> (8-OSS)
	return up

def calcAltitude( pressure):
	A = pressure/101325
	B = 1/5.25588
	C = pow(A,B)
	C = 1 - C
	C = C /0.0000225577

	return C

# Get Delta Pressure from MS4525 ---------------------------------------

#Reads Differential Pressure from I2C Ms4525 sensor
#
def MS4525Getpressure(unit):
	if not testing:
		deltapressure = bus1.read_word_data(press_address,0)
	else:
		deltapressure = 5678
	return(deltapressure)



setup()
while True:
	loop()
