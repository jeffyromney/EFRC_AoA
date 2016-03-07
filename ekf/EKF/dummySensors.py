
import socket
import time
import json
from math import sin

TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 20  # Normally 1024, but we want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

conn, addr = s.accept()
print 'Connection address:', addr
startTime = time.time()
pIT = startTime
pGT = startTime
while 1:
	sysTime = time.time() - startTime
	cT = time.time()
	# print('Previous: ',pIT, 'Current: ',cT,'diff',cT-pIT)
	if cT - pIT >= 0.01:
		pIT = cT
		#sendIMU 
		# timestamp seconds 
		# timestamp ms
		# ang rate x,y,z
		# accel x,y,z
		IMUms = float(sysTime*1000)
		IMUs = float(sysTime)
		IMUangx = sin(IMUs + 0.1) * 20
		IMUangy = sin(IMUs + 0.2) * 20
		IMUangz = sin(IMUs + 0.3) * 180
		IMUaccx = sin(IMUs + 0.4) * 2
		IMUaccy = sin(IMUs + 0.5) * 2
		IMUaccz = sin(IMUs + 0.6) * 2
		IMUoutStr = '$IMU' + str(IMUs) + ',' + str(IMUms) + ',' + str(IMUangx) + ',' + str(IMUangy) + ',' + str(IMUangz) + ',' + str(IMUaccx) + ',' + str(IMUaccy) + ',' +  str(IMUaccz)
		print(IMUoutStr)
		conn.send(IMUoutStr)
	if cT - pGT >= 0.1:
		pGT = cT
		#Send GPS 
		GPStimestamp = sysTime
		GPSstatus = 3
		GPSms = sysTime*1000
		GPSLat = (sin(GPStimestamp + 0.1) * .01) + 27
		GPSLon = (sin(GPStimestamp + 0.3) * .01) - 81
		GPSAlt = (sin(GPStimestamp + 0.5) * 10) + 300
		GPSgndspd = (sin(GPStimestamp + 0.6) * 5) + 15
		GPSCourse = sin(GPStimestamp + 0.3) * 195
		GPSVeld = sin(GPStimestamp + 0.7) 
		GPSoutStr = '$GPS' + str(GPStimestamp) + ',' + str(GPSstatus) + ',' + str(GPSms) + ',' + str(GPSLat) + ',' + str(GPSLon) + ',' + str(GPSAlt) + ',' + str(GPSgndspd) + ',' + str(GPSCourse) + ',' + str(GPSVeld)
		print(GPSoutStr)
		conn.send(GPSoutStr)

conn.close()
