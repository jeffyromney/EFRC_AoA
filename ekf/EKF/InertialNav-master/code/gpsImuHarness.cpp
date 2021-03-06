#include "estimator_22states.cpp"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <string>
#include <unistd.h>
#include <iostream>
#include <thread> // for multi-threading
#include <wiringPi.h> //for GPIO functions
#include <wiringPiI2C.h> //for I2C Functions
#include <queue>   // for std::queue
#include <termios.h> //  for comport config
#include <fcntl.h> //stuff for comports
#include <csignal>



//#include <stdlib.h>
//#include <cstdlib> // for system() call



#define GRAVITY_MSS 9.80665f
#define deg2rad 0.017453292f
#define rad2deg 57.295780f
#define pi 3.141592657f






float windowSize = 15.0;
int filterIndex = 0;

float sum0 = 0;
float sum1 = 0;
float sum2 = 0;

float window0[15];
float window1[15];
float window2[15];

float filtered0, filtered1, filtered2;






//Function Prototypes
void readIMUData();
void readGpsData();
void readMagData();
void readAhrsData();
void readTimingData();
void gpsParser();
float ConstrainFloat(float val, float min, float max);


// Estimated time delays (msec)
uint32_t msecVelDelay = 230;
uint32_t msecPosDelay = 210;
uint32_t msecHgtDelay = 350;
uint32_t msecMagDelay = 30;

// IMU input data variables
Vector3f lastAngRate;
Vector3f lastAccel;

// Magnetometer input data variables
bool newDataMag = false;

// AHRS input data variables
float ahrsEul[3];
float eulerEst[3]; // Euler angles calculated from filter states
float eulerDif[3]; // difference between Euler angle estimated by EKF and the AHRS solution

// input data timing
uint64_t msecAlignTime = 100;
uint64_t msecStartTime = 0;
uint64_t msecEndTime = 0;

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
	float velD;
	int status;
};
// Queue to hold incoming gpsData.
std::queue<gpsDataStruct> gpsInputQueue;

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
uint8_t msgID = 0;
uint8_t msgClass = 0;
uint16_t msgLength = 0;
uint16_t msgChecksum = 0;

int gpsFD = 0;
int gpsNumRead = 0;
int gpsLatestRead = 0;



// Timing Variables
unsigned long cT = 0;
unsigned long eT = 0;
unsigned long pT = 0;
unsigned long pAttT = 0;
unsigned long pAccelT = 0;
unsigned long pGyroT = 0;
unsigned long pMagT = 0;
unsigned long pOutputT = 0;
unsigned long pAlphaT = 0;
unsigned long pImuT = 0;
unsigned long pGpsT = 0;
unsigned long pAhrsT = 0;
unsigned int attPeriod = 10;
unsigned int accelPeriod = 10;
unsigned int gyroPeriod = 10;
unsigned int magPeriod = 100;
unsigned int outputPeriod = 10;
unsigned int alphaPeriod = 10;
unsigned int imuPeriod = 10;
unsigned int gpsPeriod = 200;
unsigned int ahrsPeriod = 10;


// I2C Handles and device addresses
int pressFD = 0;
int imuFD = 0;
int press_address = 0x28;
int imu_address = 0x29;
int triggered = 0;

//Used for Dummy input
std::string inStr = std::string();

//EKF object
AttPosEKF   *_ekf;

void signalHandler( int signum )
{
	triggered = signum;
	printf("\n\n%d\n\n",signum);
}

int main(int argc, char *argv[]){
	for (int i=1;i<30;i++){
		if (i != 2)
			signal (i, signalHandler);
	}
	imuFD = wiringPiI2CSetup (imu_address);

	wiringPiI2CWriteReg8(imuFD,0x3D,00); // set IMU to CONFIG operation mode
	wiringPiI2CWriteReg8(imuFD,0x3B,00); // set Units
	wiringPiI2CWriteReg8(imuFD,0x3D,11); // set IMU to NDOF_FMC_OFF operation mode

	gpsFD = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
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

	// Launch GPS Parser Thread
	std::thread gpsParserThread(gpsParser);

    // Instantiate EKF
    _ekf = new AttPosEKF();

    printf("Filter start\n");

    readTimingData();

    float dt = 0.0f; // time lapsed since last covariance prediction

	_ekf->useCompass = true;
	_ekf->useGPS = true;
    _ekf->useAirspeed = false;
    _ekf->useRangeFinder = false;
    _ekf->useOpticalFlow = false;

	//forever loop
    while (true) {

		if(triggered > 0){
			printf("\n\n SIGNAL TRIGGERED: %d\n\n",triggered);
			triggered = 0;
			break;
		}
		// Set Current Time
		cT = millis();
        pT = cT;

		//Read in Sensor Data
		readIMUData();
		readGpsData();
		readMagData();
		readAhrsData();

        // Apply dtIMU
        _ekf->dtIMU     = 0.001f*(cT - pT);

		// Initialise states, covariance and other data
		if ((cT > msecAlignTime) && !_ekf->statesInitialised && (_ekf->GPSstatus >= 3)){
			calcvelNED(_ekf->velNED, gpsCourse, gpsGndSpd, gpsVelD);

			_ekf->InitialiseFilter(_ekf->velNED, _ekf->gpsLat, _ekf->gpsLon, _ekf->gpsHgt, 0.0f);

		} else if (_ekf->statesInitialised) {

			// Run the strapdown INS equations every IMU update
			_ekf->UpdateStrapdownEquationsNED();
			#if 1
			// debug code - could be turned into a filter monitoring/watchdog function
			float tempQuat[4];
			for (uint8_t j=0; j<4; j++) tempQuat[j] = _ekf->states[j];
			_ekf->quat2eul(eulerEst, tempQuat);
			for (uint8_t j=0; j<=2; j++) eulerDif[j] = eulerEst[j] - ahrsEul[j];
			if (eulerDif[2] > M_PI) eulerDif[2] -= 2 * M_PI;
			if (eulerDif[2] < -M_PI) eulerDif[2] += 2 * M_PI;
			#endif
			// store the predicted states for substrequent use by measurement fusion
			_ekf->StoreStates(cT);
			// Check if on ground - status is used by covariance prediction
			bool onground = (((AttPosEKF::sq(_ekf->velNED[0]) + AttPosEKF::sq(_ekf->velNED[1]) + AttPosEKF::sq(_ekf->velNED[2])) < 4.0f) && (_ekf->VtasMeas < 8.0f));

			_ekf->setOnGround(onground);
			// sum delta angles and time used by covariance prediction
			_ekf->summedDelAng = _ekf->summedDelAng + _ekf->correctedDelAng;
			_ekf->summedDelVel = _ekf->summedDelVel + _ekf->dVelIMU;
			dt += _ekf->dtIMU;
			// perform a covariance prediction if the total delta angle has exceeded the limit
			// or the time limit will be exceeded at the next IMU update
			if ((dt >= (_ekf->covTimeStepMax - _ekf->dtIMU)) || (_ekf->summedDelAng.length() > _ekf->covDelAngMax))
			{
				_ekf->CovariancePrediction(dt);
				_ekf->summedDelAng.zero();
				_ekf->summedDelVel.zero();
				dt = 0.0f;
			}

			// Set global time stamp used by EKF processes
			_ekf->globalTimeStamp_ms = cT;
		}

		// Fuse GPS Measurements
		if (newDataGps){
			// printf("\nBLAH4\n");
			// calculate a position offset which is applied to NE position and velocity wherever it is used throughout code to allow GPS position jumps to be accommodated gradually
			_ekf->decayGpsOffset();

			// Convert GPS measurements to Pos NE, hgt and Vel NED
			calcvelNED(_ekf->velNED, gpsCourse, gpsGndSpd, gpsVelD);
			calcposNED(posNED, _ekf->gpsLat, _ekf->gpsLon, _ekf->gpsHgt, _ekf->latRef, _ekf->lonRef, _ekf->hgtRef);

			_ekf->hgtMea = _ekf->gpsHgt - _ekf->hgtRef;
			_ekf->posNE[0] = posNED[0];
			_ekf->posNE[1] = posNED[1];
			 // set fusion flags
			_ekf->fuseVelData = true;
			_ekf->fusePosData = true;
			_ekf->fuseHgtData = true;
			// recall states stored at time of measurement after adjusting for delays
			_ekf->RecallStates(_ekf->statesAtVelTime, (cT - msecVelDelay));
			_ekf->RecallStates(_ekf->statesAtPosTime, (cT - msecPosDelay));
			_ekf->RecallStates(_ekf->statesAtHgtTime, (cT - msecHgtDelay));
			// run the fusion step
			_ekf->FuseVelposNED();
			//printf("FuseVelposNED at time = %lu \n", cT);
		}
		else{
			// printf("\nBLAH5\n");
			_ekf->fuseVelData = false;
			_ekf->fusePosData = false;
			_ekf->fuseHgtData = false;
		}
		calcposNED(posNED, _ekf->gpsLat, _ekf->gpsLon, _ekf->gpsHgt, _ekf->latRef, _ekf->lonRef, _ekf->hgtRef);

		_ekf->posNE[0] = posNED[0];
		_ekf->posNE[1] = posNED[1];

		// fuse GPS
		if (_ekf->useGPS && cT < 1000){
			// printf("\nBLAH1\n");
			_ekf->fuseVelData = true;
			_ekf->fusePosData = true;
			_ekf->fuseHgtData = false;
			// recall states stored at time of measurement after adjusting for delays
			_ekf->RecallStates(_ekf->statesAtVelTime, (cT - msecVelDelay));
			_ekf->RecallStates(_ekf->statesAtPosTime, (cT - msecPosDelay));
			// record the last fix time
			_ekf->lastFixTime_ms = cT;
			// run the fusion step
			_ekf->FuseVelposNED();
		} else {
			// printf("\nBLAH2\n");
			_ekf->fuseVelData = false;
			_ekf->fusePosData = false;
			_ekf->fuseHgtData = false;
		}

		// Fuse Magnetometer Measurements
		if (newDataMag && _ekf->statesInitialised && _ekf->useCompass){
			_ekf->fuseMagData = true;
			_ekf->RecallStates(_ekf->statesAtMagMeasTime, (cT - msecMagDelay)); // Assume 50 msec avg delay for magnetometer data
			_ekf->magstate.obsIndex = 0;
			_ekf->FuseMagnetometer();
			_ekf->FuseMagnetometer();
			_ekf->FuseMagnetometer();
		}
		else{
			_ekf->fuseMagData = false;
		}

		struct ekf_status_report ekf_report;

		//CHECK IF THE INPUT DATA IS SANE
		int check = _ekf->CheckAndBound(&ekf_report);

		switch (check){
			case 0:
				/* all ok */
				//printf("all good:\t%f\n",eulerEst[0]);
				break;
			case 1:
			{
				printf("NaN in states, resetting\n");
				printf("fail states: ");
				for (unsigned i = 0; i < ekf_report.n_states; i++) {
					printf("%f ",ekf_report.states[i]);
				}
				printf("\n");

				printf("states after reset: ");
				for (unsigned i = 0; i < ekf_report.n_states; i++) {
					printf("%f ",_ekf->states[i]);
				}
				printf("\n");
				break;
			}
			case 2:
			{
				printf("stale IMU data, resetting\n");
				break;
			}
			case 3:
			{
				printf("switching to dynamic state\n");
				break;
			}
			case 4:
			{
				printf("excessive gyro offsets\n");
				break;
			}
			case 5:
			{
				printf("GPS velocity diversion\n");
				break;
			}
			case 6:
			{
				printf("Excessive covariances\n");
				break;
			}


			default:
			{
				printf("unknown reset condition\n");
			}
		}

		//There is an error: reset
		if (check){
			printf("RESET OCCURED AT %d milliseconds\n", (int)cT);

			if (!ekf_report.velHealth || !ekf_report.posHealth || !ekf_report.hgtHealth || ekf_report.gyroOffsetsExcessive) {
			printf("health: VEL:%s POS:%s HGT:%s OFFS:%s\n",
				((ekf_report.velHealth) ? "OK" : "ERR"),
				((ekf_report.posHealth) ? "OK" : "ERR"),
				((ekf_report.hgtHealth) ? "OK" : "ERR"),
				((!ekf_report.gyroOffsetsExcessive) ? "OK" : "ERR"));
			}

			if (ekf_report.velTimeout || ekf_report.posTimeout || ekf_report.hgtTimeout || ekf_report.imuTimeout) {
				printf("timeout: %s%s%s%s\n",
					((ekf_report.velTimeout) ? "VEL " : ""),
					((ekf_report.posTimeout) ? "POS " : ""),
					((ekf_report.hgtTimeout) ? "HGT " : ""),
					((ekf_report.imuTimeout) ? "IMU " : ""));
			}
		}

		// debug output
		//printf("Euler Angle Difference = %3.1f , %3.1f , %3.1f deg\n", rad2deg*eulerDif[0],rad2deg*eulerDif[1],rad2deg*eulerDif[2]);
		if (cT - pOutputT >= outputPeriod){
			pOutputT = cT;
			//printf("\naccx3, gyrx3, magx3: %8.4f, %8.4f, %8.4f \t %8.4f, %8.4f, %8.4f \t %8.4f, %8.4f, %8.4f", _ekf->accel.x, _ekf->accel.y, _ekf->accel.z, _ekf->angRate.x, _ekf->angRate.y, _ekf->angRate.z, _ekf->magData.x, _ekf->magData.y, _ekf->magData.z);

			//printf("\nvelx3, posx3, eulx3, times: %8.4f, %8.4f, %8.4f, \t %8.4f, %8.4f, %8.4f, \t %8.4f, %8.4f, %8.4f, \t %8.4f, %lu", (double)_ekf->states[4], (double)_ekf->states[5], (double)_ekf->states[6], (double)_ekf->states[7], (double)_ekf->states[8], (double)_ekf->states[9], eulerEst[0]*rad2deg, eulerEst[1]*rad2deg, eulerEst[2]*rad2deg, _ekf->dtIMU, cT);
			filterIndex = (filterIndex + 1)%15;

			sum0 -= window0[filterIndex];
			window0[filterIndex] = _ekf->angRate.x;
			sum0 += window0[filterIndex];
			filtered0 = sum0 / windowSize;

			sum1 -= window1[filterIndex];
			window1[filterIndex] =  _ekf->angRate.y;
			sum1 += window1[filterIndex];
			filtered1 = sum1 / windowSize;

			sum2 -= window2[filterIndex];
			window2[filterIndex] =  _ekf->angRate.z;
			sum2 += window2[filterIndex];
			filtered2 = sum2 / windowSize;

			printf("\nvel,pos,eul,t,gps,ahrs: %8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%lu,%f,%f,% 8.4f,% 8.4f,% 8.4f",
				(double)_ekf->velNED[0], (double)_ekf->velNED[1], (double)_ekf->velNED[2],
				(double)_ekf->posNE[0], (double)_ekf->posNE[1], (double)_ekf->hgtMea,
				eulerEst[0]*rad2deg, eulerEst[1]*rad2deg, eulerEst[2]*rad2deg,
				_ekf->dtIMU, cT,
				_ekf->gpsLat*rad2deg, _ekf->gpsLon*rad2deg,
				ahrsEul[0], ahrsEul[1], ahrsEul[2]
				//filtered0, filtered1, filtered2,
				//(double)_ekf->states[10], (double)_ekf->states[11], (double)_ekf->states[12]
				);


			//* Output Info and code commented out
			// State vector:
			// 0-3: quaternions (q0, q1, q2, q3)
			// 4-6: Velocity - m/sec (North, East, Down)
			// 7-9: Position - m (North, East, Down)
			// 10-12: Delta Angle bias - rad (X,Y,Z)
			// 13: Delta Velocity Z bias -m/s
			// 14-15: Wind Vector  - m/sec (North,East)
			// 16-18: Earth Magnetic Field Vector - milligauss (North, East, Down)
			// 19-21: Body Magnetic Field Vector - milligauss (X,Y,Z)
			// 22: Terrain Vertical Offset - m

			// printf("\n");
			// printf("dtIMU: %8.6f, dt: %8.6f, cT: %lu\n", _ekf->dtIMU, dt, cT);
			// printf("posNED: %8.4f, %8.4f, %8.4f, velNED: %8.4f, %8.4f, %8.4f\n", (double)_ekf->posNE[0], (double)_ekf->posNE[1], (double)_ekf->hgtMea,
				// (double)_ekf->velNED[0], (double)_ekf->velNED[1], (double)_ekf->velNED[2]);
			// printf("vTAS: %8.4f baro alt: %8.4f\n", _ekf->VtasMeas, _ekf->hgtMea);
			// printf("mag: %8.4f, %8.4f, %8.4f\n", (double)_ekf->magData.x, (double)_ekf->magData.y, (double)_ekf->magData.z);
			// printf("states (quat)        [1-4]: %8.4f, %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[0], (double)_ekf->states[1], (double)_ekf->states[2], (double)_ekf->states[3]);
			// printf("states (vel m/s)     [5-7]: %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[4], (double)_ekf->states[5], (double)_ekf->states[6]);
			// printf("states (pos m)      [8-10]: %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[7], (double)_ekf->states[8], (double)_ekf->states[9]);
			// printf("states (delta ang) [11-13]: %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[10], (double)_ekf->states[11], (double)_ekf->states[12]);
			// printf("states (delta vel) [14]: %8.4ff\n", (double)_ekf->states[13]);
			// printf("states (wind)      [15-16]: %8.4f, %8.4f\n", (double)_ekf->states[14], (double)_ekf->states[15]);
			// printf("states (earth mag) [17-19]: %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[16], (double)_ekf->states[17], (double)_ekf->states[18]);
			// printf("states (body mag)  [20-22]: %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[19], (double)_ekf->states[20], (double)_ekf->states[21]);
			// printf("states (terain offset) [23]: %8.4ff\n", (double)_ekf->states[22]);
			// printf("states: %s %s %s %s %s %s %s %s\n",
				// (_ekf->statesInitialised) ? "INITIALIZED" : "NON_INIT",
				// //(_ekf->_onGround) ? "ON_GROUND" : "AIRBORNE",
				// (_ekf->fuseVelData) ? "FUSE_VEL" : " INH_VEL",
				// (_ekf->fusePosData) ? "FUSE_POS" : " INH_POS",
				// (_ekf->fuseHgtData) ? "FUSE_HGT" : " INH_HGT",
				// (_ekf->fuseMagData) ? "FUSE_MAG" : " INH_MAG",
				// (_ekf->fuseVtasData) ? "FUSE_VTAS" : " INH_VTAS",
				// (_ekf->useAirspeed) ? "USE_AIRSPD" : "IGN_AIRSPD",
				// (_ekf->useCompass) ? "USE_COMPASS" : "IGN_COMPASS");
		}

		if(triggered > 0){
			printf("\n\n SIGNAL TRIGGERED: %d\n\n",triggered);
			triggered = 0;
			break;
		}
	}

    delete _ekf;
	gpsParserThread.join();
}

void readIMUData(){
	while(cT - pImuT <= imuPeriod){
		cT = millis();
                usleep(1000);
	}
	float accelData [3];
	float gyroData [3];

	//Read Accel
	int baseAddr = 0x08;
	for (int i=0;i<3;i++){
		short reading = wiringPiI2CReadReg16(imuFD,(baseAddr + (i*2)));
		//reading = ((reading & 0x00ff) << 8) | ((reading & 0xff00)>>8);
		// correct for msb glitches
		if (reading < -16500)
			reading += 0x8000;
		else if (reading > 16500)
			reading -= 0x8000;
		accelData[i] = ((float)reading)/100.0;///16.0;
	}

	//Read Gyro
	baseAddr = 0x14;
	for (int i=0;i<3;i++){
		short reading = wiringPiI2CReadReg16(imuFD,(baseAddr + (i*2)));
		//correct for msb glitches
		if (reading < -16500)
			reading += 0x8000;
		else if (reading > 16500)
			reading -= 0x8000;
		gyroData[i] = ((float)reading)/16.0;
	}
	//printf(", %lu",pImuT);
	_ekf->dtIMU     = 0.001f*(cT - pImuT);
	_ekf->angRate.x = gyroData[0]*deg2rad;
	_ekf->angRate.y = gyroData[1]*deg2rad;
	_ekf->angRate.z = gyroData[2]*deg2rad;
	_ekf->accel.x   = accelData[0];
	_ekf->accel.y   = accelData[1];
	_ekf->accel.z   = accelData[2];
	_ekf->dAngIMU = 0.5f*(_ekf->angRate + lastAngRate)*_ekf->dtIMU;
	lastAngRate = _ekf->angRate;
	_ekf->dVelIMU = 0.5f*(_ekf->accel + lastAccel)*_ekf->dtIMU;
	lastAccel = _ekf->accel;
	pImuT = cT;
}


void readGpsData(){
	if(gpsInputQueue.empty())
		newDataGps = false;
		// printf("\nBLAH6\n");
	while(!gpsInputQueue.empty()){
		// printf("\nBLAH7\n");
		float gpsDt = (cT - pGpsT);
		_ekf->updateDtGpsFilt(gpsDt);
		gpsDataStruct inputData = gpsInputQueue.front();
		gpsInputQueue.pop();
		pGpsT = cT;
		_ekf->GPSstatus = inputData.status;
		gpsCourse = deg2rad*inputData.course;
		gpsGndSpd = inputData.gndSpd;
		gpsVelD = inputData.velD;
		_ekf->gpsLat = deg2rad*inputData.lat;
		_ekf->gpsLon = deg2rad*inputData.lng;//; - M_PI;
		_ekf->gpsHgt = inputData.alt;

		newDataGps = true;//(inputData.status > 2);
	}
}


// Read Magnetometer Data from BNO055 Sensor
void readMagData(){
	if(cT - pMagT >= magPeriod){
		pMagT = cT;
		newDataMag = true;
		int baseAddrDat = 0x0E;
		int baseAddrOff = 0x5B;
		float magDatas [3];
		float magBias [3];
		for (int i=0;i<3;i++){
			short readingDat = wiringPiI2CReadReg16(imuFD,(baseAddrDat + (i*2)));
			short readingOff = wiringPiI2CReadReg16(imuFD,(baseAddrOff + (i*2)));
			//correct for msb glitches
			if (readingDat < -16500)
				readingDat += 0x8000;
			else if (readingDat > 16500)
				readingDat -= 0x8000;
			if (readingOff < -16500)
				readingOff += 0x8000;
			else if (readingOff > 16500)
				readingOff -= 0x8000;
			magDatas[i] = ((float)readingDat)/16.0;
			magBias[i] = ((float)readingOff)/16.0;
		}
		_ekf->magData.x = magDatas[0];
		_ekf->magBias.x = magBias[0];
		_ekf->magData.y = magDatas[1];
		_ekf->magBias.y = magBias[1];
		_ekf->magData.z = magDatas[2];
		_ekf->magBias.z = magBias[2];
	}
	else
		newDataMag = false;
}



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

void readAhrsData(){
	if(cT - pAhrsT >= ahrsPeriod){
		pAhrsT = cT;
		ahrsEul[0] = imuGetRoll();
		ahrsEul[1] = imuGetPitch();
		ahrsEul[2] = imuGetHeading();
	}
}



void readTimingData(){
    msecStartTime = 1000*cT;
    msecAlignTime = msecStartTime;
    msecEndTime   = msecStartTime + 10000;
    msecVelDelay  = 20;
    msecPosDelay  = 80;
    msecHgtDelay  = 10;
    msecMagDelay  = 50;
    _ekf->EAS2TAS = 1;

    printf("msecVelDelay %d\nmsecPosDelay %d\nmsecHgtDelay %d\nmsecMagDelay %d\n",
            msecVelDelay, msecPosDelay, msecHgtDelay, msecMagDelay);
}


float ConstrainFloat(float val, float min, float max){
    float ret;
    if (val > max) {
        ret = max;
        ekf_debug("> max: %8.4f, val: %8.4f", (double)max, (double)val);
    } else if (val < min) {
        ret = min;
        ekf_debug("< min: %8.4f, val: %8.4f", (double)min, (double)val);
    } else {
        ret = val;
    }

    if (!isfinite(val)) {
        ekf_debug("constrain: non-finite!");
    }

    return ret;
}


//runs as thread to continuously get and update GPS Data
//Unlike the Arduino data, the GPS data is not queued because it runs at a much slower rate. We will likely be getting many (about 5) readings per solution
void gpsParser(){

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
		  gpsDataStruct gpsDataIn;
		  gpsDataIn.lat = ((float)inData.parsed.lat)*1E-7;
		  gpsDataIn.lng = ((float)inData.parsed.lon)*1E-7;
		  gpsDataIn.alt = ((float)inData.parsed.height)*1E-3;
		  gpsDataIn.time = cT;
		  gpsDataIn.course = ((float)inData.parsed.headMot)*1E-5;
		  gpsDataIn.gndSpd = ((float)inData.parsed.gSpeed)*1E-3;
		  gpsDataIn.velD = ((float)inData.parsed.velD)*1E-3;
		  gpsDataIn.status = (inData.parsed.fixType);
		  // printf("\nBLAH10\n");
		  gpsInputQueue.push(gpsDataIn);
		  pGpsT = cT;
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
