#include "estimator_22states.cpp"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
//#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <string>
#include <unistd.h>
#include <iostream>
#include <thread> // for multi-threading
#include <wiringPi.h> //for GPIO functions
#include <wiringPiI2C.h> //for I2C Functions
//#include <cstdlib> // for system() call
#include <queue>   // for std::queue


//Function Prototypes
void readIMUData();
void readGpsData();
void readMagData();
void readAirData();
void readAhrsData();
void readTimingData();
void readOnboardData();
void readFlowData();
void readDistData();
float ConstrainFloat(float val, float min, float max);


// Estimated time delays (msec)
uint32_t msecVelDelay = 230;
uint32_t msecPosDelay = 210;
uint32_t msecHgtDelay = 350;
uint32_t msecRngDelay = 100;
uint32_t msecMagDelay = 30;
uint32_t msecTasDelay = 210;
uint32_t msecOptFlowDelay = 55;

// IMU input data variables
float imuIn;
float tempImu[8];
float IMUtimestamp;
static uint64_t IMUusec = 0;
Vector3f lastAngRate;
Vector3f lastAccel;

// Magnetometer input data variables
float magIn;
float tempMag[8];
float tempMagPrev[8];
float MAGtimestamp = 0;
uint32_t MAGmsec = 0;
uint32_t lastMAGmsec = 0;
bool newDataMag = false;

// AHRS input data variables
float ahrsIn;
float tempAhrs[7];
float tempAhrsPrev[7];
float AHRStimestamp = 0;
uint32_t AHRSmsec = 0;
uint32_t lastAHRStime = 0;
float ahrsEul[3];
float ahrsErrorRP;
float ahrsErrorYaw;
float eulerEst[3]; // Euler angles calculated from filter states
float eulerDif[3]; // difference between Euler angle estimated by EKF and the AHRS solution

// ADS input data variables
float adsIn;
float tempAds[10];
float tempAdsPrev[10];
float ADStimestamp = 0;
uint32_t ADSmsec = 0;
uint32_t lastADSmsec = 0;
float Veas;
bool newAdsData = false;

//Onboard Variables
float onboardTimestamp = 0;
uint32_t onboardMsec = 0;
uint32_t lastOnboardMsec = 0;
bool newOnboardData = false;
float onboardPosNED[3];
float onboardVelNED[3];
float onboardLat;
float onboardLon;
float onboardHgt;

//Optical Flow Variables
uint32_t flowMsec = 0;
uint32_t lastFlowMsec = 0;
bool newFlowData = false;
bool newOptFlowData = false;
float flowTimestamp;      // in seconds
float flowRawPixelX;       // in pixels
float flowRawPixelY;       // in pixels
float flowDistance;        // in meters
float flowQuality;   // distance normalized between 0 and 1
float flowSensorId;        // sensor ID
float flowGyroX = 0.0f;
float flowGyroY = 0.0f;
float flowGyroBiasX = 0.0f;
float flowGyroBiasY = 0.0f;
float flowRadX;
float flowRadY;
float flowRawGroundSpeedX;
float flowRawGroundSpeedY;
uint32_t distMsec = 0;
uint32_t lastDistMsec = 0;
bool newDistData = false;
float distTimestamp = 0.0f;
bool distValid = false;
float distGroundDistance;
float distLastValidReading;

// input data timing
uint64_t msecAlignTime;
uint64_t msecStartTime;
uint64_t msecEndTime;

//GPS Variables
float gpsGndSpd;
float gpsCourse;
float gpsVelD;
float posNED[3];
bool newDataGps = false;
float gpsRaw[7];
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
unsigned long pImuT;
unsigned long pGpsT;
unsigned long pAhrsT;
int attPeriod = 10;
int accelPeriod = 10;
int gyroPeriod = 10;
int magPeriod = 100;
int outputPeriod = 10;
int alphaPeriod = 10;
int imuPeriod = 10;
int gpsPeriod = 200;
int ahrsPeriod = 10;


// I2C Handles and device addresses
int pressFD;
int imuFD;
int press_address = 0x28;
int imu_address = 0x29;

//Used for Dummy input
std::string inStr = std::string();

//EKF object
AttPosEKF                   *_ekf;





// Function to run in thread to constantly reads GPS data and adds to queue. This queue will be read by the main thread to determine if there is new GPS data.
void gpsParser(){
	if(cT - pGpsT >= gpsPeriod){
		gpsDataStruct gpsDataIn;
		gpsDataIn.lat = 29.1900;
		gpsDataIn.lng = -81.0894;
		gpsDataIn.alt = 1000;
		gpsDataIn.time = cT;
		gpsDataIn.course = 275.84;
		gpsDataIn.gndSpd = 0;
		gpsDataIn.velD = 0.0;
		gpsDataIn.status = 3;
		gpsInputQueue.push(gpsDataIn);
		pGpsT = cT;
	}
}

int printstates(){
    printf("States:\n");
    unsigned i = 0;
    printf("Quaternion:\n");
    for (; i<4; i++){
        printf(" %e", _ekf->states[i]);
    }
    printf("\n");
    for (; i<4+6; i++){
        printf(" %e", _ekf->states[i]);
    }
    printf("\n");
    for (; i<4+6+6; i++){
        printf(" %e", _ekf->states[i]);
    }
    printf("\n");
    for (; i < sizeof(_ekf->states) / sizeof(_ekf->states[0]); i++){
        printf(" %e", _ekf->states[i]);
    }
    printf("\n");

    return 0;
}

void error(const char *msg){
    perror(msg);
    exit(0);
}

int main(int argc, char *argv[]){
	imuFD = wiringPiI2CSetup (imu_address);
	
	wiringPiI2CWriteReg8(imuFD,0x3D,00); // set IMU to CONFIG operation mode
	wiringPiI2CWriteReg8(imuFD,0x3B,00); // set Units
	wiringPiI2CWriteReg8(imuFD,0x3D,11); // set IMU to NDOF_FMC_OFF operation mode

	// Launch GPS Parser Thread
	std::thread gpsParserThread(gpsParser);
	
    // Instantiate EKF
    _ekf = new AttPosEKF();

    printf("Filter start\n");

    readTimingData();

    float dt = 0.0f; // time lapsed since last covariance prediction

    int nextComma = 0;
    static uint32_t GPSmsec = 0;
    static uint32_t lastGPSmsec = 0;
    static float GPStimestamp = 0;

	//forever loop
    while (true) {
		
	
		// Set Current Time
		cT = millis();
        pT = cT;
		
		//Read in Sensor Data
		readIMUData();
		readGpsData();
		readMagData();
		readAirData();
		readAhrsData();
		readOnboardData();
		readFlowData();
		readDistData();

        // Apply dtIMU
        _ekf->dtIMU     = 0.001f*(cT - pT);

		// Initialise states, covariance and other data
		if ((cT > msecAlignTime) && !_ekf->statesInitialised && (_ekf->GPSstatus == 3)){
			calcvelNED(_ekf->velNED, gpsCourse, gpsGndSpd, gpsVelD);

			_ekf->InitialiseFilter(_ekf->velNED, _ekf->gpsLat, _ekf->gpsLon, _ekf->gpsHgt, 0.0f);

		} else if ((cT > msecAlignTime) && !_ekf->statesInitialised) {

			float initVelNED[3];

			initVelNED[0] = 0.0f;
			initVelNED[1] = 0.0f;
			initVelNED[2] = 0.0f;
			_ekf->posNE[0] = posNED[0];
			_ekf->posNE[1] = posNED[1];

			_ekf->InitialiseFilter(initVelNED, 0.0, 0.0, 0.0f, 0.0f);

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

		// Fuse optical flow measurements
		if (newFlowData && _ekf->statesInitialised && _ekf->useOpticalFlow && flowQuality > 0.5 && _ekf->Tnb.z.z > 0.71f){
			// recall states and angular rates stored at time of measurement after adjusting for delays
			_ekf->RecallStates(_ekf->statesAtFlowTime, (cT - msecOptFlowDelay));
			_ekf->RecallOmega(_ekf->omegaAcrossFlowTime, (cT - 2*msecOptFlowDelay));

			// Calculate bias errorsfor flow sensor internal gyro
			flowGyroBiasX = 0.999f * flowGyroBiasX + 0.001f * (flowGyroX - _ekf->omegaAcrossFlowTime[0]);
			flowGyroBiasY = 0.999f * flowGyroBiasY + 0.001f * (flowGyroY - _ekf->omegaAcrossFlowTime[1]);

			//use sensor internal rates corrected for bias errors
			_ekf->omegaAcrossFlowTime[0] = flowGyroX - flowGyroBiasX;
			_ekf->omegaAcrossFlowTime[1] = flowGyroY - flowGyroBiasY;

			// calculate rotation matrix
			// Copy required states to local variable names
			float q0 = _ekf->statesAtFlowTime[0];
			float q1 = _ekf->statesAtFlowTime[1];
			float q2 = _ekf->statesAtFlowTime[2];
			float q3 = _ekf->statesAtFlowTime[3];
			float q00 = _ekf->sq(q0);
			float q11 = _ekf->sq(q1);
			float q22 = _ekf->sq(q2);
			float q33 = _ekf->sq(q3);
			float q01 = q0 * q1;
			float q02 = q0 * q2;
			float q03 = q0 * q3;
			float q12 = q1 * q2;
			float q13 = q1 * q3;
			float q23 = q2 * q3;
			_ekf->Tnb_flow.x.x = q00 + q11 - q22 - q33;
			_ekf->Tnb_flow.y.y = q00 - q11 + q22 - q33;
			_ekf->Tnb_flow.z.z = q00 - q11 - q22 + q33;
			_ekf->Tnb_flow.y.x = 2*(q12 - q03);
			_ekf->Tnb_flow.z.x = 2*(q13 + q02);
			_ekf->Tnb_flow.x.y = 2*(q12 + q03);
			_ekf->Tnb_flow.z.y = 2*(q23 - q01);
			_ekf->Tnb_flow.x.z = 2*(q13 - q02);
			_ekf->Tnb_flow.y.z = 2*(q23 + q01);

			// scale from raw pixel flow rate to radians/second
			//float scaleFactor = 0.03f; // best value for quad106.zip data using the 16 mm lens
			//float scaleFactor = 0.06f; // best value for InputFilesPX4_flow.zip data
			//float scaleFactor = 0.882f; // best value for quad123.zip data which outputs flow rates that have already been scaled to rad/sec
			float scaleFactor = 1.000f; // best value for quadOptFlowLogData.zip data which outputs flow rates that have already been scaled to rad/sec
			flowRadX = flowRawPixelX * scaleFactor;
			flowRadY = flowRawPixelY * scaleFactor;

			// calculate motion compensated angular flow rates used for fusion in the main nav filter
			_ekf->flowRadXYcomp[0] = flowRadX/_ekf->flowStates[0] + _ekf->omegaAcrossFlowTime[0];
			_ekf->flowRadXYcomp[1] = flowRadY/_ekf->flowStates[0] + _ekf->omegaAcrossFlowTime[1];

			// these flow rates are not motion compensated and are used for focal length scale factor estimation
			_ekf->flowRadXY[0] = flowRadX;
			_ekf->flowRadXY[1] = flowRadY;

			// perform optical flow fusion
			_ekf->fuseOptFlowData = true;
			_ekf->fuseRngData = false;

			// don't try to estimate focal length scale factor if GPS is not being used or there is no range finder.
			if (_ekf->useGPS && _ekf->useRangeFinder) {
				_ekf->OpticalFlowEKF();
			}
			_ekf->FuseOptFlow();
			_ekf->fuseOptFlowData = false;

			// estimate speed over ground for cross-check of data (debugging only)
			float tempQuat[4];
			float euler[3];
			for (uint8_t j=0; j<4; j++) tempQuat[j] = _ekf->states[j];
			_ekf->quat2eul(euler, tempQuat);
			float bx = (flowRadX - _ekf->angRate.x) * distLastValidReading;
			float by = (flowRadY - _ekf->angRate.y) * distLastValidReading;
			flowRawGroundSpeedY = cos(euler[2]) * bx + -sin(euler[2]) * by;
			flowRawGroundSpeedX = -(sin(euler[2]) * bx + cos(euler[2]) * by);
		} else {
			_ekf->fuseOptFlowData = false;
		}

		// Fuse Ground distance Measurements
		if (newDistData && _ekf->statesInitialised && _ekf->useRangeFinder){
			if (distValid > 0.0f && _ekf->Tnb.z.z > 0.9f) {
				distLastValidReading = distGroundDistance;
				_ekf->rngMea = distGroundDistance;
				_ekf->fuseRngData = true;
				_ekf->fuseOptFlowData = false;
				_ekf->RecallStates(_ekf->statesAtRngTime, (cT - msecRngDelay));
				_ekf->OpticalFlowEKF();
				_ekf->fuseRngData = false;
			}
		}

			// Fuse GPS Measurements
			if (newDataGps){
				// calculate a position offset which is applied to NE position and velocity wherever it is used throughout code to allow GPS position jumps to be accommodated gradually
				_ekf->decayGpsOffset();

				// Convert GPS measurements to Pos NE, hgt and Vel NED
				calcvelNED(_ekf->velNED, gpsCourse, gpsGndSpd, gpsVelD);
				calcposNED(posNED, _ekf->gpsLat, _ekf->gpsLon, _ekf->gpsHgt, _ekf->latRef, _ekf->lonRef, _ekf->hgtRef);

				_ekf->posNE[0] = posNED[0];
				_ekf->posNE[1] = posNED[1];
				 // set fusion flags
				_ekf->fuseVelData = true;
				_ekf->fusePosData = true;
				// recall states stored at time of measurement after adjusting for delays
				_ekf->RecallStates(_ekf->statesAtVelTime, (cT - msecVelDelay));
				_ekf->RecallStates(_ekf->statesAtPosTime, (cT - msecPosDelay));
				// run the fusion step
				_ekf->FuseVelposNED();
				printf("FuseVelposNED at time = %e \n", IMUtimestamp);
			}
			else{
				_ekf->fuseVelData = false;
				_ekf->fusePosData = false;
			}
			calcposNED(posNED, _ekf->gpsLat, _ekf->gpsLon, _ekf->gpsHgt, _ekf->latRef, _ekf->lonRef, _ekf->hgtRef);

			_ekf->posNE[0] = posNED[0];
			_ekf->posNE[1] = posNED[1];

			// fuse GPS
			if (_ekf->useGPS && cT < 1000){
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
				_ekf->fuseVelData = false;
				_ekf->fusePosData = false;
				_ekf->fuseHgtData = false;
			}

            if (newAdsData && _ekf->statesInitialised){
                // Could use a blend of GPS and baro alt data if desired
                _ekf->hgtMea = 1.0f*_ekf->baroHgt + 0.0f*_ekf->gpsHgt - _ekf->hgtRef - _ekf->baroHgtOffset;
                _ekf->fuseVelData = false;
                _ekf->fusePosData = false;
                _ekf->fuseHgtData = true;
                // recall states stored at time of measurement after adjusting for delays
                _ekf->RecallStates(_ekf->statesAtHgtTime, (cT - msecHgtDelay));
                // run the fusion step
                _ekf->FuseVelposNED();
//                printf("time = %e \n", IMUtimestamp);
            }
            else{
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

            // Fuse Airspeed Measurements
            if (newAdsData && _ekf->statesInitialised && _ekf->VtasMeas > 8.0f && _ekf->useAirspeed){
                _ekf->fuseVtasData = true;
                _ekf->RecallStates(_ekf->statesAtVtasMeasTime, (cT - msecTasDelay)); // assume 100 msec avg delay for airspeed data
                _ekf->FuseAirspeed();
            }
            else{
                _ekf->fuseVtasData = false;
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
					printf("\nvelx3, posx3, eulx3, times: %8.4f, %8.4f, %8.4f, \t %8.4f, %8.4f, %8.4f, \t %8.4f, %8.4f, %8.4f, \t %8.4f, %lu", (double)_ekf->states[4], (double)_ekf->states[5], (double)_ekf->states[6], (double)_ekf->states[7], (double)_ekf->states[8], (double)_ekf->states[9], eulerEst[0], eulerEst[1], eulerEst[2], _ekf->dtIMU, cT);
                }

    }

	/* Output Info and code commented out
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
            // printf("dtIMU: %8.6f, dt: %8.6f, cT: %u\n", _ekf->dtIMU, dt, cT);
            // printf("posNED: %8.4f, %8.4f, %8.4f, velNED: %8.4f, %8.4f, %8.4f\n", (double)_ekf->posNED[0], (double)_ekf->posNED[1], (double)_ekf->posNED[2],
            //     (double)_ekf->velNED[0], (double)_ekf->velNED[1], (double)_ekf->velNED[2]);
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
            // printf("states: %s %s %s %s %s %s %s %s %s\n",
            //     (_ekf->statesInitialised) ? "INITIALIZED" : "NON_INIT",
            //     (_ekf->onGround) ? "ON_GROUND" : "AIRBORNE",
            //     (_ekf->fuseVelData) ? "FUSE_VEL" : "INH_VEL",
            //     (_ekf->fusePosData) ? "FUSE_POS" : "INH_POS",
            //     (_ekf->fuseHgtData) ? "FUSE_HGT" : "INH_HGT",
            //     (_ekf->fuseMagData) ? "FUSE_MAG" : "INH_MAG",
            //     (_ekf->fuseVtasData) ? "FUSE_VTAS" : "INH_VTAS",
            //     (_ekf->useAirspeed) ? "USE_AIRSPD" : "IGN_AIRSPD",
            //     (_ekf->useCompass) ? "USE_COMPASS" : "IGN_COMPASS");
	*/

    delete _ekf;
	gpsParserThread.join();
}

void readIMUData(){
	while(cT - pImuT <= imuPeriod){
		cT = millis();
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
	printf(", %lu",pImuT);
	_ekf->dtIMU     = 0.001f*(cT - pImuT);
	_ekf->angRate.x = gyroData[0];
	_ekf->angRate.y = gyroData[1];
	_ekf->angRate.z = gyroData[2];
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
	while(!gpsInputQueue.empty()){
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
		_ekf->gpsLon = deg2rad*inputData.lng - M_PI;
		_ekf->gpsHgt = inputData.alt;
		
		newDataGps = (inputData.status > 2);
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
		MAGmsec = cT;
	}
	else
		newDataMag = false;
}


void readAirData(){
    newAdsData = false;
}


void readDistData(){
    newDistData = false;
}


void readOnboardData(){
    newOnboardData = false;
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


void readFlowData(){
    newFlowData = false;
}


void readTimingData(){
    msecStartTime = 1000*cT;
    msecAlignTime = msecStartTime;
    msecEndTime   = msecStartTime + 10000;
    msecVelDelay  = 20;
    msecPosDelay  = 80;
    msecHgtDelay  = 10;
    msecMagDelay  = 50;
    msecTasDelay  = 10;
    _ekf->EAS2TAS = 1;

    printf("msecVelDelay %d\nmsecPosDelay %d\nmsecHgtDelay %d\nmsecMagDelay %d\nmsecTasDelay %d\n",
            msecVelDelay, msecPosDelay, msecHgtDelay, msecMagDelay, msecTasDelay);
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
