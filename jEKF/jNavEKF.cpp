#include "jNavEKF_core.h"
#include <stdio.h>
#define VELNE_NOISE_DEFAULT     0.5f
#define VELD_NOISE_DEFAULT      0.7f
#define POSNE_NOISE_DEFAULT     0.5f
#define ALT_NOISE_DEFAULT       0.5f
#define MAG_NOISE_DEFAULT       0.05f
#define GYRO_PNOISE_DEFAULT     0.015f
#define ACC_PNOISE_DEFAULT      0.5f
#define GBIAS_PNOISE_DEFAULT    8E-06f
#define ABIAS_PNOISE_DEFAULT    0.00005f
#define MAGE_PNOISE_DEFAULT     0.0003f
#define MAGB_PNOISE_DEFAULT     0.0003f
#define VEL_GATE_DEFAULT        6
#define POS_GATE_DEFAULT        30
#define HGT_GATE_DEFAULT        20
#define MAG_GATE_DEFAULT        3
#define MAG_CAL_DEFAULT         0
#define GLITCH_ACCEL_DEFAULT    150
#define GLITCH_RADIUS_DEFAULT   20
#define FLOW_MEAS_DELAY         25
#define FLOW_NOISE_DEFAULT      0.3f
#define FLOW_GATE_DEFAULT       3


#define earthRate 0.000072921f // earth rotation rate (rad/sec)
#define STARTUP_WIND_SPEED 3.0f
#define MAX_GYRO_BIAS 0.1745f
#define INIT_ACCEL_BIAS_UNCERTAINTY 0.1f



// constructor
NavEKF_core::NavEKF_core() 
    state(*reinterpret_cast<struct state_elements *>(&states)),
    gpsNEVelVarAccScale(0.05f),     // Scale factor applied to horizontal velocity measurement variance due to manoeuvre acceleration - used when GPS doesn't report speed error
    gpsDVelVarAccScale(0.07f),      // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration - used when GPS doesn't report speed error
    gpsPosVarAccScale(0.05f),       // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
    msecHgtDelay(60),               // Height measurement delay (msec)
    msecMagDelay(40),               // Magnetometer measurement delay (msec)
    msecTasDelay(240),              // Airspeed measurement delay (msec)
    gpsRetryTimeUseTAS(10000),      // GPS retry time with airspeed measurements (msec)
    gpsRetryTimeNoTAS(7000),        // GPS retry time without airspeed measurements (msec)
    gpsFailTimeWithFlow(5000),      // If we have no GPS for longer than this and we have optical flow, then we will switch across to using optical flow (msec)
    hgtRetryTimeMode0(10000),       // Height retry time with vertical velocity measurement (msec)
    hgtRetryTimeMode12(5000),       // Height retry time without vertical velocity measurement (msec)
    tasRetryTime(5000),             // True airspeed timeout and retry interval (msec)
    magFailTimeLimit_ms(10000),     // number of msec before a magnetometer failing innovation consistency checks is declared failed (msec)
    magVarRateScale(0.05f),         // scale factor applied to magnetometer variance due to angular rate
    gyroBiasNoiseScaler(2.0f),      // scale factor applied to imu gyro bias learning before the vehicle is armed
    accelBiasNoiseScaler(1.0f),     // scale factor applied to imu accel bias learning before the vehicle is armed
    msecGpsAvg(200),                // average number of msec between GPS measurements
    msecHgtAvg(100),                // average number of msec between height measurements
    msecMagAvg(100),                // average number of msec between magnetometer measurements
    msecBetaAvg(100),               // average number of msec between synthetic sideslip measurements
    msecBetaMax(200),               // maximum number of msec between synthetic sideslip measurements
    msecFlowAvg(100),               // average number of msec between optical flow measurements
    dtVelPos(0.2f),                 // number of seconds between position and velocity corrections. This should be a multiple of the imu update interval.
    covTimeStepMax(0.02f),          // maximum time (sec) between covariance prediction updates
    covDelAngMax(0.05f),            // maximum delta angle between covariance prediction updates
    TASmsecMax(200),                // maximum allowed interval between airspeed measurement updates
    DCM33FlowMin(0.71f),            // If Tbn(3,3) is less than this number, optical flow measurements will not be fused as tilt is too high.
    fScaleFactorPnoise(1e-10f),     // Process noise added to focal length scale factor state variance at each time step
    flowTimeDeltaAvg_ms(100),       // average interval between optical flow measurements (msec)
    flowIntervalMax_ms(100),        // maximum allowable time between flow fusion events
    gndEffectTimeout_ms(1000),          // time in msec that baro ground effect compensation will timeout after initiation
    gndEffectBaroScaler(4.0f)      // scaler applied to the barometer observation variance when operating in ground effect
{
}


// Check basic filter health metrics and return a consolidated health status
bool NavEKF_core::healthy(void) const
{
    uint8_t faultInt;
    getFilterFaults(faultInt);
    if (faultInt > 0) {
        return false;
    }
    if (velTestRatio > 1 && posTestRatio > 1 && hgtTestRatio > 1) {
        // all three metrics being above 1 means the filter is
        // extremely unhealthy.
        return false;
    }
    // Give the filter a second to settle before use
    if ((imuSampleTime_ms - ekfStartTime_ms) < 1000 ) {
        return false;
    }
    // barometer and position innovations must be within limits when on-ground
    float horizErrSq = sq(innovVelPos[3]) + sq(innovVelPos[4]);
    if ((!hgtHealth || fabsf(hgtInnovFiltState) > 1.0f || horizErrSq > 2.0f)) {
        return false;
    }

    // all OK
    return true;
}


// resets position states to last GPS measurement or to zero if in constant position mode
void NavEKF_core::ResetPosition(void)
{
    // Store the position before the reset so that we can record the reset delta
    posResetNE.x = state.position.x;
    posResetNE.y = state.position.y;

    if (constPosMode || (PV_AidingMode != AID_ABSOLUTE)) {
        state.position.x = 0;
        state.position.y = 0;
    } else if (!gpsNotAvailable) {
        // write to state vector and compensate for GPS latency
        state.position.x = gpsPosNE.x + 0.001f*velNED.x*float(frontend._msecPosDelay);
        state.position.y = gpsPosNE.y + 0.001f*velNED.y*float(frontend._msecPosDelay);
        // the estimated states at the last GPS measurement are set equal to the GPS measurement to prevent transients on the first fusion
        statesAtPosTime.position.x = gpsPosNE.x;
        statesAtPosTime.position.y = gpsPosNE.y;
    }
    // stored horizontal position states to prevent subsequent GPS measurements from being rejected
    for (uint8_t i=0; i<=49; i++){
        storedStates[i].position.x = state.position.x;
        storedStates[i].position.y = state.position.y;
    }

    // Calculate the position jump due to the reset
    posResetNE.x = state.position.x - posResetNE.x;
    posResetNE.y = state.position.y - posResetNE.y;

    // store the time of the reset
    lastPosReset_ms = imuSampleTime_ms;
}

// Reset velocity states to last GPS measurement if available or to zero if in constant position mode or if PV aiding is not absolute
// Do not reset vertical velocity using GPS as there is baro alt available to constrain drift
void NavEKF_core::ResetVelocity(void)
{
    // Store the velocity before the reset so that we can record the reset delta
    velResetNE.x = state.velocity.x;
    velResetNE.y = state.velocity.y;

    if (constPosMode || PV_AidingMode != AID_ABSOLUTE) {
         state.velocity.zero();
         state.vel1.zero();
         state.vel2.zero();
         posDownDerivative = 0.0f;
    } else if (!gpsNotAvailable) {
        // reset horizontal velocity states
        state.velocity.x  = velNED.x; // north velocity from blended accel data
        state.velocity.y  = velNED.y; // east velocity from blended accel data
        state.vel1.x      = velNED.x; // north velocity from IMU1 accel data
        state.vel1.y      = velNED.y; // east velocity from IMU1 accel data
        state.vel2.x      = velNED.x; // north velocity from IMU2 accel data
        state.vel2.y      = velNED.y; // east velocity from IMU2 accel data
        // over write stored horizontal velocity states to prevent subsequent GPS measurements from being rejected
        for (uint8_t i=0; i<=49; i++){
            storedStates[i].velocity.x = velNED.x;
            storedStates[i].velocity.y = velNED.y;
        }
    }

    // Calculate the velocity jump due to the reset
    velResetNE.x = state.velocity.x - velResetNE.x;
    velResetNE.y = state.velocity.y - velResetNE.y;

    // store the time of the reset
    lastVelReset_ms = imuSampleTime_ms;
}



// reset the vertical position state using the last height measurement
void NavEKF_core::ResetHeight(void)
{
    // read the altimeter
    readHgtData();
    // write to the state vector
    state.position.z = -hgtMea; // down position from blended accel data
    state.posD1 = -hgtMea; // down position from IMU1 accel data
    state.posD2 = -hgtMea; // down position from IMU2 accel data
    terrainState = state.position.z + rngOnGnd;
    // Reset the vertical velocity state using GPS vertical velocity if we are airborne (use arm status as a surrogate)
    // Check that GPS vertical velocity data is available and can be used
    if (vehicleArmed && !gpsNotAvailable) {
        state.velocity.z =  velNED.z;
    }
    // reset stored vertical position states to prevent subsequent GPS measurements from being rejected
    for (uint8_t i=0; i<=49; i++){
        storedStates[i].position.z = -hgtMea;
    }
    // reset the height state for the complementary filter used to provide a vertical position dervative
    posDown = state.position.z;
}


// this function is used to initialise the filter whilst moving, using the AHRS DCM solution
// it should NOT be used to re-initialise after a timeout as DCM will also be corrupted
bool NavEKF_core::InitialiseFilterDynamic(void)
{

    // this forces healthy() to be false so that when we ask for ahrs
    // attitude we get the DCM attitude regardless of the state of AHRS_EKF_USE
    statesInitialised = false;

    // If we are a plane and don't have GPS lock then don't initialise
    if (_ahrs->get_gps().status() < 3) {
        return false;
    }

    // Set re-used variables to zero
    InitialiseVariables();

    // get initial time deltat between IMU measurements (sec)
    dtDelAng = dtIMUavg = _ahrs->get_ins().get_loop_delta_t();

    // set number of updates over which gps and baro measurements are applied to the velocity and position states
    gpsUpdateCountMaxInv = (dtIMUavg * 1000.0f)/float(msecGpsAvg);
    gpsUpdateCountMax = uint8_t(1.0f/gpsUpdateCountMaxInv);
    hgtUpdateCountMaxInv = (dtIMUavg * 1000.0f)/float(msecHgtAvg);
    hgtUpdateCountMax = uint8_t(1.0f/hgtUpdateCountMaxInv);
    magUpdateCountMaxInv = (dtIMUavg * 1000.0f)/float(msecMagAvg);
    magUpdateCountMax = uint8_t(1.0f/magUpdateCountMaxInv);
    flowUpdateCountMaxInv = (dtIMUavg * 1000.0f)/float(msecFlowAvg);
    flowUpdateCountMax = uint8_t(1.0f/flowUpdateCountMaxInv);

    // calculate initial orientation and earth magnetic field states
    state.quat = calcQuatAndFieldStates(_ahrs->roll, _ahrs->pitch);

    // write to state vector
    state.gyro_bias.zero();
    state.accel_zbias1 = 0;
    state.accel_zbias2 = 0;
    state.wind_vel.zero();

    // read the GPS and set the position and velocity states
    readGpsData();
    ResetVelocity();
    ResetPosition();

    // read the barometer and set the height state
    ResetHeight();

    // set stored states to current state
    StoreStatesReset();

    // set to true now that states have be initialised
    statesInitialised = true;

    // define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(earthRateNED, _ahrs->get_home().lat);

    // initialise IMU pre-processing states
    readIMUData();

    // initialise the covariance matrix
    CovarianceInit();

    return true;
}

// Update Filter States - this should be called whenever new IMU data is available
void NavEKF_core::UpdateFilter()
{
    // zero the delta quaternion used by the strapdown navigation because it is published
    // and we need to return a zero rotation of the INS fails to update it
    correctedDelAngQuat.initialise();

    // don't run filter updates if states have not been initialised
    if (!statesInitialised) {
        return;
    }


    //get starting time for update step
    imuSampleTime_ms = cT;

    // read IMU data and convert to delta angles and velocities
    readIMUData();

    static bool prev_armed = false;
    bool armed = getVehicleArmStatus();

    // the vehicle was previously disarmed and time has slipped
    // gyro auto-zero has likely just been done - skip this timestep
    if (!prev_armed && dtDelAng > dtIMUavg*5.0f) {
        // stop the timer used for load measurement
        prev_armed = armed;
        return;
    }
    prev_armed = armed;

    // detect if the filter update has been delayed for too long
    if (dtDelAng > 0.2f) {
        // we have stalled for too long - reset states
        ResetVelocity();
        ResetPosition();
        ResetHeight();
        StoreStatesReset();
        //Initialise IMU pre-processing states
        readIMUData();
        // stop the timer used for load measurement
        return;
    }

    // check if on ground
    SetFlightAndFusionModes();

    // Check arm status and perform required checks and mode changes
    performArmingChecks();

    // run the strapdown INS equations every IMU update
    UpdateStrapdownEquationsNED();

    // store the predicted states for subsequent use by measurement fusion
    StoreStates();

    // sum delta angles and time used by covariance prediction
    summedDelAng = summedDelAng + correctedDelAng;
    summedDelVel = summedDelVel + correctedDelVel12;
    dt += dtDelAng;

    // perform a covariance prediction if the total delta angle has exceeded the limit
    // or the time limit will be exceeded at the next IMU update
    if (((dt >= (covTimeStepMax - dtDelAng)) || (summedDelAng.length() > covDelAngMax))) {
        CovariancePrediction();
    } else {
        covPredStep = false;
    }

    // Update states using GPS, altimeter, compass, airspeed and synthetic sideslip observations
    SelectVelPosFusion();
    SelectMagFusion();
    SelectFlowFusion();
    SelectTasFusion();
    SelectBetaFusion();

}


// select fusion of velocity, position and height measurements
void NavEKF_core::SelectVelPosFusion()
{

    // If we haven't received height data for a while, then declare the height data as being timed out
    // set timeout period based on whether we have vertical GPS velocity available to constrain drift
    hgtRetryTime = (useGpsVertVel && !velTimeout) ? hgtRetryTimeMode0 : hgtRetryTimeMode12;
    if (imuSampleTime_ms - lastHgtMeasTime > hgtRetryTime) {
        hgtTimeout = true;
    }

        fuseHgtData = false;

    // check for and read new GPS data
    readGpsData();

    // Specify which measurements should be used and check data for freshness
    if (PV_AidingMode == AID_ABSOLUTE) {

        // check if we can use opticalflow as a backup
        bool optFlowBackup = false;

        // Set GPS time-out threshold depending on whether we have an airspeed sensor to constrain drift
        uint16_t gpsRetryTimeout = gpsRetryTimeNoTAS;

        // Set the time that copters will fly without a GPS lock before failing the GPS and switching to a non GPS mode
        uint16_t gpsFailTimeout = gpsRetryTimeout;

        // If we haven't received GPS data for a while, then declare the position and velocity data as being timed out
        if (imuSampleTime_ms - lastFixTime_ms > gpsFailTimeout) {
            posTimeout = true;
            velTimeout = true;
            // If this happens in flight and we don't have airspeed or sideslip assumption or optical flow to constrain drift, then go into constant position mode.
            // Stay in that mode until the vehicle is re-armed.
            // If we can do optical flow nav (valid flow data and hieght above ground estimate, then go into flow nav mode.
            // Stay in that mode until the vehicle is dis-armed.
            if (vehicleArmed && !useAirspeed() && !assume_zero_sideslip()) {
                if (optFlowBackup) {
                    // we can do optical flow only nav
                    frontend._fusionModeGPS = 3;
                    PV_AidingMode = AID_RELATIVE;
                    constPosMode = false;
                } else {
                    constPosMode = true;
                    PV_AidingMode = AID_NONE;
                    posTimeout = true;
                    velTimeout = true;
                    // reset the velocity
                    ResetVelocity();
                    // store the current position to be used to keep reporting the last known position
                    lastKnownPositionNE.x = state.position.x;
                    lastKnownPositionNE.y = state.position.y;
                    // reset the position
                    ResetPosition();
                }
                // set the position and velocity timeouts to indicate we are not using GPS data
                posTimeout = true;
                velTimeout = true;
            }
        }

        // command fusion of GPS data and reset states as required
        if (newDataGps && (PV_AidingMode == AID_ABSOLUTE)) {
            // reset data arrived flag
            newDataGps = false;
            // reset state updates and counter used to spread fusion updates across several frames to reduce 10Hz pulsing
            memset(&gpsIncrStateDelta[0], 0, sizeof(gpsIncrStateDelta));
            gpsUpdateCount = 0;
            // use both if GPS use is enabled
            fuseVelData = true;
            fusePosData = true;
            // If a long time since last GPS update, then reset position and velocity and reset stored state history
            if (imuSampleTime_ms - secondLastFixTime_ms > gpsRetryTimeout) {
                ResetPosition();
                ResetVelocity();
                // record the fail time
                lastPosFailTime = imuSampleTime_ms;
                // Reset the normalised innovation to avoid false failing the bad position fusion test
                posTestRatio = 0.0f;
            }
        } else {
            fuseVelData = false;
            fusePosData = false;
        }
    } else if (constPosMode && (fuseHgtData || ((imuSampleTime_ms - lastConstPosFuseTime_ms) > 200))) {
        // In constant position mode use synthetic position and velocity measurements set to zero whenever we are fusing a height measurement
        // If no height has been received for 200 msec, then fuse anyway so we have a guaranteed minimum aiding rate equivalent to GPS
        // only fuse synthetic measurements when rate of change of velocity is less than 0.5g to reduce attitude errors due to launch acceleration
        // do not use velocity fusion to reduce the effect of movement on attitude
        if (!vehicleArmed) {
            fuseVelData = true;
        } else {
            fuseVelData = false;
        }
        if (accNavMag < 4.9f) {
            fusePosData = true;
        } else {
            fusePosData = false;
        }
        // record the fusion time - used to control fusion rate when there is no baro data
        lastConstPosFuseTime_ms = imuSampleTime_ms;
    } else {
        fuseVelData = false;
        fusePosData = false;
    }

    // perform fusion
    if (fuseVelData || fusePosData || fuseHgtData) {
        // ensure that the covariance prediction is up to date before fusing data
        if (!covPredStep) CovariancePrediction();
        FuseVelPosNED();
    }

    // Fuse corrections to quaternion, position and velocity states across several time steps to reduce 5 and 10Hz pulsing in the output
    if (gpsUpdateCount < gpsUpdateCountMax) {
        gpsUpdateCount ++;
        for (uint8_t i = 0; i <= 9; i++) {
            states[i] += gpsIncrStateDelta[i];
        }
    }
    if (hgtUpdateCount < hgtUpdateCountMax) {
        hgtUpdateCount ++;
        for (uint8_t i = 0; i <= 9; i++) {
            states[i] += hgtIncrStateDelta[i];
        }
    }

    // Detect and declare bad GPS aiding status for minimum 10 seconds if a GPS rejection occurs after
    // rejection of GPS and reset to GPS position. This addresses failure case where errors cause ongoing rejection
    // of GPS and severe loss of position accuracy.
    uint32_t gpsRetryTime;
    if (useAirspeed()) {
        gpsRetryTime = gpsRetryTimeUseTAS;
    } else {
        gpsRetryTime = gpsRetryTimeNoTAS;
    }
    if ((posTestRatio > 2.0f) && ((imuSampleTime_ms - lastPosFailTime) < gpsRetryTime) && ((imuSampleTime_ms - lastPosFailTime) > gpsRetryTime/2) && fusePosData) {
        lastGpsAidBadTime_ms = imuSampleTime_ms;
        gpsAidingBad = true;
    }
    gpsAidingBad = gpsAidingBad && ((imuSampleTime_ms - lastGpsAidBadTime_ms) < 10000);
}

