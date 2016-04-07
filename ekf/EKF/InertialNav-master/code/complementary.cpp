#include "complementary.h"
#include <cstring>
#include <math.h>
#include <stdio.h>

Complementary::Complementary()
{
    pData = data = deltas = output = FILTER_DATA_INIT;
    FiltInit = false;
    //ctor
}

Complementary::~Complementary()
{
    //dtor
}

void Complementary::InitFilt(Filter_Data_t* inData)
{
    memcpy(&pData,inData,sizeof(Filter_Data_t));
    FiltInit = true;
}


void Complementary::rotateAccel(float euler[3], float accel[3], float outData[3])
{
    float rotMat[3][3];

    rotMat[0][0] = cos(euler[2]) * cos(euler[1]);
    rotMat[0][1] = cos(euler[1]) * sin(euler[2]);
    rotMat[0][2] = -sin(euler[1]);
    rotMat[1][0] = cos(euler[2]) * sin(euler[0]) * sin(euler[1]) - cos(euler[0]) * sin(euler[2]);
    rotMat[1][1] = cos(euler[0]) * cos(euler[2]) + sin(euler[0]) * cos(euler[2]) * sin(euler[1]);
    rotMat[1][2] = cos(euler[1]) * sin(euler[1]);
    rotMat[2][0] = sin(euler[0]) * sin(euler[2]) + cos(euler[0]) * cos(euler[2]) * sin(euler[1]);
    rotMat[2][1] = cos(euler[0]) * sin(euler[2]) * sin(euler[1]) - cos(euler[2]) * sin(euler[0]);
    rotMat[2][2] = cos(euler[0]) * cos(euler[1]);



    outData[0] = rotMat[0][0] * accel[0] + rotMat[0][1] * accel[1] + rotMat[0][2] * accel[2];
    outData[1] = rotMat[1][0] * accel[0] + rotMat[1][1] * accel[1] + rotMat[1][2] * accel[2];
    outData[2] = rotMat[2][0] * accel[0] + rotMat[2][1] * accel[1] + rotMat[2][2] * accel[2];

//	return outData;

}



void Complementary::runCompFilt(Filter_Data_t* inData)
{

    memcpy(&data,inData,sizeof(Filter_Data_t));

    if(!data.useGPS)
    {
        iConst = 1.0;
        gConst = 0;
    }
    else
    {
        //STATIC FILTER CONSTANTS INSTEAD OF TIME COMPUTED.
        iConst = 0.6;
        gConst = 0.4;

    }


    double m_per_deg_lat = 111132.954 - 559.822 * cos( 2 * data.lat ) + 1.175 * cos( 4 * data.lat);
    double m_per_deg_lon = 111132.954 * cos ( data.lat );

    deltas.lat = data.lat - pData.lat;
    deltas.lon = data.lon - pData.lon;
    deltas.alt = data.alt - pData.alt;

    deltas.ned[0] = deltas.lat * m_per_deg_lat;
    deltas.ned[1] = deltas.lon * m_per_deg_lon;
    deltas.ned[2] = -deltas.alt;

    deltas.vNed[0] = data.vNed[0] - pData.vNed[0];
    deltas.vNed[1] = data.vNed[1] - pData.vNed[1];
    deltas.vNed[2] = data.vNed[2] - pData.vNed[2];


    // Calculate Yaw
    data.euler[2] = atan2(
                        (data.mag[2] * sin(data.euler[0]))
                        - (data.mag[1] * cos(data.euler[0]))
                        ,
                        (data.mag[0] * cos(-data.euler[1]))
                        + (data.mag[1] * sin(-data.euler[1]) * sin(data.euler[0]))
                        + (data.mag[2] * sin(-data.euler[1]) * cos(data.euler[0]))
                    );



    float ecefAccel[3];
    float dVNed[3];
    float dNed[3];
    //account for accel offsets
    data.accel[0] -= ACCEL_OFFSET_X;
    data.accel[1] -= ACCEL_OFFSET_Y;
    data.accel[2] -= ACCEL_OFFSET_Z;
    rotateAccel(data.euler, data.accel,ecefAccel);
    dVNed[0] = ecefAccel[0] * data.dTi;
    dVNed[1] = ecefAccel[1] * data.dTi;
    dVNed[2] = -ecefAccel[2] * data.dTi;
    dNed[0] = dVNed[0] * data.dTi;
    dNed[1] = dVNed[1] * data.dTi;
    dNed[2] = -dVNed[2] * data.dTi;
    output.ned[0] = pData.ned[0] + (deltas.ned[0] * gConst + dNed[0] * iConst);
    output.ned[1] = pData.ned[1] + (deltas.ned[1] * gConst + dNed[1] * iConst);
    output.ned[2] = pData.ned[2] + (deltas.ned[2] * gConst + dNed[2] * iConst);
    output.vNed[0] = pData.vNed[0] + (deltas.vNed[0] * gConst) + (dVNed[0] * iConst);
    output.vNed[1] = pData.vNed[1] + (deltas.vNed[1] * gConst) + (dVNed[1] * iConst);
    output.vNed[2] = pData.vNed[2] + (deltas.vNed[2] * gConst) + (dVNed[2] * iConst);
//  printf("[%f, %f, %f : %f, %f, %f]",dNed[0],  dNed[1], dNed[2], output.ned[0], output.ned[1], output.ned[2]);
//  output.euler = NULL;
//  output.accel = NULL;
//  output.useGPS = NULL;
    output.lat = pData.lat + (deltas.lat * gConst + (dNed[0] / m_per_deg_lat) * iConst);
    output.lon = pData.lon + (deltas.lon * gConst + (dNed[1] / m_per_deg_lon) * iConst);
    output.alt = -output.ned[2];
//  output.dTi = NULL;
//  output.dTg = NULL;
    output.accel[0] = data.accel[0];
    output.accel[1] = data.accel[1];
    output.accel[2] = data.accel[2];
    output.euler[0] = data.euler[0];
    output.euler[1] = data.euler[1];
    output.euler[2] = data.euler[2];
    std::memcpy(&pData,&output,sizeof(pData));
}
