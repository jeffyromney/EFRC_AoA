#include "arduinoalpha.h"
#include <string>
#include <queue>
#include <stdlib.h>
#include <termios.h>
#include <stdio.h>
#include <iostream>
#include <fcntl.h>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <stdint.h>
#include <netinet/in.h>
#include <cstring>

ArduinoAlpha::ArduinoAlpha():
    arduinoInputQueue()
{
    //ctor
}

ArduinoAlpha::~ArduinoAlpha()
{
    //dtor
}

// runs as a thread to continuously check serial port and parse new data
void ArduinoAlpha::arduinoReadFunc()
{
    bool done = false;
    while(!done)
    {
        switch(state)
        {
        case 0://waiting for the newline char
            if (read(ardFD,&c,1) > 0)
            {
                if(c == '\n')
                    state = 1;//received newline. start parsing
            }
            else
            {
//                usleep(1000);
                done = true;
//                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            break;
        case 1:
//            int numRead = 0;
            if(numRead < 7)
            {
                int latestRead = read(ardFD,&rxBuf[numRead],7-numRead);//read 7 bytes
                if(latestRead > 0)
                {
                    numRead += latestRead;
                }
                else
                {
//                    usleep(1000);
                    done = true;
//                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
            else{
                numRead = 0;
                int tmpAlpha = ((rxBuf[0] << 8) | rxBuf[1]);
                if(tmpAlpha > 2500)
                {
                    tmpAlpha -= 65536;
                }
                inputStruct.alpha = ((float)tmpAlpha)/50.0;
                inputStruct.pfwd  = ((float)((rxBuf[2] << 8) | rxBuf[3]));
                inputStruct.p45   = ((float)((rxBuf[4] << 8) | rxBuf[5]));
                arduinoInputQueue.push(inputStruct);
                state = 0;
            }
            break;
        }// switch end
    }// while end
}// function end

int  ArduinoAlpha::init()
{
    ardFD = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if(ardFD == -1)
        return -1;

    if(tcgetattr(ardFD, &config) < 0);
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);
    config.c_oflag = 0;
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;
    config.c_cc[VMIN]  = 1;
    config.c_cc[VTIME] = 0;
    if(cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
    {
        return -2;
    }
    if(tcsetattr(ardFD, TCSAFLUSH, &config) < 0)
    {
        return -3;
    }
    return 0;
}


bool ArduinoAlpha::getLast_flush(arduinoDataStruct* data){
//    arduinoDataStruct tmp1 = arduinoInputQueue.front();
    if(this->hasData()){
//      arduinoDataStruct tmp = arduinoInputQueue.back();
      memcpy(data,&arduinoInputQueue.back(),sizeof(arduinoDataStruct));
//      data = arduinoInputQueue.back();
      while(!arduinoInputQueue.empty()){
        arduinoInputQueue.pop();
      }
      return true;
    }
    else return false;
}
bool ArduinoAlpha::getNext(arduinoDataStruct* data){
    if(this->hasData()){
      memcpy(data,&arduinoInputQueue.front(),sizeof(arduinoDataStruct));
//      data = arduinoInputQueue.front();
      return true;
    }
    else return false;
}

bool ArduinoAlpha::hasData(){
    return (!arduinoInputQueue.empty());
}
