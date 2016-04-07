#include "arduinoalpha.h"

ArduinoAlpha::ArduinoAlpha()
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
    arduinoDataStruct inputStruct;
    int state = 0;
    unsigned char c;
    while(true)
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
                usleep(1000);
            }
            break;
        case 1:
            int numRead = 0;
            while(numRead < 7)
            {
                int latestRead = read(ardFD,&rxBuf[numRead],7-numRead);//read 7 bytes
                if(latestRead > 0)
                {
                    numRead += latestRead;
                }
                else
                {
                    usleep(1000);
                }
            }
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
            break;
        }// switch end
    }// while end
}// function end

int initArd()
{
    ardFD = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if(ardFD == -1)
        return -1;

    if(tcgetattr(ardFD, &config) < 0)
        //printf(" FAILED TO GET CONFIG DATA\n");
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
