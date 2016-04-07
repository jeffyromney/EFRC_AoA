#ifndef ARDUINOALPHA_H
#define ARDUINOALPHA_H

#include <string>
#include <queue>
#include <stdlib.h>
#include <termios.h>
#include <stdio.h>
#include <iostream>
#include <fcntl.h>

struct alphaDataStruct{
  float alpha;
  float pfwd;
  float p45;
};

class ArduinoAlpha
{
    public:
        /** Default constructor */
        ArduinoAlpha();
        /** Default destructor */
        virtual ~ArduinoAlpha();
        /** Access port
         * \return The current value of port
         */
        std::string Getport() { return port; }
        /** Set port
         * \param val New value to set
         */
        void Setport(const std::string &val) { port = val; }
        void arduinoReadFunc();
        int init();
        bool getNext(alphaDataStruct* data);
        bool getLast_flush(alphaDataStruct* data);
        bool hasData();
    protected:
    private:
        std::string port; //!< Member variable "port"
        struct termios  config;
        unsigned int rxBufIndexer = 0;
        unsigned char rxBuf[7];
        std::queue<alphaDataStruct> arduinoInputQueue;
        int ardFD;
        int state = 0;
        unsigned char c;
        alphaDataStruct inputStruct;
        int numRead = 0;
};

#endif // ARDUINOALPHA_H
