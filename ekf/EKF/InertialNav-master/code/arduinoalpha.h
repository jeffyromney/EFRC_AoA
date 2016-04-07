#ifndef ARDUINOALPHA_H
#define ARDUINOALPHA_H

#include <string>
#include <queue>
#include <stdlib.h>
#include <termios.h>
#include <stdio.h>
#include <iostream>


struct arduinoDataStruct{
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
        void Setport(std::string val) { port = val; }
    protected:
    private:
        std::string port; //!< Member variable "port"
        //struct termios  config;
        unsigned int rxBufIndexer = 0;
        unsigned char rxBuf[7];
        std::queue<arduinoDataStruct> arduinoInputQueue;
        void arduinoReadFunc();
        int ardFD;
};

#endif // ARDUINOALPHA_H
