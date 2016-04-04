#ifndef COMPLEMENTARY_H
#define COMPLEMENTARY_H



#define ACCEL_OFFSET_X 0.1516
#define ACCEL_OFFSET_Y -0.0149
#define ACCEL_OFFSET_Z 9.83109

#define FILTER_DATA_INIT {\
false,\
0.0,\
0.0,\
{0.0,0.0,0.0},\
{0.0,0.0,0.0},\
{0.0,0.0,0.0},\
0.0,\
0.0,\
0.0,\
{0.0,0.0,0.0},\
{0.0,0.0,0.0},\
}

typedef struct Filter_Data_t{
  bool useGPS;
  float dTi;
  float dTg;
  float euler[3];
  float accel[3];
  float mag[3];
  float lat;
  float lon;
  float alt;
  float ned[3];
  float vNed[3];
}Filter_Data_t;

class Complementary
{
    public:
        /** Default constructor */
        Complementary();
        /** Default destructor */
        virtual ~Complementary();
        /** Access compFiltInit
         * \return The current value of compFiltInit
         */
        bool GetcompFiltInit() { return compFiltInit; }
        /** Set compFiltInit
         * \param val New value to set
         */
        void SetcompFiltInit(bool val) { compFiltInit = val; }
        /** Access gConst
         * \return The current value of gConst
         */
        float GetgConst() { return gConst; }
        /** Set gConst
         * \param val New value to set
         */
        void SetgConst(float val) { gConst = val; }
        /** Access iConst
         * \return The current value of iConst
         */
        float GetiConst() { return iConst; }
        /** Set iConst
         * \param val New value to set
         */
        void SetiConst(float val) { iConst = val; }

        void InitFilt(Filter_Data_t* inData);

        void LoadData(Filter_Data_t* inData);

        Filter_Data_t pData;
        Filter_Data_t data;
        Filter_Data_t deltas;
        Filter_Data_t output;

        void runCompFilt(Filter_Data_t* inData);
        void rotateAccel(float euler[3], float accel[3], float outData[3]);
    protected:
    private:
        bool compFiltInit; //!< Member variable "compFiltInit"
        float gConst; //!< Member variable "gConst"
        float iConst; //!< Member variable "iConst"
};

#endif // COMPLEMENTARY_H
