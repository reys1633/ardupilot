#include "missileRRT.h"
#include <AP_HAL/AP_HAL.h>

class acsAP {
    public:
        acsAP();
        ~acsAP();
        void update( double tNow, double gyro[3], double rpy[3] );
       
    private:  
        const AP_HAL::HAL &hal = AP_HAL::get_HAL();

        double dcCmdInt[4];
        double dcAchInt[4];
        double gasTimeUsed = 0.0; 
        double tLast = 0.0;
        double dt;
        uint8_t ipin;
        double fAvail;
        double wn;
        double momArm;
        double kfore;
        double kback;
        double fb[2];
        double ypCmd[2];
        double fCmd[2];
        double val;
        double dc[2];
        double posCmd[2];
        double negCmd[2];
        double dcCmd[4];

};        
