//#ifndef ACSAP_H
//#define ACSAP_H

#include "missileRRT.h"
#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS/sdcard.h"

class acsAP {

public:
    acsAP();
    ~acsAP();
    void init();
    const AP_HAL::HAL &hal = AP_HAL::get_HAL();
    uint8_t update( double ypCmd[2], double gyro[3], double rpy[3], double tof, double dtStep, int printon );
    void shutdown();
    
private:
    double dcCmdInt[4];
    double dcAchInt[4];

    double tLast;
    double dt;

    bool   motoIgn = false;
    bool   paraDep = false;
    double gasUsed = 0.0;
    int    testnoz = 1; // used to produce "Ready" tone by pulsing nozzles
    int    toneCount = 0;
    int    firesrm = 0;
    int    firepara= 0;
};
//#endif // ACSAP_H

