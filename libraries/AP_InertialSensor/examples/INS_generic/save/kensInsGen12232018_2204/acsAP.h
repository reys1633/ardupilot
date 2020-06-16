#ifndef ACSAP_H
#define ACSAP_H

#include "missileRRT.h"
#include <AP_HAL/AP_HAL.h>

class acsAP {

public:
    acsAP();
    ~acsAP();
    void update( double gyro[3], double rpy[3], double tof, double dtStep );

private:
    const AP_HAL::HAL &hal = AP_HAL::get_HAL();
    double dcCmdInt[4];
    double dcAchInt[4];

    double tLast;
    double dt;

    bool motoIgn = false;
    bool paraDep = false;
    double gasUsed = 0.0;

};
#endif // ACSAP_H

