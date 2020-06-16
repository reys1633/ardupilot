//#ifndef ACSAP2_H
//#define ACSAP2_H

//#include "missileRRT.h"
#include <AP_HAL/AP_HAL.h>

class acsAP2 {

public:
    acsAP2();
    ~acsAP2();
    void update( double gyro[3], double rpy[3], double tof, double dtStep );
    const AP_HAL::HAL &hal = AP_HAL::get_HAL();
    
private:
    
    //const AP_HAL::HAL &hal;
};
//#endif // ACSAP_H

