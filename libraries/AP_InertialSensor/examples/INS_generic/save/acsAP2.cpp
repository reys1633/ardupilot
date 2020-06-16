#include "acsAP2.h"
//#include <math.h>
//#include "missileRRT.h"

acsAP2::acsAP2() {   //constructor
     //hal = AP_HAL::get_HAL();
    //for(int i=0; i<6; i++) {  // make sure all valves are off
       // hal.gpio->pinMode( 50+i, HAL_GPIO_OUTPUT );
       // hal.gpio->write( 50+i, 0 );
    //}
}

acsAP2::~acsAP2() { }  // destructor

void acsAP2::update( double gyro[3], double rpy[3], double tof, double dtStep )
{
     hal.console->printf("tof: %6.3f \n", tof );   
}
