#include "acsAP.h"
#include <cmath>
#include <AP_Math/AP_Math.h>
acsAP::acsAP() {
}

acsAP::~acsAP() { }

void acsAP::update( double tnow, double gyro[3], double rpy[3] )

{
    //hal.console->printf("%8.4f \n", tnow );
    
    fAvail = tankF0 + gasTimeUsed * tankSlope ;
          
    // forward gain and feedback gain
    wn     = fAvail * omega0 / tankF0;
    momArm = xCg - xAcs;
    kfore  = wn * wn * iyy / momArm;
    kback  = 2.0 * zeta * wn;
          
    // yaw-pitch rate feedback 
    fb[0] = gyro[2] * kback;
    fb[1] = gyro[1] * kback;
    ypCmd[0] = 0.0;
    ypCmd[1] = 0.0; 
    fCmd[0] = -(ypCmd[0] - rpy[2] - fb[0]) * kfore;
    fCmd[1] = -(ypCmd[1] - rpy[1] - fb[1]) * kfore;

    // ACS PWM Controller, 500 hz, 0.
    // lessor of fAvail and acsMax
    
    //val = math::min((float)fAvail, (float)maxACS);

    if ((float)fAvail < (float)maxACS) {
       val = (float)fAvail;
       }
       else{
       val = (float)maxACS;
       }
    
     hal.console->printf("\n %6.3f %6.3f \n", 
        fAvail, maxACS);


    // dutyCycle
    dc[0] = fCmd[0] / val;
    dc[1] = fCmd[1] / val;

    // convert 2 bi commands to 4 uni valve commands
    posCmd[0] = (dc[0] > 0)*dc[0];
    posCmd[1] = (dc[1] > 0)*dc[1]; //pos nozzles, 1 & 2
    negCmd[0] = -(dc[0] < 0)*dc[0];
    negCmd[1] = -(dc[1] < 0)*dc[1]; //neg nozzles, 3 & 4
    dcCmd[0] = posCmd[0];
    dcCmd[1] = posCmd[1];
    dcCmd[2] = negCmd[0];
    dcCmd[3] = negCmd[1]; // goes to integrator
    

   
    uint8_t tint = int(tnow * 5);
    uint8_t testVal = tint % 2; // find even times


// PWM
    int pwmCmd[4];
    for(int i=0; i<4; i++) {
        pwmCmd[i] = (int)(dcCmdInt[i] > dcAchInt[i]); // 0 or 1, output to valves
        ipin = 52;
        hal.gpio->pinMode( ipin+i, HAL_GPIO_OUTPUT );
        hal.gpio->write( ipin+i, pwmCmd[i] );
    }
    if( false ) {
        hal.gpio->pinMode( 53, HAL_GPIO_OUTPUT );
        hal.gpio->write( 53, testVal );
        hal.gpio->pinMode( 54, HAL_GPIO_OUTPUT );
        hal.gpio->write( 54, testVal );
        hal.gpio->pinMode( 55, HAL_GPIO_OUTPUT );
        hal.gpio->write( 55, testVal );   }

  

    // sum all valve commands to integrate for gas usage estimate
    // (count how many valves are open on this cycle)
    double sumCmds = pwmCmd[0] + pwmCmd[1] + pwmCmd[2] + pwmCmd[3]; 

    // update integrators
    for(int i=0; i<4; i++) {

        dcCmdInt[i] += dt * dcCmd[i];
        dcAchInt[i] += dt * pwmCmd[i];
  hal.console->printf("%8.4f %3d %6.3f %6.3f %6.3f %3i %3i %3i %3i %3i \n", 
        tnow, testVal, rpy[0], rpy[1], rpy[2], pwmCmd[0], pwmCmd[1], pwmCmd[2], pwmCmd[3],int(dt));

    }

    // integrate sumCmds to find equiv total time on
    gasTimeUsed += dt * sumCmds;

}
