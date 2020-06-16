#include "acsAP.h"

acsAP::acsAP() { 
    for(int i=0; i<4; i++) {  // make sure all valves are off
        hal.gpio->pinMode( 50+i, HAL_GPIO_OUTPUT );
        hal.gpio->write( 50+i, 0 );
    }
}

acsAP::~acsAP() { }


void acsAP::update( double gyro[3], double rpy[3], double tof, double dtStep )
{
    // ignite motor by setting pin(?) high
    if( !motoIgn ) {
        if( tof > 0.050 ) {
            hal.gpio->pinMode( 50, HAL_GPIO_OUTPUT );
            hal.gpio->write( 50, 0 );
            motoIgn = true;
        }
    }
        // deploy parachute by setting pin(?) high
    if( !paraDep ) {
        if( tof > 1.5 ) {
            hal.gpio->pinMode( 51, HAL_GPIO_OUTPUT );
            hal.gpio->write( 51, 0 );
        }
    }
    
    if(tof > 20.0L) {
        for(int i=0; i<4; i++) {  // make sure all valves are off
            hal.gpio->pinMode( 52+i, HAL_GPIO_OUTPUT );
            hal.gpio->write( 52+i, 0 );
        }

        hal.console->printf("stopping at 20 seconds\n");
         while(true) {   // do forever loop to suspend operation
            continue;
        }
    }

    double fAvail = tankF0 + gasUsed * tankSlope ;
          
    // forward gain and feedback gain
    double wn     = fAvail * omega0 / tankF0;
    double momArm = xCg - xAcs;
    double kfore  = wn * wn * iyy / momArm;
    double kback  = 2.0L * zeta * wn;

    // yaw-pitch rate feedback 
    double fb[2] = { gyro[0] * kback, gyro[1] * kback };
    double ypCmd[2] = {0.0, 0.0}; 
    double fCmd[2] = {-(ypCmd[0] - rpy[0] - fb[0]) * kfore, 
                          -(ypCmd[1] - rpy[1] - fb[1]) * kfore };
/*  double fb[2] = { gyro[2] * kback, gyro[1] * kback };
    double ypCmd[2] = {0.0, 0.0}; 
    double fCmd[2] = {-(ypCmd[0] - rpy[2] - fb[0]) * kfore, 
                          -(ypCmd[1] - rpy[1] - fb[1]) * kfore }; */
                       
    // lessor of fAvail and acsMax
    // double val = min(fAvail, maxACS);
    double val = fAvail;
    if(maxACS < fAvail) {
        val = maxACS;
    }

    // dutyCycle
    double dc[2] = { fCmd[0] / val, fCmd[1] / val };

    // convert 2 bi commands to 4 uni valve commands
    double posCmd[2] = { (dc[0] > 0)*dc[0], (dc[1] > 0)*dc[1]}; //pos nozzles, 1 & 2
    double negCmd[2] = {-(dc[0] < 0)*dc[0],-(dc[1] < 0)*dc[1]}; //neg nozzles, 3 & 4
    double dcCmd[4] = { posCmd[0], posCmd[1], negCmd[0], negCmd[1] }; // goes to integrator
         
    uint8_t tint = (int)( tof * 5 );
    uint8_t testVal = tint % 2; // find even times

    // PWM
    int pwmCmd[4];
    for(int i=0; i<4; i++) {
        pwmCmd[i] = (int)(dcCmdInt[i] > dcAchInt[i]); // 0 or 1, output to valves
        hal.gpio->pinMode( 52+i, HAL_GPIO_OUTPUT );
        hal.gpio->write( 52+i, pwmCmd[i] );
    }

    hal.console->printf("%8.4f %3d %6.3f %6.3f %6.3f %3i %3i %3i %3i\n", 
        tof, testVal, rpy[0], rpy[1], rpy[2], pwmCmd[0], pwmCmd[1], pwmCmd[2], pwmCmd[3] );

    // sum all valve commands to integrate for gas usage estimate
    // (count how many valves are open on this cycle)
    double sumCmds = pwmCmd[0] + pwmCmd[1] + pwmCmd[2] + pwmCmd[3]; 

    // update integrators
    for(int i=0; i<4; i++) {
        dcCmdInt[i] += dtStep * dcCmd[i];
        dcAchInt[i] += dtStep * pwmCmd[i];
    }

    // integrate sumCmds to find equiv total time on
    gasUsed += dtStep * sumCmds;
}
