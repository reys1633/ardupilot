#include "acsAP.h"
#define R2D 57.2958

acsAP::acsAP() { }  //constructor

acsAP::~acsAP() { }  // destructor

void acsAP::init() 
{
    for(int i=0; i<6; i++) {  // make sure all valves are off
        hal.gpio->pinMode( 50+i, HAL_GPIO_OUTPUT );
        hal.gpio->write( 50+i, 0 );
    }
    
} 

void acsAP::shutdown() 
{
    // turn off all valves and squib drivers
    for(int i=0; i<6; i++) {  // make sure all valves are off
        hal.gpio->pinMode( 50+i, HAL_GPIO_OUTPUT );
        hal.gpio->write( 50+i, 0 );
    }
    for(int i=0; i<6; i++) {
        hal.gpio->pinMode( 50+i, HAL_GPIO_OUTPUT );
        hal.gpio->write( 50+i, 0 );
    }
}

uint8_t acsAP::update( double ypCmd[2], double gyro[3], double rpy[3], double tof, double dtStep, int printon )
{
    // return;
    firesrm   = 0;
    firepara  = 0;
    int armed = 1;
    
    // ignite motor by setting pin(?) high
    if( !motoIgn ) {
        hal.gpio->pinMode( 50, HAL_GPIO_OUTPUT );
        if( (tof > tSrm) && (tof <= tSrm+durSrm) ) {
            hal.gpio->write( 51, armed ); // red output
            firesrm = 16;                 // for logging
        } else {
            hal.gpio->write( 51, 0 );
        }
    }
        // deploy parachute by setting pin(?) high
    if( !paraDep ) {
         hal.gpio->pinMode( 51, HAL_GPIO_OUTPUT );
         if( (tof > tPara) && (tof <= tPara+durPara) ) {
            hal.gpio->write( 50, armed ); // white output
            firepara = 32;                // for logging
         } else {
            hal.gpio->write( 50, 0 );             
         }
    }
      
    double fAvail = tankF0 + gasUsed * tankSlope ;
 
    // forward gain and feedback gain
    double wn     = fAvail * omega0 / tankF0;
    double momArm = xCg - xAcs;
    double kfore  = wn * wn * iyy / momArm;
    double kback  = 2.0L * zeta / wn;
/*
    // yaw-pitch rate feedback 
    double ratefb[2] = { gyro[2] * kback,   // was gyro[0]
                        gyro[1] * kback };
    // test pattern ypcmd was computed here...moved to INS_gene                    
    double fyzCmd[2] = { -(ypCmd[0] - rpy[2] - ratefb[0]) * kfore, 
                       -(ypCmd[1] - rpy[1] - ratefb[1]) * kfore }; */
    
/*
        // yaw-pitch rate feedback 
        double ratefb[2] = { gyro[0] * kback,   // was gyro[0]
                             gyro[1] * kback };
        // test pattern ypcmd was computed here...moved to INS_gene                    
        double fyzCmd[2] = { -(ypCmd[0] - rpy[0] - ratefb[0]) * kfore, 
                             -(ypCmd[1] - rpy[1] - ratefb[1]) * kfore };
    } else { */
    double ratefb[2] = { gyro[2] * kback,   // was gyro[0]
                         gyro[1] * kback };
        // test pattern ypcmd was computed here...moved to INS_gene                    
    double fyzCmd[2] = { -(ypCmd[0] - rpy[2] - ratefb[0]) * kfore, 
                         -(ypCmd[1] - rpy[1] - ratefb[1]) * kfore };
                         
  //fyzCmd[0] = 0.0; // zero out y (yaw) command
                             
    // lessor of fAvail and acsMax
    // doubl>e val = math::min( fAvail, maxACS );
    double val = fAvail;
    if(maxACS < fAvail) {
        val = maxACS;
    }

    // dutyCycle
    double dc[2] = { fyzCmd[0] / val, fyzCmd[1] / val };

    // limit duty cycle (dc) to 1 (100%)
    if(dc[0]>1.0) dc[0] = 1.0;
    if(dc[0]<-1.0) dc[0] = -1.0;
    if(dc[1]>1.0) dc[1] = 1.0;
    if(dc[1]<-1.0) dc[1] = -1.0;
    
    // convert 2 bi commands to 4 uni valve commands
    //pos nozzles, 1 & 2
    double posCmd[2] = { (dc[0] > 0)*dc[0], (dc[1] > 0)*dc[1]}; 
    //neg nozzles, 3 & 4
    double negCmd[2] = {-(dc[0] < 0)*dc[0],-(dc[1] < 0)*dc[1]}; 
    
    // goes to integrator
    double dcCmd[4] = { posCmd[0], posCmd[1], negCmd[0], negCmd[1] }; 
         
    //uint8_t tint = (int)( tof * 5 );
    //uint8_t testVal = tint % 2; // find even times

    // PWM
    int pwmCmd[6] = {0, 0, 0, 0, 0, 0};
    
    bool openloop = false;
    if( openloop ) {
        float longperiod = 1.980;
        float shortperiod = 0.020;
        float t0 = 2.0;
        float t1 = t0+shortperiod; // go neg
        float t2 = t1+longperiod;
        float t3 = t2+shortperiod; // stop
        float t4 = t3+longperiod;
        float t5 = t4+shortperiod; // go back
        float t6 = t5+longperiod;
        float t7 = t6+shortperiod; // stop
        if(tof>t0 && tof<=t1) pwmCmd[1] = 1;
        if(tof>t2 && tof<=t3) pwmCmd[3] = 1;
        if(tof>t4 && tof<=t5) pwmCmd[3] = 1;
        if(tof>t6 && tof<=t7) pwmCmd[1] = 1;
        for(int i=0; i<4; i++) {
            hal.gpio->pinMode( 52+i, HAL_GPIO_OUTPUT );
            hal.gpio->write( 52+i, pwmCmd[i] );
        }
    } else {
        for(int i=0; i<4; i++) {    //4
            pwmCmd[i]=0;
        }
        
        if( toneCount < toneQty ) {   // 50*0.002 = 
            if( true ) {   // for lab test
                // create tone of 50 pulses (12 each noz)
                // to indicate "Ready"
                pwmCmd[testnoz]=1;    //1
                testnoz++;
                if( testnoz > 3 ) {   //3
                    testnoz = 0;
                }
            } else {  // for flight test 1
                pwmCmd[0] = 1;
                pwmCmd[1] = 1;
                pwmCmd[2] = 1;
                pwmCmd[3] = 1;
            }
            toneCount++;
        } else {
            if( tof > 0.020 ) { // 3/20/2019
                for(int i=0; i<4; i++) {
                    pwmCmd[i] = (int)( dcCmdInt[i] > dcAchInt[i] ); // 0 or 1, output to valves
                } // end of for
            } // end of if
        }
        
   //   pwmCmd[0] = 0;  // turn off specific valves // vlv1 = BR (pos pitch)
   //     pwmCmd[1] = 0;  // valve #2 (of 4) not working
   //   pwmCmd[2] = 0;  //                          // vlv3 = W (neg pitch)
   //     pwmCmd[3] = 0;  // so turn off #4 also    

        for(int i=0; i<4; i++) {                         // 4
            hal.gpio->pinMode( 52+i, HAL_GPIO_OUTPUT );  //52
            hal.gpio->write( 52+i, pwmCmd[i] );          //52
        }
    }
    uint8_t cmdret = pwmCmd[0]*1 + pwmCmd[1]*2 + pwmCmd[2]*4 + pwmCmd[3]*8 + firesrm + firepara;
    
    // sum all valve commands to integrate for gas usage estimate
    // (count how many valves are open on this cycle)
    double sumCmds = pwmCmd[0] + pwmCmd[1] + pwmCmd[2] + pwmCmd[3]; \

    // update integrators
    for(int i=0; i<4; i++) {
        dcCmdInt[i] += dtStep * dcCmd[i];
        dcAchInt[i] += dtStep * pwmCmd[i];
    }
    
    if( printon ) {
        hal.console->printf("t:%6.3f: ", tof );
        hal.console->printf("%6.2f %3d %6.2f %6.2f : ",
            dcCmd[0], pwmCmd[0], dcCmdInt[0], dcAchInt[0] );
        hal.console->printf("%6.2f %3d %6.2f %6.2f",
            dcCmd[2], pwmCmd[2], dcCmdInt[2], dcAchInt[2] );
        hal.console->printf(" : %1d %1d %1d %1d", pwmCmd[0], pwmCmd[1], pwmCmd[2], pwmCmd[3] );  
    }
    
    // integrate sumCmds to find equiv total time on
    gasUsed += dtStep * sumCmds;
        
    return cmdret;

}
