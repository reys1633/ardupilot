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

void acsAP::shutdown() // turn off all valves and squib drivers
{
    for(int i=0; i<6; i++) {  // make sure all valves are off
        hal.gpio->pinMode( 50+i, HAL_GPIO_OUTPUT );
        hal.gpio->write( 50+i, 0 );
    }
}

uint8_t acsAP::update( double ypCmd[2], double gyro[3], double rpy[3], double time, double tof )
{
    // return;
    firesrm   = 0;
    firepara  = 0;
    
    
    
    int armed = 1;  ///////////////////////////////////////////////
        
    
    
    // ignite motor by setting pin(?) high
    hal.gpio->pinMode( iSRM, HAL_GPIO_OUTPUT );
//  if( ( tSrm <= tof) && (tof < tSrm+durSrm ) ) {
    
//  hal.console->printf(" acsAP time:%8.3f\n", time );
    
    if( ( tSrm <= time) && (time < tSrm+durSrm ) ) { // for autolaunch
        hal.gpio->write( iSRM, armed ); // red output was 51
        firesrm = 16;                 // for logging
    } else {
        hal.gpio->write( iSRM, 0 );
    }
    // deploy parachute by setting pin(?) high
    hal.gpio->pinMode( iPARA, HAL_GPIO_OUTPUT );
    if( ( tPara <= tof ) && ( tof < tPara+durPara ) ) {
        hal.gpio->write( iPARA, armed ); // white output
        firepara = 32;                // for logging
    } else {
        hal.gpio->write( iPARA, 0 );             
    }
      
    double fAvail = tankF0 + gasUsed * tankSlope ;
 
    // forward gain and feedback gain
    double wn     = fAvail * omega0 / tankF0;
    double momArm = xCg - xAcs;
    double kfore  = wn * wn * iyy / momArm;
    double kback  = 2.0L * zeta / wn;

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
    double posCmd[2] = { (dc[0] > 0)*dc[0], (dc[1] > 0)*dc[1] }; 
    //neg nozzles, 3 & 4
    double negCmd[2] = {-(dc[0] < 0)*dc[0],-(dc[1] < 0)*dc[1] }; 
    
    /*
    //temp flip pos and neg
    //pos nozzles, 1 & 2
    double negCmd[2] = { (dc[0] > 0)*dc[0], (dc[1] > 0)*dc[1]}; 
    //neg nozzles, 3 & 4
    double posCmd[2] = {-(dc[0] < 0)*dc[0],-(dc[1] < 0)*dc[1]}; 
    */
    
    // goes to integrator
    double dcCmd[4] = { posCmd[0], posCmd[1], negCmd[0], negCmd[1] }; 
         
    //uint8_t tint = (int)( tof * 5 );
    //uint8_t testVal = tint % 2; // find even times

    // PWM
    int pwmCmd[4] = {0, 0, 0, 0}; //, 0, 0};
    
    //bool openloop = false;
    if( OPENLOOP ) {
        float off = 0.01;
        float on  = 0.99;
        float t0 = 1.0; //2.0;
        float t1 = t0+on; // go neg
        float t2 = t1+off;
        float t3 = t2+on; // stop
        float t4 = t3+off;
        float t5 = t4+on; // go back
        float t6 = t5+off;
        float t7 = t6+on; // stop
        if( t0 <= tof && tof < t1 ) pwmCmd[0] = 1; // 1.98 to 2
        if( t2 <= tof && tof < t3 ) pwmCmd[1] = 1; // 3.98 to 4
        if( t4 <= tof && tof < t5 ) pwmCmd[2] = 1; // 5.98 to 6
        if( t6 <= tof && tof < t7 ) pwmCmd[3] = 1; // 7.98 to 8
        //hal.console->printf(" open loop \n");
        //for(int i=0; i<4; i++) {
        //    hal.gpio->pinMode( 52+i, HAL_GPIO_OUTPUT );
        //    hal.gpio->write( 52+i, pwmCmd[i] );
        //}
    } else {
        for(int i=0; i<4; i++) {    //4
            pwmCmd[i]=0;
        }
        
        if( toneCount < toneQty ) {   // 50*0.002 = 
            // create tone of 50 pulses (12 each noz)
            // to indicate "Ready"
            pwmCmd[ testnoz ] = 1;
            testnoz++;
            if( testnoz > 3 ) {
                testnoz = 0;
            }
            toneCount++;
        } else {
            // normal, active path
            if( tof > 0.020 ) { // 3/20/2019 clear launcher
                for(int i=0; i<4; i++) {
                    pwmCmd[i] = (int)( dcCmdInt[i] > dcAchInt[i] ); // 0 or 1, output to valves
                } // end of for
            } // end of if clear launcher
        } // end of if tone
    } // end of if openloop
    
    //pwmCmd[0] = 0;  // #2
    //pwmCmd[1] = 0;  // #3
    //pwmCmd[2] = 0;  // #4
    //pwmCmd[3] = 0;  // #1
    
    //if( true ) {
    // TEMPORARY FIX   TEMPORARY FIX   TEMPORARY FIX
    /*int pwmCmdb[4];
    pwmCmdb[0] = pwmCmd[0]; // trig by +y, #2 valve
    pwmCmdb[1] = pwmCmd[1]; // trig by +p, #3 valve
    pwmCmdb[2] = pwmCmd[2]; // trig by -y, #4 valve
    pwmCmdb[3] = pwmCmd[3]; */
    // TEMPORARY FIX   TEMPORARY FIX   TEMPORARY FIX
    //}
    
    for(int i=0; i<4; i++) {                         // 4
        hal.gpio->pinMode( iVALV+i, HAL_GPIO_OUTPUT );  //52 now 50
        hal.gpio->write( iVALV+i, pwmCmd[i] );          //52
    }

    // return value for logging
    uint8_t cmdret = pwmCmd[0]*1 + pwmCmd[1]*2 + pwmCmd[2]*4 + pwmCmd[3]*8 + firesrm + firepara;
    
    // sum all valve commands to integrate for gas usage estimate
    // (count how many valves are open on this cycle)
    double sumCmds = pwmCmd[0] + pwmCmd[1] + pwmCmd[2] + pwmCmd[3]; \

    // update integrators
    for(int i=0; i<4; i++) {
        dcCmdInt[i] += dtStep * dcCmd[i];
        dcAchInt[i] += dtStep * pwmCmd[i];
    }
    
    // integrate sumCmds to find equiv total time on
    gasUsed += dtStep * sumCmds;
        
    return cmdret;

}
