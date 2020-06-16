#include "attcmd.h"

attcmd::attcmd() { }

attcmd::~attcmd() { }

void update( float & ypCmd, double tof ) {

    float ypCmd[2];
    
    float k = 5.0;
    
    float t1 = 1.0*k;
    float t2 = 2.0*k;
    float t3 = 3.0*k;
    float t4 = 4.0*k;
    float t5 = 5.0*k;
    float t6 = 6.0*k;
    float t7 = 7.0*k;
    float t8 = 8.0*k;

    if( true ) {                    // lab test #2
        ypCmd[0] = 0.0;
        ypCmd[1] = 0.0;
        if        ( (tof > t1) && (tof < t2) ) {
            ypCmd[1] = 45.0 / R2D;           // pitch
        } else if ( (tof > t3) && (tof < t4) ) {
            ypCmd[1] = 45.0 / R2D;
        } else if ( (tof > t5) && (tof < t6) ) {
            ypCmd[1] = 20.0 / R2D;           // pitch
        } else if ( (tof > t7) && (tof < t8) ) {
            ypCmd[1] = 40.0 / R2D;
        }
        
    } else {                // lab test #1
        
        if        ( (tof > t1) && (tof < t2) ) {    // test 1/14
            ypCmd[1] = -10.0 / R2D;               // this was going 
        } else if ( (tof > t3) && (tof < t4) ) {    // to 0, not 1
            ypCmd[1] = 10.0 / R2D;                 
        } else if ( (tof > t5) && (tof < t6) ) {    // andrew may
            ypCmd[1] = -10.0 / R2D;               // be right
        } else if ( (tof > t7) && (tof < t8) ) {
            ypCmd[1] = 10.0 / R2D;
        }
    }
    return ypCmd;
}