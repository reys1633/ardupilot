//
// Simple test for the AP_InertialSensor driver.
// modified to be framework for Risk Reduction Test
// INS driver, Autopilot and ACS Controller
// backup to ngc email 1/17/2018

// Pitch Plane Functional Test at AR 1/xxxxxxx
// 2D Functional Test:
// 3/20/2019
// moved INS_generic line 310 if( tof > 0.020 )
// applied to pwmCmd acsAP line 154 
// removed time condi

//#include "missileRRT.h"
#include "acsAP.h"
//#include "attcmd.h"
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
//#include <StorageManager/StorageManager.h>
#include <stdlib.h>
#include <string.h>
//#include <iostream>

#define R2D 57.2958

#define USE_SD TRUE

#if USE_SD
#include "AP_HAL_ChibiOS/sdcard.h"
#ifndef HAL_STORAGE_FILE
// using SKETCHNAME allows the one microSD to be used
// for multiple vehicle types
#define LOG_FILE_NAME "/APM/" "imudata" ".dat"
#endif
#endif

acsAP ap; // construct the autopilot

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static AP_InertialSensor ins;

// board specific config
static AP_BoardConfig BoardConfig;

void setup(void);
void loop(void);

void Log_Write_IMUA(void);
    
//enum mode { calIns, noztest, stab, stopex };
//mode mymode  = calIns;
bool stopex = false;

double accel[3];
double gyro[3];
double gyrof[3];
double accelf[3];
double fAvail = 0.0;
uint64_t count = 0;
long int cnt = 0;
uint16_t nFile = 0;
uint32_t uStart = 0;
uint32_t cycleEtu = 0;
Vector3f acc;
Vector3f gyr;
uint8_t vCmd = 0;
    
bool launched = false;
    
double tLau = 0.0;

// Options
int printacs  = 0;
int printthis = 0;
int fd;   // for log file
        
double ypCmd[2] = {0.0, 0.0}; // yaw pitch attitude cmd, deg 

#define LOW_PASS_FREQ  50 // Hz
#define LOW_PASS_ZETA 0.7
#if LOW_PASS_FREQ > 1
float filtw = 6.28 * LOW_PASS_FREQ;
float filtw2 = filtw * filtw;
float filtfb = 2*LOW_PASS_ZETA/filtw;
#endif

double tStartFrm = 0.0;
double tStart = -1.0; // time when execution begins
double tFinish;

// moved these up from inside "loop"
double accbia[3];
double gyrbia[3];
double tof  = 0.0;
double accsum[3]= {0.0, 0.0, 0.0};
double gyrsum[3]= {0.0, 0.0, 0.0};
 

void setup(void) {
    BoardConfig.init();  // setup any board specific drivers
    ins.init( freq );
    ap.init();
#if USE_SD
    sdcard_init();
    fd = open( "/APM/rrtlog.dat", O_WRONLY|O_CREAT );
    hal.scheduler->delay(1);
#endif
} // of setup


void loop(void)
{
//    bool launched = false;
//    double tLau = 0.0;
   
    
    
//    double accbia[3]; // moved up higer
//    double gyrbia[3];
    
    
    double rpy[3]; // = {0.0, 0.0, 0.0}; // roll, pitch, yaw meas
    double vel[3] = {0.0, 0.0, 0.0};     // velocity m/s meas
    double pos[3] = {0.0, 0.0, 0.0};     // position x, y, z (+
#if LOW_PASS_FREQ > 1 // hz
    // low pass filter
//  float accdot[3];
//  float accdd[3];
    float gyrdot[3];
    float gyrdd[3];
/*  float filtw = 6.28 * LOW_PASS_FREQ;
    float filtw2 = filtw * filtw;
    float filtzeta = 0.7;
    float filtfb = 2*filtzeta/filtw; */
#endif
    
    // flush any user input
    while ( hal.console->available() ) {
        hal.console->read();
    }

    ins.update();  // clear out any existing samples from ins

    count = 0;
    
    while( !stopex ) {      // loop forever
        if( tStart < 0.0 ) {
            tStart = AP_HAL::micros64()/1000000.0L;
        }
        double time = AP_HAL::micros64()/1000000.0 - tStart;
//      hal.console->printf( "time:%8.3f\n", time ); 
        
        if( tof > tofmax || time > tmax ) {
            ap.shutdown();
            stopex = true;
#if USE_SD
            hal.console->printf("Closing log\n"); 
            close( fd );
            
            // write out the data
            fd = open( "/APM/rrtlog.dat", O_RDONLY );
            float anum;
            hal.console->printf("    tof     gfx     gfy     gfz     rol ");
            hal.console->printf("    pit     yaw     afx     afy     afz    vcmd   ypcmd\n");

            for( int k=0; k< 50; k++ ) {
//          while( fd != EOF ) {
                for(int j=0; j<12; j++ ) {
                    read( fd, &anum, 4 );
                    hal.console->printf("%7.3f ", anum );
                }
		//    	hal.scheduler->delay(1);
                hal.console->printf("\n");
            }
#endif     
            hal.console->printf("Finished -- Please unplug Pix\n");
            hal.scheduler->delay(2000);
            hal.console->printf("Finished -- Please unplug Pix\n");
        } // tof > tmax
        
        // wait until we have a sample
        ins.wait_for_sample();

        uStart = AP_HAL::micros(); // start frame timer
        //double tStart = uStart / 1000000.0;
 
        // read samples from ins
        ins.update();

        acc = ins.get_accel( 0 ); // int ii = 0; // which INS
        gyr = ins.get_gyro( 0 );
        
//        hal.console->printf("time= %5.3f  tof= %5.3f reading INS\n", time, tof );
        
        accel[2] = acc.x; // pix x ==> msl z
        accel[1] = acc.y; // pix y ==> msl y (no change
        accel[0] = acc.z; // pix z ==> msl x
        gyro[2] = gyr.x; // pix roll ==> msl yaw
        gyro[1] = gyr.y; // pix pitch ==> msl pitch (no change)
        gyro[0] = gyr.z; // pix yaw ==> msl roll:  0.000 pc
        
        for(uint8_t i=0; i<3; i++) {
//          gyro[i] = gyro[i] - gyrbia[i];
//          accel[i] = accel[i] - accbia[i];
#if LOW_PASS_FREQ > 1                
            // low pass filter
         /* accdd[i] = ( accel[i]-accelf[i]-accdot[i]*filtfb ) * filtw2;
            accelf[i] = accelf[i] + dtStep * accdot[i];
            accdot[i] = accdot[i] + dtStep * accdd[i]; */
            
            gyrdd[i] = ( gyro[i]-gyrof[i]-gyrdot[i]*filtfb ) * filtw2;
            gyrof[i] = gyrof[i] + dtStep * gyrdot[i];
            gyrdot[i] = gyrdot[i] + dtStep * gyrdd[i];
#else
            accelf[i] = accel[i];
            gyrof[i] = gyro[i];
#endif
            accelf[i] = accel[i]; // can't filter, we want peak
        }

        // originally designed for lab tests.  Now we want 
        // 1. sensors read all the time
        // 2. until launch, accumlate and average
        // 3. after cal, start calling acsAP so tone can be generated (tof=0)
        // 4. after launch detect, tof begins increasing
        // Sequence:
        // 1. Boot, cal
        // 2. Tone
        // 3. Detect Launch @ accelf[2] < -9.8L    INS_gen line 204
        // 4. Start control @ tof > 0.020          acsAP   line 152
        // 5. SRM Ign       @ tof > 0.050          acsAP   line  39
        //    if no SRM
        // 6. Parachute     @ tof > 1.5            acsAP   line  48
        //    if SRM present
        // 6. Parachute     @ tof > 3.0            acsAP   line  48
        
        if( !launched ) {  // pre launch
            tof = 0.0;
            
            // calibrate for a time defined by durCal
            if( time <= durCal ) {

                for(uint8_t i=0; i<3; i++) {
                    accsum[i] += accel[i];  // using raw sesnsor output
                    gyrsum[i] += gyro[i];   // using raw sesnsor output
                    accbia[i] = accsum[i] / count;
                    gyrbia[i] = gyrsum[i] / count;
                }
//                hal.console->printf("%7.3f %3d %7.3f %7.3f INS Cal\n", 
//                    time, count, accel[0], accbia[0] );
            }
            
//double const durCal    = 3.0;   // cycles for INS Cal
//double const cmdLau    = 4.1;   // scripted launch cmd time

            // detect launch event ... one time executiion
            if( accel[0] < gThreshold || time > cmdLau ) {  // time > durCal ||
               // while( 1 ) 
              //  {
                    //float pie = 3.14;
                    hal.console->printf("launch detected \n\n");
               // }
                
                launched = true;

                count = 0;
                cnt = 0;
            //  write IMU biases as first datapoint in log file
                float zero = 0.0;
                write( fd, &zero, 4 ); // tof 1
                
                float temp = gyrbia[0]*R2D;
                write( fd, &temp, 4 ); // 2
                temp = gyrbia[1]*R2D;
                write( fd, &temp, 4 ); // 3
                temp = gyrbia[2]*R2D;
                write( fd, &temp, 4 ); // 4
                
                write( fd, &zero, 4 ); // 5
                write( fd, &zero, 4 ); // 6
                write( fd, &zero, 4 ); // 7
                
                temp = accbia[0];
                write( fd, &temp, 4 ); // 8
                temp = accbia[1];
                write( fd, &temp, 4 ); // 9
                temp = accbia[2];
                write( fd, &temp, 4 ); // 10
                write( fd, &zero, 4 );   // vCmd // 11
                write( fd, &zero, 4 );   // pCmd // 12 //didn't do this unfortunately
                
//              hal.scheduler->delay( 500 ); // 2000
                
                tLau = time; //AP_HAL::micros() / 1000000.0;
            }
            
        } else {     // post launch
            
// remove bias from filtered values
//          for(uint8_t i=0; i<3; i++) {
//              gyrof[i] = gyrof[i] - gyrbia[i]; // can't do this because
//              accelf[i] = accelf[i] - accbia[i]; // gyrof is output of integrator
//              accelf[i] = accbia[i];
//          }
            
            // we have detected the eject impulse
            tof = time - tLau;
 
            // subtract the biases
            
            // moved here from acsAP.cpp   
            //attcmd::update( *ypCmd, tof );
            ypCmd[0] = 0.0;
            ypCmd[1] = 0.0;
/*          if( (0.25 < tof) && (tof < 0.5) ) {
                ypCmd[1] = -20.0;
            } else if( (0.75 < tof) && (tof<1.0) ) {
                ypCmd[1] = 20.0;
            } */
#if USE_SD
            //tof32 = AP_HAL::millis() - tLau32;
            float temp = (float)tof;
                    write( fd, &temp, 4 );  // 1
                    
            temp = gyrof[0]*R2D;
                    write( fd, &temp, 4 );  // 2
            temp = gyrof[1]*R2D;
                    write( fd, &temp, 4 );  // 3
            temp = gyrof[2]*R2D;
                    write( fd, &temp, 4 );  // 4
            temp = rpy[0]*R2D;
                    write( fd, &temp, 4 );  // 5
            temp = rpy[1]*R2D;
                    write( fd, &temp, 4 );  // 6
            temp = rpy[2]*R2D;
                    write( fd, &temp, 4 );  // 7
            temp = accelf[0];
                    write( fd, &temp, 4 );  // 8
            temp = accelf[1];
                    write( fd, &temp, 4 );  // 9
            temp = accelf[2];
                    write( fd, &temp, 4 );  // 10
            temp = vCmd;
                    write( fd, &temp, 4 );  // 11
            temp = ypCmd[1];
                    write( fd, &temp, 4 );  // 12   
#endif            

            // update integrators;
            for(uint8_t i=0; i<3; i++) {
                rpy[i] += dtStep * gyrof[i];
                pos[i] += dtStep * vel[i];
                vel[i] += dtStep * accelf[i];
            }
        } // if launched
        
        cycleEtu += AP_HAL::micros() - uStart;
        // print accel/gyro result every 500 cycles
        if ( (count % nConsole == 0) & !printacs ) {
            printthis = 1;
            uint16_t pct = (uint16_t)(cycleEtu/500/20.0); //2000*100%; //uStart;
            if( !launched ) {
                hal.console->printf("tim: %6.3f pct: %2u cnt: %5llu ", time, pct, count );
                hal.console->printf("gyro: %6.1f %6.1f %6.1f ", gyrbia[0]*R2D, gyrbia[1]*R2D, gyrbia[2]*R2D );
                hal.console->printf("acc: %6.2f %6.2f %6.2f ",  accbia[0],     accbia[1],     accbia[2] );
            } else {
                hal.console->printf("tof: %6.3f pct: %2u cnt: %5llu ", tof, pct, count );
                hal.console->printf("gyro: %6.1f %6.1f %6.1f ", gyrof[0]*R2D, gyrof[1]*R2D, gyrof[2]*R2D );
                hal.console->printf("acc: %6.2f %6.2f %6.2f ", accelf[0],    accelf[1],    accelf[2] );
            }
//          hal.console->printf("rpy: %6.2f %6.2f %6.2f ", rpy[0]*R2D, rpy[1]*R2D, rpy[2]*R2D );
            hal.console->printf("lau: %d ", launched );
//          hal.console->printf("pC: %4.0f ", ypCmd[1]*R2D );
            hal.console->printf("vlv: %1d %1d %1d %1d %1d %1d", 
                vCmd%2, (int)(vCmd/2.)%2, (int)(vCmd/4.)%2, (int)(vCmd/8.)%2, (int)(vCmd/16.)%2, (int)(vCmd/32.)%2 );
//          thisStartT = AP_HAL::micros(); // start time for this 1 sec period
            cycleEtu = 0;
        } // print counter
        
        
        if( time > durCal ) {     //if( tof > 0.020 ) {
        ///////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////
            vCmd = ap.update( ypCmd, gyrof, rpy, tof, dtStep, printacs );
        ///////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////
        }
        if( printthis | printacs ) {
            hal.console->printf("\n");
            printthis = 0;
        }
        count++;
        cnt++;
    } // end of while true
}  // end of "loop" method

AP_HAL_MAIN();

// line 251 accelf[0] to accelf[0]
//            if( accel[0] < gThreshold || time > cmdLau ) {  // time > durCal ||

