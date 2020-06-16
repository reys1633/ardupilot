/* 10012019
 * ypCmd[2] = {0.0, 20.0}
 * squib 100/200 => 150/300 mg
 * ??? bias removal, 
 * flipping polarity inacsAP.cpp // polarity ok
 * 1004 v008bt added acs.shutdown call to post 
 * no tone, toneQty 64 => 16, bench test
 * 
 * */
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InertialSensor/AP_InertialSensor_Invensense.h>
//#include <AP_InertialSensor/AP_InertialSensor_LIS331.h>
//#include <AP_Notify/Buzzer.h>
#include <stdlib.h>
#include <string.h>
#include "missileRRT.h"
#include "acsAP.h"
#define R2D 57.2958
#define D2R 0.01745329251
#define USE_SD TRUE

#if USE_SD
#include "AP_HAL_ChibiOS/sdcard.h"
#ifndef HAL_STORAGE_FILE
// using SKETCHNAME allows the one microSD to be used
// for multiple vehicle types
#define LOG_FILE_NAME "/APM/" "imudata" ".dat"
#endif
#endif

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static AP_InertialSensor ins;

// board specific config************************************************
static AP_BoardConfig BoardConfig;

static acsAP acs;

//static AP_InertialSensor_LIS331 lis;

//static Buzzer buzzer;

void setup(void);
void loop(void);
void removeBias(void);
void integrate(void);
void logToMem(void);
void xferToSD(void);

//void Log_Write_IMUA(void);

bool stopex = false;
bool launched = false;
uint8_t vCmd = 0;
double tof  = 0.0;
double tlau = -1.0;
//double time   = -1.0;
//double tStart = -1.0; // time when execution begins
double accel[3];
double gyro[3];
Vector3f acc;
Vector3f gyr;
//uint fd;
//uint ft;
//uint32_t count = 0;
uint32_t memcnt = 0;
uint32_t outcnt = 0;
uint32_t execnt = 0;
int firesrm  = 0; // 0 or 16
int firepara = 0; // 0 or 32
Vector3f extern raw_accel;
//Vector3f extern raw_accel_400g;
#define LOW_PASS_FREQ  50 // Hz
#define LOW_PASS_ZETA 0.7
#if LOW_PASS_FREQ > 1
float filtw = 6.28 * LOW_PASS_FREQ;
float filtw2 = filtw * filtw;
float filtfb = 2*LOW_PASS_ZETA/filtw;
#endif
// yaw pitch attitude cmd, deg 
double ypCmd[2] = {0.0, 0.0 }; //0.349}; // 20 deg pitch
// Options
int printacs  = 0;

/*enum Color { red, green, blue };
Color r = red; */

enum modes { CAL, PRE, FLT, POST };
modes mode = CAL;
//uint nn = tofmax * freq * 13; // 1500
//float_t stor[20000]; // 19500
//uint maxrec = 13000+100;    // 1000*13
float_t stor[28200]; // 1 sec @1khz record all raw INS data
//float_t stor[32600]; // 1 sec @1khz record all raw INS data
//float_t stor[29542]; // 1 sec @1khz record all raw INS data
//double_t stor[28200];
//  double gyr0[3];
//  doulbe gyr1[3];
double gyrof[3] = {0.0, 0.0, 0.0};
double rpy[3] = {0.0, 0.0, 0.0}; // roll, pitch, yaw meas
double vel[3] = {0.0, 0.0, 0.0};     // velocity m/s meas
double pos[3] = {0.0, 0.0, 0.0};     // position x, y, z (+
uint32_t calCnts = 0;
double gyrAccum[3] = {0.0, 0.0, 0.0};
double gyrBia[3] = {0.0, 0.0, 0.0};

void setup(void) {
    hal.console->printf("Setting up the board...\n");
    BoardConfig.init();  // setup any board specific drivers
    double boardTime = AP_HAL::micros64()/1000000.0L;
    hal.console->printf("Board setup finished in %f\n", boardTime);
    hal.console->printf("Setting up the INS...\n");
    ins.init( freq );
    double insTime = AP_HAL::micros64()/1000000.0L;
    hal.console->printf("INS setup finished in %f\n", insTime);
    //lis.begin();
    hal.console->printf("waiting %d sec to charge supercaps\n", chargdelay/1000 );
    hal.scheduler->delay( chargdelay );
    
    hal.console->printf("tmax = %f\n", tmax);
    
    for(int i=1; i<5; i++){
        hal.console->printf("Charging %d\n", 5-i);
        hal.scheduler->delay( 1000 );
    }
    acs.init();

#if USE_SD
//  sdcard_init();
//  fd = open( "/APM/ins2.dat", O_WRONLY|O_CREAT );
//  ft = open( "/APM/test.txt", O_WRONLY|O_CREAT );
//  hal.scheduler->delay(1);
#endif
    
    //acs.shutdown();
    /*for(int i=0; i<6; i++) {  // make sure all valves are off
        hal.gpio->pinMode( 50+i, HAL_GPIO_OUTPUT );
        hal.gpio->write( 50+i, 0 );
    }*/
} // of setup

void removeBias(void) {
    for(uint8_t i=0; i<2; i++) {
      //gyro[i] -= gyrBia[i];
        gyro[i] = gyro[i]; // - gyrBia[i];
    }
}

void integrate(void) {
// Integrate states
    for(uint8_t i=0; i<3; i++) {
        rpy[i] += dtStep * gyrof[i];
        pos[i] += dtStep * vel[i];
        vel[i] += dtStep * accel[i];
    }
}

void logToMem(void){
    stor[ memcnt ] = tof;
    stor[memcnt+1] = gyrof[1] * R2D; // pitchrate
    stor[memcnt+2] = gyrof[2] * R2D;
    stor[memcnt+3] = rpy[1] * R2D;   // pitch
    stor[memcnt+4] = rpy[2] * R2D;
    stor[memcnt+5] = vCmd;
    memcnt += 6;
    //hal.console->printf("logToMem tof%f memcnt%d\n", tof, memcnt);
}

void xferToSD(void){
    hal.console->printf("dumping stor data to SD Card\n");
    sdcard_init();
    float temp;
    int fd = -1;
    //fd = open( "/APM/v001.dat", O_WRONLY|O_CREAT );//09302019_0936
    //fd = open( "/APM/v002.dat", O_WRONLY|O_CREAT );//09302019_1026
    //fd = open( "/APM/v004.dat", O_WRONLY|O_CREAT );//09302019_1026
    //120psi, ext power, small maneuvers, watch LEDs
    //fd = open( "/APM/v005.dat", O_WRONLY|O_CREAT );//10022019_1449
    //fd = open( "/APM/v006ft.dat", O_WRONLY|O_CREAT );//10032019_1100
    //fd = open( "/APM/v008bt.dat", O_WRONLY|O_CREAT );//10042019_1130
    //fd = open( "/APM/v008ft.dat", O_WRONLY|O_CREAT );//10032019_1230 no one
    fd = open( "/APM/v009bt.dat", O_WRONLY|O_CREAT );//10032019_1322
    
    
    uint8_t nThisLine = 0;
    for( int k=0; k<memcnt; k++ ) { // memcnt
        temp = stor[k];
        write( fd, &temp, 4 );
        hal.console->printf("%8.3f ", temp );
        if( nThisLine++ >= 5 ) 
        { 
            hal.console->printf("\n");
            nThisLine = 0;
            hal.scheduler->delay(4);
        }
    }
    hal.console->printf("Closing log file\n");
    close( fd );
    while( true ) { // do forever
        continue;
    }
}

void loop(void) {

    // flush any user input
    while ( hal.console->available() ) {
        hal.console->read();
    }
    ins.update();  // clear out any existing samples from ins
    double time   = -1.0; // 0 when loop starts
    #if LOW_PASS_FREQ > 1 // hz
        float gyrdot[3]={0.0, 0.0, 0.0};
        float gyrdd[3];
    #endif
        
    double starttime = AP_HAL::micros64()/1000000.0;
    
    while( !stopex ) {
        time = AP_HAL::micros64()/1000000.0 - starttime;
        ins.wait_for_sample();
        ins.update();
        acc = ins.get_accel( 0 ); 
        gyr = ins.get_gyro( 0 );
        accel[2] = acc.x; // pix x ==> msl z
        accel[1] = acc.y; // pix y ==> msl y (no change
        accel[0] = acc.z; // pix z ==> msl x
        // if( calCnts < 500 ) {
        gyro[2] = gyr.x; // pix roll  ==> msl yaw
        gyro[1] = gyr.y; // pix pitch ==> msl pitch
        gyro[0] = gyr.z; // pix yaw   ==> msl roll: 
        for(uint8_t i=0; i<3; i++) {
            #if LOW_PASS_FREQ > 1                
                // low pass filter
                gyrdd[i] = ( gyro[i]-gyrof[i]-gyrdot[i]*filtfb ) * filtw2;
                gyrof[i] = gyrof[i] + dtStep * gyrdot[i];
                gyrdot[i] = gyrdot[i] + dtStep * gyrdd[i];
            #else
//              accelf[i] = accel[i];
                gyrof[i] = gyro[i];
            #endif
            // Integrate states
            //rpy[i] += dtStep * gyrof[i];
            //pos[i] += dtStep * vel[i];
            //vel[i] += dtStep * accel[i];
        }
        
        // marty michelle johnson 5195068
        
        // sequence control and one-time algorithms
        int openLoop = 1;
        
        switch( mode )
        {
            case CAL  :
                if(calCnts >= 500) { 
                    mode = PRE; 
                    hal.console->printf(" mode=>Pre\n"); 
                    /* Buzzer test from MMLPlayer.cpp
                    float frequency = 1000.0;
                    float volume = 1000.0;
                    float duration = 1.0;
                    float _note_duration_us = duration*1e6;
                    
                    hal.util->toneAlarm_set_buzzer_tone(frequency, volume, _note_duration_us/1000U);}
                    */
                    
                    //buzzer.play_pattern(buzzer.SINGLE_BUZZ);
                break;
            case PRE : 
                if( accel[0] < gThreshold || openLoop ) { //|| true 
                    tlau = time;
                    hal.console->printf(" launch detected at %f\n", tlau);
                    launched = true;
                    mode = FLT;
                }
                break;
                   
            case FLT : 
                if( tof >= tofmax ) { // || time > tmax
                    mode = POST; 
                }
                break;
                
            case POST : 
                break;
        } // switch( mode )
        
        
/*        if( time > tmax ) {
            stopex = true;
            hal.console->printf("stopping (tmax)\n");
            acs.shutdown();
        } */
        
        // execute control
        switch( mode ) {
            case CAL:
                for(uint8_t i=0; i<3; i++) {
                    gyrAccum[i] += gyro[i];
                    gyrBia[i] = gyrAccum[i]/calCnts;
                }
                calCnts++;
                break;
                
            case PRE:
                /////////////////////////////////////////////////////
                /////////////////////////////////////////////////////
                vCmd = acs.update( ypCmd, gyrof, rpy, tof, dtStep, openLoop );//printacs
                /////////////////////////////////////////////////////
                /////////////////////////////////////////////////////
                logToMem();
                break;
                
            case FLT:
                tof = time - tlau;
                removeBias();
                integrate();
                /////////////////////////////////////////////////////
                /////////////////////////////////////////////////////
                vCmd = acs.update( ypCmd, gyrof, rpy, tof, dtStep, printacs );
                /////////////////////////////////////////////////////
                /////////////////////////////////////////////////////
                logToMem();
                break;
                
            case POST:
                acs.shutdown();
                xferToSD();
                stopex = true;
                hal.scheduler->delay(1000);
                break;
        }  // switch case
        
        if( outcnt++ % 50 != 0 ) {  
            continue;
        }
        hal.console->printf("t%6.3f  tof%6.3f  q%7.3f  r%7.3f  p%7.3f  y%7.3f  gb1 %7.3f  gb2%7.3f cnt%6lu  acc%6.2f\n", 
            time, tof, gyrof[1], gyrof[2], rpy[1]*R2D, rpy[2]*R2D, gyrBia[1], gyrBia[2], outcnt, accel[0] );
    } // while( !stopex )
}// end of "loop" method
//////////////////////////////////////////////////////    
//////////////////////////////////////////////////////    


/////////////////////////////////////////////////////    
/*    while( !stopex ) {      // loop forever
        time = AP_HAL::micros64()/1000000.0;
        
        if( tof < tofmax ) {    // time < tmax &&
            
            if( tlau < 0.0 ) {  // pre-launch, one time code
                if( accel[0] < gThreshold || true ) {
                    tlau = AP_HAL::micros64()/1000000.0L;  
                    hal.console->printf(" launch detected at %f\n", tlau);
                    launched = true;
                }
            }
                
            // wait until we have a sample
            ins.wait_for_sample();
            ins.update();

            if( launched ) {
                tof = time - tlau;
            } else {
                tof = 0.0;
            }
            acc = ins.get_accel( 0 ); 
            gyr = ins.get_gyro( 0 );
             
            accel[2] = acc.x; // pix x ==> msl z
            accel[1] = acc.y; // pix y ==> msl y (no change
            accel[0] = acc.z; // pix z ==> msl x
            
            if( calCnts < 500 ) {
                gyro[2] = gyr.x; // pix roll ==> msl yaw
                gyro[1] = gyr.y; // pix pitch ==> msl pitch (no change)
                gyro[0] = gyr.z; // pix yaw ==> msl roll:  0.000 pc
                gyrAccum[0] += gyro[0];
                gyrAccum[1] += gyro[1];
                gyrAccum[2] += gyro[2];
                gyrBia[0] = gyrAccum[0]/calCnts;
                gyrBia[1] = gyrAccum[1]/calCnts;
                gyrBia[2] = gyrAccum[2]/calCnts;
                calCnts++;
            } else {
                gyro[2] = gyr.x - gyrBia[2]; // pix roll ==> msl yaw
                gyro[1] = gyr.y - gyrBia[1]; // pix pitch ==> msl pitch (no change)
                gyro[0] = gyr.z - gyrBia[0]; // pix yaw ==> msl roll:  0.000 pc
            }
             
            for(uint8_t i=0; i<3; i++) {
//              gyro[i] = gyro[i] - gyrbia[i];
//              accel[i] = accel[i] - accbia[i];
#if LOW_PASS_FREQ > 1                
                // low pass filter
              accdd[i] = ( accel[i]-accelf[i]-accdot[i]*filtfb ) * filtw2;
                accelf[i] = accelf[i] + dtStep * accdot[i];
                accdot[i] = accdot[i] + dtStep * accdd[i]; 
                gyrdd[i] = ( gyro[i]-gyrof[i]-gyrdot[i]*filtfb ) * filtw2;
                gyrof[i] = gyrof[i] + dtStep * gyrdot[i];
                gyrdot[i] = gyrdot[i] + dtStep * gyrdd[i];
#else
//              accelf[i] = accel[i];
                gyrof[i] = gyro[i];
#endif
//              accelf[i] = accel[i]; // can't filter, we want peak
                // Integrate states
                rpy[i] += dtStep * gyrof[i];
                pos[i] += dtStep * vel[i];
                vel[i] += dtStep * accel[i];
            }

            ////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////
            vCmd = acs.update( ypCmd, gyrof, rpy, tof, dtStep, printacs );
            ////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////
            
            //hal.console->printf("writing data to stor %d\n",outcnt);
            
            if( mode == FLT ) {  // launched
                stor[ outcnt ] = tof;
                stor[outcnt+1] = gyrof[1] * R2D; // pitchrate
                stor[outcnt+2] = gyrof[2] * R2D;
                stor[outcnt+3] = rpy[1] * R2D;   // pitch
                stor[outcnt+4] = rpy[2] * R2D;
                stor[outcnt+5] = vCmd;
                outcnt += 6;
            }

            if( execnt % 10 ) {
                hal.console->printf("t %8.3f gf1 %8.3f gf2 %8.3f p %8.3f y %8.3f gb1 %8.3f gb2 %8.3f\n", 
                    tof, gyrof[1], gyrof[2], rpy[1], rpy[2], gyrBia[1], gyrBia[2] );
            }
            //hal.console->printf("Time: %f Accel: %f Stor: %f\n", time, accel[1], &stor);
            // hal.console->printf("%f, %lu\n", time, count);
            execnt++;

        } else {  // time !< tmax (should stop now)
    
            hal.console->printf("stopping\n");
//          ap.shutdown();

            hal.scheduler->delay(100);
            stopex = true;
//            hal.console->printf("    tof     gfx     gfy     gfz     rol ");
//            hal.console->printf("    pit     yaw     afx     afy     afz    vcmd   ypcmd\n");
//            hal.console->printf("    tof     gx1     gy1     gz1     ax1     ay1     az1 ");
//            hal.console->printf("    gx2     gy2     gz2     ax2     ay2     az2\n");

            // dump stored data to SD Card
            sdcard_init();
            
            float temp;
            int fd = -1;
           
            fd = open( "/APM/ins4.dat", O_WRONLY|O_CREAT );
            for( int k=0; k<outcnt; k++ ) { // outcnt
//              hal.console->printf("%7d %12.3f \n", k, stor[k] );
                temp = stor[k];
                write( fd, &temp, 4 );
//              hal.console->printf("%8.3f ", stor[k] );
            }
            hal.console->printf("Closing log file\n");
            close( fd );
            hal.scheduler->delay(1000);
//          fd = open( "/APM/ins3.dat", R_WRONLY );
            
            // dump to console via mavlink
            int cnt = 1;
            for(int k = 0; k<outcnt; k++) {
                hal.console->printf("%11.3f", stor[k] );
                if( cnt >= 6 ) {
                    cnt = 0;
                    hal.scheduler->delay( 1 );
                    hal.console->printf("\n");
                }
                cnt += 1; // print every 10th line
            } 
            
/ *            // read SD and write data to console via mavlink
            hal.console->printf("outcnt= %7ld \n", outcnt );
            fd = open( "/APM/ins3.dat", O_RDONLY );
            float anum;

//          for( int k=0; k< 100; k++ ) {
//		    while( fd != EOF ) {
            int thiscnt = 0;
            while(thiscnt<outcnt) {
                for(int j=0; j<14; j++ ) { // was 12
                    read( fd, &anum, 4 );
                    //hal.console->printf("%7.3f ", anum );
                }
                thiscnt = thiscnt+14;
                hal.scheduler->delay( 5 );
                //hal.console->printf("\n");
            }* /
            hal.scheduler->delay( 1000 );
            hal.console->printf("Finished -- Please unplug Pix\n");
            hal.scheduler->delay(2000);
            hal.console->printf("Finished -- Please unplug Pix\n");
        } // tof > tmax// end of if true
    }
    //hal.console->printf("Finished"); */
//}// end of "loop" method

AP_HAL_MAIN();
