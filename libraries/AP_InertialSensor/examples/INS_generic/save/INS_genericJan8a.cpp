//
// Simple test for the AP_InertialSensor driver.
// modified to be framework for Risk Reduction Test
// INS driver, Autopilot and ACS Controller
//
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
//#include <StorageManager/StorageManager.h>
//#include <string>
#include <stdlib.h>
#include <string.h>
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

#include "acsAP.h"
acsAP ap; // construct the autopilot

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static AP_InertialSensor ins;

// board specific config
static AP_BoardConfig BoardConfig;

void setup(void);
void loop(void);
void Log_Write_IMUA(void);
    
const uint16_t freq = 500;
const double dtStep = 0.002;
double tStart;
double tFinish;

double accel[3];
double gyro[3];
double fAvail = 0.0;
uint64_t count = 0;
long int cnt = 0;
uint16_t nFile = 0;
uint32_t uStart = 0;
        
Vector3f acc;
Vector3f gyr;
uint8_t vCmd = 0;
bool stopexec = false;

// Options
int printacs  = 0;
int printthis = 0;
int fd;   // for log file
    
//uint32_t anum = 12345;
//uint32_t bnum = 23456;
//float    cnum = 3.14159;

void setup(void)
{
    BoardConfig.init();  // setup any board specific drivers
    ins.init( freq );
    ap.init();
    
#if USE_SD
    sdcard_init();
    fd = open( "/APM/rrtlog.dat", O_WRONLY|O_CREAT );
#endif
  //sdcard_stop();
    
} // of setup

void loop(void)
{
    bool launched = false;
    double tLau = 0.0;
    double tof = 0.0;
#if USE_SD
  //uint32_t tLau32 = 0;
  //uint32_t tof32 = 0;
#endif
    double accsum[3]= {0.0, 0.0, 0.0};
    double gyrsum[3]= {0.0, 0.0, 0.0};
    double accbia[3];
    double gyrbia[3];
    double rpy[3]; // = {0.0, 0.0, 0.0}; // roll, pitch, yaw
    double vel[3] = {0.0, 0.0, 0.0};
    double pos[3] = {0.0, 0.0, 0.0};
    
    // flush any user input
    while ( hal.console->available() ) {
        hal.console->read();
    }

    ins.update();  // clear out any existing samples from ins

    count = 0;
    
    while( !stopexec ) {      // loop forever
        tof = AP_HAL::micros64()/1000000.0L;
        if( tof > 30.0 ) {
            ap.shutdown();
            stopexec = true;
#if USE_SD
            hal.console->printf("Closing log\n"); 
            close( fd );
            
            // write out the data
            fd = open( "/APM/rrtlog.dat", O_RDONLY );
            float anum;
            while( fd != EOF ) {
                for(int j=0; j<11; j++ ) {
                    read( fd, &anum, 4 );
                    hal.console->printf("%6.3f ", anum );
                }
                hal.console->printf("\n");
            }
#endif     
            hal.console->printf("Finished -- Please unplug Pix\n");
            hal.scheduler->delay(2000);
            hal.console->printf("Finished -- Please unplug Pix\n");
        }
        // wait until we have a sample
        ins.wait_for_sample();

        uStart = AP_HAL::micros();
        //double tStart = uStart / 1000000.0;
 
        // read samples from ins
        ins.update();

     // int ii = 0; // which INS
        acc = ins.get_accel( 1 );
        gyr = ins.get_gyro( 1 );
        accel[0] = acc.x;
        accel[1] = acc.y;
        accel[2] = acc.z;
        gyro[0] = gyr.x;
        gyro[1] = gyr.y;
        gyro[2] = gyr.z;

        if( !launched ) {  // pre launch
            tof = 0.0;
            // calibrate for a number of samples or a 2 gee pulse
            for(uint8_t i=0; i<3; i++) {
                accsum[i] += accel[i];
                gyrsum[i] += gyro[i];
                accbia[i] = accsum[i] / count;
                gyrbia[i] = gyrsum[i] / count;
            }
            // detect launch event
            if( count > 1999 || accel[2] < -14.8L ) {
                launched = true;
                tLau = AP_HAL::micros() / 1000000.0;
#if USE_SD
              //tLau32 = AP_HAL::millis();
#endif
                count = 0;
                cnt = 0;
            }
            
        } else {     // post launch

            // we have detected the eject impulse
            tof = AP_HAL::micros() / 1000000.0 - tLau;
            // subtract the biases
            for(uint8_t i=0; i<3; i++) {
                gyro[i] = gyro[i] - gyrbia[i];
                accel[i] = accel[i] - accbia[i];
            }
#if USE_SD
            //tof32 = AP_HAL::millis() - tLau32;
            float toff = (float)tof;
                    write( fd, &toff, 4 );  // 1
            float temp = gyro[0]*R2D;
                    write( fd, &temp, 4 );  // 2
            temp = gyro[1]*R2D;
                    write( fd, &temp, 4 );  // 3
            temp = gyro[2]*R2D;
                    write( fd, &temp, 4 );  // 4
            temp = rpy[0]*R2D;
                    write( fd, &temp, 4 );  // 5
            temp = rpy[1]*R2D;
                    write( fd, &temp, 4 );  // 6
            temp = rpy[2]*R2D;
                    write( fd, &temp, 4 );  // 7
            temp = accel[0];
                    write( fd, &temp, 4 );  // 8
            temp = accel[1];
                    write( fd, &temp, 4 );  // 9
            temp = accel[2];
                    write( fd, &temp, 4 );  // 10
            temp = vCmd;
                    write( fd, &temp, 4 );  // 11
#endif            

            // update integrators;
            for(uint8_t i=0; i<3; i++) {
                rpy[i] += dtStep * gyro[i];
                pos[i] += dtStep * vel[i];
                vel[i] += dtStep * accel[i];
            }
        } // if launched
        
        // print accel/gyro result every 500 cycles
        if ( (count % 500 == 0) & !printacs ) {
            printthis = 1;
            uint16_t etu = AP_HAL::micros() - uStart;
            hal.console->printf("tof: %6.3f etu: %u cnt: %5llu ", tof, etu, count ); 
            hal.console->printf("gyro: %6.2f %6.2f %6.2f ", gyro[0]*R2D, gyro[1]*R2D, gyro[2]*R2D );
            hal.console->printf("rpy: %6.2f %6.2f %6.2f ", rpy[0]*R2D, rpy[1]*R2D, rpy[2]*R2D );
            hal.console->printf("lau: %d", launched );
            hal.console->printf("vlv: %1d %1d %1d %1d", 
                vCmd%2, (int)(vCmd/2.)%2, (int)(vCmd/4.)%2, (int)(vCmd/8.)%2 );
        } // print counter
        
        if( tof > 0.020 ) {
        ///////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////
            vCmd = ap.update( gyro, rpy, tof, dtStep, printacs );
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
