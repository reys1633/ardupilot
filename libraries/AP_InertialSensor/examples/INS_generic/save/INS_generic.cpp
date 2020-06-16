#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InertialSensor/AP_InertialSensor_Invensense.h>
//#include <AP_InertialSensor/AP_InertialSensor_LIS331.h>
#include <stdlib.h>
#include <string.h>
#include "missileRRT.h"
#include "acsAP.h"
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

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static AP_InertialSensor ins;

// board specific config************************************************
static AP_BoardConfig BoardConfig;

static acsAP acs;

//static AP_InertialSensor_LIS331 lis;

void setup(void);
void loop(void);

void Log_Write_IMUA(void);

bool stopex = false;
double tStart = -1.0; // time when execution begins
double accel[3];
double gyro[3];
Vector3f acc;
Vector3f gyr;
//uint fd;
//uint ft;
uint32_t count = 0;
uint32_t outcnt = 0;
int firesrm  = 0; // 0 or 16
int firepara = 0; // 0 or 32
Vector3f extern raw_accel;
Vector3f extern raw_accel_400g;


//uint nn = tofmax * freq * 13; // 1500
//float_t stor[20000]; // 19500
//uint maxrec = 13000+100;    // 1000*13
float_t stor[28200]; // 1 sec @1khz record all raw INS data
//float_t stor[32600]; // 1 sec @1khz record all raw INS data
//float_t stor[29542]; // 1 sec @1khz record all raw INS data
//double_t stor[28200];

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
    hal.scheduler->delay( chargdelay );
    hal.console->printf("tmax = %f\n", tmax);
    acs.init();

#if USE_SD
//  sdcard_init();
//  fd = open( "/APM/ins2.dat", O_WRONLY|O_CREAT );
//  ft = open( "/APM/test.txt", O_WRONLY|O_CREAT );
//  hal.scheduler->delay(1);
#endif
    
    acs.shutdown();
    /*for(int i=0; i<6; i++) {  // make sure all valves are off
        hal.gpio->pinMode( 50+i, HAL_GPIO_OUTPUT );
        hal.gpio->write( 50+i, 0 );
    }*/
} // of setup


void loop(void)
{
    // flush any user input
    while ( hal.console->available() ) {
        hal.console->read();
    }
    ins.update();  // clear out any existing samples from ins
    count = 0;

    while( !stopex ) {      // loop forever
        count++;
        if( tStart < 0.0 ) {
            tStart = AP_HAL::micros64()/1000000.0L;
            hal.console->printf(" starting %f\n", tStart);
        }
        
        double time = AP_HAL::micros64()/1000000.0 - tStart;
//      double tof = time;
//      hal.console->printf(" %6.3f\n", tof );
//      hal.console->printf( "time:%8.3f\n", time );   
        if( time > tmax ) {
            hal.console->printf("stopping\n");
        }
        
        if( time > tmax ) {           // tof > tofmax ||
//          ap.shutdown();
            hal.console->printf("stopping\n");
            hal.scheduler->delay(100);
            stopex = true;
//            hal.console->printf("    tof     gfx     gfy     gfz     rol ");
//            hal.console->printf("    pit     yaw     afx     afy     afz    vcmd   ypcmd\n");
//            hal.console->printf("    tof     gx1     gy1     gz1     ax1     ay1     az1 ");
//            hal.console->printf("    gx2     gy2     gz2     ax2     ay2     az2\n");

            // dump stored data to SD Card
            sdcard_init();
            
/*            
            int findex = 0;
            int fd = -1;
            char* fn;
            
            while( fd < 0 )
                fn = "/APM/ins" + findex + ".dat";
                fd = open( fn, O_WRONLY|O_CREAT )
                findex++;
            end

            int num = 0;
            String save = at.getText().toString() + ".jpg";
            File file = new File(myDir, save);
            while(file.exists()) {
                save = at.getText().toString() + (num++) +".jpg";
                file = new File(myDir, save); 
            }
*/
            float temp;
            int fd = -1;
           
            /*
            int fileCounter = 1;
            const char* fileCount;
            char* fileName = "/APM/ins3";
            char* fileType = ".dat";
            char* insFileName;
            while (1) {
                //fileCount = const char*(fileCounter);
                //insFileName = strcat(fileName, fileCount, fileType);
                insFileName = "/APM/ins3" + fileCounter + ".dat";
                if (open(insfileName, O_RDONLY) == NULL) {
                    break;
                }
                fileCounter++;
            }
        
            hal.console->printf("Writing to %s\n", insFileName);
            */
            fd = open( "/APM/ins3.dat", O_WRONLY|O_CREAT );
            for( int k=0; k<outcnt; k++ ) { // outcnt
//              hal.console->printf("%7d %12.3f \n", k, stor[k] );
                temp = stor[k];
                write( fd, &temp, 4 );
             } 

            hal.console->printf("Closing log\n");
            close( fd );
            hal.scheduler->delay(1000);
            
            // read SD and write data to console via mavlink
            hal.console->printf("outcnt= %7ld \n", outcnt );
            fd = open( "/APM/ins3.dat", O_RDONLY );
            float anum;

//          for( int k=0; k< 100; k++ ) {
//			while( fd != EOF ) {
            int thiscnt = 0;
            while(thiscnt<outcnt) {
                for(int j=0; j<14; j++ ) { // was 12
                    read( fd, &anum, 4 );
                    //hal.console->printf("%7.3f ", anum );
                }
                thiscnt = thiscnt+14;
                hal.scheduler->delay( 5 );
                //hal.console->printf("\n");
			}
    
            hal.console->printf("Finished -- Please unplug Pix\n");
            hal.scheduler->delay(2000);
            hal.console->printf("Finished -- Please unplug Pix\n");
        } // tof > tmax
        
        if (time > 0.25) {
            hal.scheduler->delay(20);
            //After 0.25 seconds, record at 50Hz to save space in stor[]
        }
        
        // wait until we have a sample
        ins.wait_for_sample();
        
//      float temp = (float)time; //tof;

//        if( outcnt < 20000 ) {
            stor[outcnt] = (float)time;
            outcnt++;
//        }
        //hal.console->printf("Or Here?");       
		// read samples from ins
        ins.update();
        //hal.console->printf("%f,%f,%f\n", raw_accel[0], raw_accel[1], raw_accel[2]);
        //lis.readAxes(raw_accel_400g[0], raw_accel_400g[1], raw_accel_400g[2]);
        //hal.console->printf("%f,%f,%f\n", raw_accel_400g[0], raw_accel_400g[1], raw_accel_400g[2]);
        int idx;
        for( int iins=0; iins<2; iins++) {
			acc = ins.get_accel( iins ); 
            //acc = mpu._read_fifo();
            //hal.console->printf("%f, %f\n", acc[1], acc[2]);
		    gyr = ins.get_gyro( iins );

            for( int ichan=0; ichan<3; ichan++) {
                idx = outcnt + iins*6 + ichan;
                stor[idx] = gyr[ichan]; 
            }
            for( int ichan=0; ichan<3; ichan++) {
                idx = outcnt + iins*6 + ichan + 3;
                //stor[idx] = acc[ichan]; 
                stor[idx] = raw_accel[ichan];
            }
        } 
        outcnt = outcnt + 12;
        idx++;
        stor[idx] = float( firesrm + firepara );
		outcnt++;
        /*
		// ignite SRM
		hal.gpio->pinMode( 51, HAL_GPIO_OUTPUT );
        if( (tof > tSrm) && (tof <= tSrm+durSrm) ) {
            hal.gpio->write( 51, armed ); // red output
            firesrm = 16;                 // for logging
        } else {
            hal.gpio->write( 51, 0 );
            firesrm = 0;
        }
        // deploy parachute by setting pin(?) high`1
        hal.gpio->pinMode( 50, HAL_GPIO_OUTPUT );
        if( (tof > tPara) && (tof <= tPara+durPara) ) {
            hal.gpio->write( 50, armed ); // white output
            firepara = 32;                // for logging
        } else {
            hal.gpio->write( 50, 0 ); 
            firepara = 0;            
        } */
        //hal.console->printf("Time: %f Accel: %f Stor: %f\n", time, accel[1], &stor);
        hal.console->printf("%f, %lu\n", time, count);

    } // end of while true
    //hal.console->printf("Finished");
}  // end of "loop" method

AP_HAL_MAIN();


