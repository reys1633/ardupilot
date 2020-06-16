#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
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

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static AP_InertialSensor ins;

// board specific config
static AP_BoardConfig BoardConfig;

void setup(void);
void loop(void);

void Log_Write_IMUA(void);

double tStart = -1.0; // time when execution begins
double accel[3];
double gyro[3];

void setup(void) {
    BoardConfig.init();  // setup any board specific drivers
    ins.init( freq );
    ap.init();
#if USE_SD
    sdcard_init();
    fd = open( "/APM/ins.dat", O_WRONLY|O_CREAT );
    hal.scheduler->delay(1);
#endif
    for(int i=0; i<6; i++) {  // make sure all valves are off
        hal.gpio->pinMode( 50+i, HAL_GPIO_OUTPUT );
        hal.gpio->write( 50+i, 0 );
    }
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
        if( tStart < 0.0 ) {
            tStart = AP_HAL::micros64()/1000000.0L;
        }
        double time = AP_HAL::micros64()/1000000.0 - tStart;
//      hal.console->printf( "time:%8.3f\n", time );      
        if( time > tmax ) {           // tof > tofmax ||
            ap.shutdown();
            stopex = true;
#if USE_SD
            hal.console->printf("Closing log\n"); 
            close( fd );
            
            // write out the data
            fd = open( "/APM/ins.dat", O_RDONLY );
            float anum;
            hal.console->printf("    tof     gfx     gfy     gfz     rol ");
            hal.console->printf("    pit     yaw     afx     afy     afz    vcmd   ypcmd\n");

            for( int k=0; k< 50; k++ ) {
				while( fd != EOF ) {
					for(int j=0; j<13; j++ ) { // was 12
						read( fd, &anum, 4 );
						hal.console->printf("%7.3f ", anum );
					}
					hal.console->printf("\n");
				}
			}
#endif     
            hal.console->printf("Finished -- Please unplug Pix\n");
            hal.scheduler->delay(2000);
            hal.console->printf("Finished -- Please unplug Pix\n");
        } // tof > tmax
        
        // wait until we have a sample
        ins.wait_for_sample();

        float temp = (float)time; //tof;
        write( fd, &temp, 4 );  // 1

		// read samples from ins
        ins.update();
        for( int iins=0; iins<2; i++) {
			acc = ins.get_accel( iins ); 
			gyr = ins.get_gyro( iins );
			accel[2] = acc.x; // pix x ==> msl z
			accel[1] = acc.y; // pix y ==> msl y (no change
			accel[0] = acc.z; // pix z ==> msl x
			gyro[2] = gyr.x; // pix roll ==> msl yaw
			gyro[1] = gyr.y; // pix pitch ==> msl pitch (no change)
			gyro[0] = gyr.z; // pix yaw ==> msl roll:  0.000 pc
            temp = gyro[0]*R2D;
                    write( fd, &temp, 4 );  // 2
            temp = gyro[1]*R2D;
                    write( fd, &temp, 4 );  // 3
            temp = gyro[2]*R2D;
                    write( fd, &temp, 4 );  // 4
            temp = accel[0];
                    write( fd, &temp, 4 );  // 8
            temp = accel[1];
                    write( fd, &temp, 4 );  // 9
            temp = accel[2];
                    write( fd, &temp, 4 );  // 10
		}
		
		// ignite SRM
		hal.gpio->pinMode( 51, HAL_GPIO_OUTPUT );
        if( (tof > tSrm) && (tof <= tSrm+durSrm) ) {
            hal.gpio->write( 51, armed ); // red output
          //firesrm = 16;                 // for logging
        } else {
            hal.gpio->write( 51, 0 );
        }
    
        // deploy parachute by setting pin(?) high
        hal.gpio->pinMode( 50, HAL_GPIO_OUTPUT );
        if( (tof > tPara) && (tof <= tPara+durPara) ) {
            hal.gpio->write( 50, armed ); // white output
          //firepara = 32;                // for logging
        } else {
            hal.gpio->write( 50, 0 );             
        }
    } // end of while true
}  // end of "loop" method

AP_HAL_MAIN();


