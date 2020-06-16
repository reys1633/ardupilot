//
// Simple test for the AP_InertialSensor driver.
//
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include "acsAP.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/AP_Math.h>
#include <cmath>


//#include <drivers/drv_gpio.h>

//#include <drivers/drv_hrt.h>
//#include </home/k/src/Firmware_PX4/src/drivers/drv_hrt.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();
AP_InertialSensor ins;
AP_Scheduler scheduler{nullptr};
// board specific config
static AP_BoardConfig BoardConfig;

void setup(void);
void loop(void);
const uint16_t freq = 2000;
const double dt   = 0.0005;
acsAP ap; // construct the autopilot

void setup(void)
{
    // setup any board specific drivers
    BoardConfig.init();
    ins.init( freq );
}

void loop(void)
{           
    hal.console->printf("\n");

    double accsum[3] = {0.0, 0.0, 0.0};
    double gyrsum[3] = {0.0, 0.0, 0.0};
    double gyrbia[3];
    double accbia[3];
    uint64_t countPre = 0;
    uint64_t countFly = 0;
    uint64_t trun = 0;
    uint64_t tlau = 0;
    // some integrators
    static double rpy[3] = {0.0, 0.0, 0.0}; // roll, pitch, yaw
    static double vel[3] = {0.0, 0.0, 0.0};
    static double pos[3] = {0.0, 0.0, 0.0};
    double tnow  = 0.0;   // double to int
    //static double tlast = 0.0;
    bool parachuteDep = false;
    bool motorIgnited = false;

    /* The most commonly used hal functions are:
    hal.console->printf() to print strings
    AP_HAL::millis() and AP_HAL::micros() to get the time since boot
    hal.scheduler->delay() and hal.scheduler->delay_microseconds() to sleep for a short time
    hal.gpio->pinMode(), hal.gpio->read() and hal.gpio->write() for accessing GPIO pins
    I2C access via hal.i2c
    SPI access via hal.spi */

    // flush any user input
    while (hal.console->available()) {
        hal.console->read();
    }

    // clear out any existing samples from ins
    ins.update();

    // loop as long as user does not press a key
    while (!hal.console->available() ) {
        if(tnow > 10.0) {
            while(true) {
               continue;
            }
        }   

        // wait until we have a sample
        ins.wait_for_sample();

        if( trun <= 1.0 ) {
          trun = 1.0; //hrt_absolute_time(); // when did INS start running
        }

        // read samples from ins
        ins.update();

        uint8_t ii = 1; // which imu, 0 or 1
        Vector3f acc = ins.get_accel(ii);
        Vector3f gyr = ins.get_gyro(ii);
        double accel[3] = {acc.x, acc.y, acc.z};
        double gyro[3] = {gyr.x, gyr.y, gyr.z};

        

        if( countPre++ < 999 || 2 < -19.6 ) {  // calibrate for a number of samples or a 2 gee pulse

          if ( false ) {
            hal.console->printf("%7.2f%7.2f%7.2f", (double)accel[0], (double)accel[1], (double)accel[2]);
            hal.console->printf("%7.2f%7.2f%7.2f", (double)gyro[0], (double)gyro[1], (double)gyro[2] );
            hal.console->printf("\n");
          } else {
            for(uint8_t i=0; i<3; i++) {
                accsum[i] += accel[i];
                gyrsum[i] += gyro[i];
                accbia[i] = accsum[i] / countPre;
                gyrbia[i] = gyrsum[i] / countPre;
            }
//            hal.console->printf("%7lu", countPre );
            hal.console->printf("%7.2f%7.2f%7.2f", (double)accbia[0], (double)accbia[1], (double)accbia[2] );
            hal.console->printf("%7.2f%7.2f%7.2f", (double)gyrbia[0], (double)gyrbia[1], (double)gyrbia[2] );
            hal.console->printf("\n");
          }
        } else {

          // we have detected the eject impulse

          if( tlau == 0 ) {
            tlau = 0.0; // hrt_absolute_time(); // set tlau
          } 
          
          for(uint8_t i=0; i<3; i++) {
              gyro[i] -= gyrbia[i];
              accel[i] -= accbia[i];
          }

          /////////////////////////////////////////////////////////////////
          if(countFly % 4 == 0) {
             ap.update( tnow, gyro, rpy);
          }
          /////////////////////////////////////////////////////////////////
 
          // motor ignition
          if(tnow > 0.050 && !motorIgnited ) {  // make sure that tnow starts at launch
               motorIgnited = true;
               //ap.ignMotor();
          }

          // parachute deployment
          if(tnow > 1.5 && !parachuteDep ) {
               parachuteDep = true;
               //ap.depParachute();
          }
          
          // update integrators for velocity and position
          tnow = (double)countFly / freq; //hrt_absolute_time();
          // update integrators;
          //dt = tnow - tlast;
          for(uint8_t i=0; i<3; i++) {
              rpy[i] += dt * (gyro[i]-gyrbia[i]);
              pos[i] += dt * vel[i];
              vel[i] += dt * (accel[i]-accbia[i]);
          }

          //tlast = tnow;

          // print each accel/gyro result every 1 second
          if (countFly++ % freq != 0) {
            continue;
          }
          if (false) {
              hal.console->printf("%6.3f %6.3f %7.2f%7.2f%7.2f  %7.2f%7.2f%7.2f  %7.2f%7.2f%7.2f  %7.2f%7.2f%7.2f\n", 
                     tnow, dt, accel[0], accel[1], accel[2], vel[0], vel[1], vel[2], pos[0], pos[1], pos[2], rpy[0], rpy[1], rpy[2] );
          } else {
            
          }
        }
    }
}

AP_HAL_MAIN();
