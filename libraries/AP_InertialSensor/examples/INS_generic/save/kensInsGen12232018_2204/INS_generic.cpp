//
// Simple test for the AP_InertialSensor driver.
//
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include "acsAP.h"
//#include <drivers/device/ringbuffer.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
//const AP_HAL::HAL &hal = AP_HAL::get_HAL();
AP_InertialSensor ins;

//AP_Scheduler scheduler{nullptr};

static AP_BoardConfig BoardConfig;

void setup(void);
void loop(void);
const uint16_t freq = 500;
const double dtStep = 0.002;
double tStart;
double tFinish;
acsAP ap; // construct the autopilot

class INS_generic {
public:
    void setup();
    void loop();
    
private:
    //uint64_t trun = 0;
    //uint64_t tlau = 0;
    
    //int *p = new int[10]
    
    //double* dtLog  = NULL;         // Pointer initialized with null
};

void setup(void)
{
    // setup any board specific drivers
    BoardConfig.init();
    ins.init( freq );
}

void loop(void)
{
    double tof = 0.0;
    bool launched = false;
    uint64_t count = 0;
    uint64_t tLau = 0;
    double accsum[3]= {0.0, 0.0, 0.0};
    double gyrsum[3]= {0.0, 0.0, 0.0};
 // double accavg[3];
 // double gyravg[3];
    double accbia[3];
    double gyrbia[3];
 // double accmsr[3];
 // double gyrmsr[3];
    double rpy[3] = {0.0, 0.0, 0.0}; // roll, pitch, yaw
    double vel[3] = {0.0, 0.0, 0.0};
    double pos[3] = {0.0, 0.0, 0.0};
    double *dtLog  = new double[999];       // Request memory for the variable

    /* The most commonly used hal functions are:
    hal.console->printf() to print strings
    AP_HAL::millis() and AP_HAL::micros() to get the time since boot
    hal.scheduler->delay() and hal.scheduler->delay_microseconds() to sleep for a short time
    hal.gpio->pinMode(), hal.gpio->read() and hal.gpio->write() for accessing GPIO pins
    I2C access via hal.i2c
    SPI access via hal.spi */

    // flush any user input
    while ( hal.console->available() ) {
        hal.console->read();
    }

    // clear out any existing samples from ins
    ins.update();

    // loop as long as user does not press a key
    while (!hal.console->available() ) {
        tStart = AP_HAL::millis() / 1000.0;
        // wait until we have a sample
        ins.wait_for_sample();

        // read samples from ins
        ins.update();

        uint8_t ii = 1; // which imu, 0 or 1
        Vector3f acc = ins.get_accel(ii);
        Vector3f gyr = ins.get_gyro(ii);
        double accel[3] = {acc.x, acc.y, acc.z};
        double gyro[3] = {gyr.x, gyr.y, gyr.z};

        if( !launched ) {
            tof = 0.0;
            // calibrate for a number of samples or a 2 gee pulse
            if( count++ < 999 || accel[2] < -14.8L ) {
                launched = true;
                tLau = AP_HAL::millis() / 1000.0;
            }
            for(uint8_t i=0; i<3; i++) {
                accsum[i] += accel[i];
                gyrsum[i] += gyro[i];
                accbia[i] = accsum[i] / count;
                gyrbia[i] = gyrsum[i] / count;
            }

            count = 0;
        } else {
            count++;
            // we have detected the eject impulse
            tof = AP_HAL::millis() / 1000.0 - tLau;
          
            // subtract the biases
            for(uint8_t i=0; i<3; i++) {
                gyro[i] -= gyrbia[i];
                accel[i] -= accbia[i];
            }
            
            ////////////////////////////////////////////
            ////////////////////////////////////////////
            ap.update( gyro, rpy, tof, dtStep );
            ////////////////////////////////////////////
            ////////////////////////////////////////////
 
            // update integrators for velocity and position
            // update integrators;
            for(uint8_t i=0; i<3; i++) {
                rpy[i] += dtStep * (gyro[i]-gyrbia[i]);
                pos[i] += dtStep * vel[i];
                vel[i] += dtStep * (accel[i]-accbia[i]);
            }

            // print each accel/gyro result every 1 second
            if (count++ % freq != 0) {
                continue;
            }
            if (false) {
                hal.console->printf("%6.3f %6.3f %7.2f%7.2f%7.2f  %7.2f%7.2f%7.2f  %7.2f%7.2f%7.2f  %7.2f%7.2f%7.2f pwms:%3i\n", 
                     tof, dtStep, accel[0], accel[1], accel[2], vel[0], vel[1], vel[2], pos[0], pos[1], pos[2],
                     rpy[0], rpy[1], rpy[2], BoardConfig.get_pwm_count() );
            } else {
                hal.console->printf("pwms: %3d\n", BoardConfig.get_pwm_count() );
            }
        }
    }
    double tThis = AP_HAL::millis() / 1000.0;
    dtLog[count] = tStart - tThis;
    // wait until scheduled time of completion
    while( tThis < tStart + dtStep )
        tThis = AP_HAL::millis() / 1000.0;
    }

AP_HAL_MAIN();
