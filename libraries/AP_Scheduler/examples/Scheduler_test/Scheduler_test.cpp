//
// Simple test for the AP_Scheduler interface
//

#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <DataFlash/DataFlash.h>
#include "acsAP.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_Int32 log_bitmask;
DataFlash_Class DataFlash{log_bitmask};

acsAP acs;

class SchedTest {
public:
    void setup();
    void loop();

private:
    AP_InertialSensor ins;
    AP_Scheduler scheduler{nullptr};

    uint32_t ins_counter = 0;
    uint32_t avg_counter = 0; // for averaging multiple INS measurements
    uint32_t acs_counter = 0;
    static const AP_Scheduler::Task scheduler_tasks[];

    // scheduled methods
    void ins_update(void);
    void acs_update( void );
    void one_hz_print(void);
    void five_second_call(void);
    
    double accsum[3];
    double gyrsum[3];
    double accavg[3];
    double gyravg[3];
    double accbia[3];
    double gyrbia[3];
    double accmsr[3];
    double gyrmsr[3];
    double rpy[3] = {0.0, 0.0, 0.0}; // roll, pitch, yaw
    double vel[3] = {0.0, 0.0, 0.0};
    double pos[3] = {0.0, 0.0, 0.0};
    
    uint64_t count = 0;
 //   uint64_t countFly = 0;
    bool fly = false;
};

static AP_BoardConfig board_config;
static SchedTest schedtest;

#define SCHED_TASK(func, _interval_ticks, _max_time_micros) SCHED_TASK_CLASS(SchedTest, &schedtest, func, _interval_ticks, _max_time_micros)
/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in 20ms units) and the maximum time
  they are expected to take (in microseconds)
 */
const AP_Scheduler::Task SchedTest::scheduler_tasks[] = {
    SCHED_TASK(ins_update,           1000,    500), //   50,   1000),
    SCHED_TASK(acs_update,            500,   2000),
    SCHED_TASK(one_hz_print,            1,    100),
    SCHED_TASK(five_second_call,      0.2,    100),
};


void SchedTest::setup(void)
{
    board_config.init();
  //int pwms = board_config.get_pwm_count(); 
  //hal.console->printf("pwms: %3d\n", board_config.get_pwm_count() );
        
    ins.init( scheduler.get_loop_rate_hz() );

    // initialise the scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), (uint32_t)-1);
}


void SchedTest::loop(void)
{
    // run all tasks
    scheduler.loop();
}


/*   update inertial sensor, reading data   */
void SchedTest::ins_update(void)
{   
    // hal.console->printf("ins update: t=%lu\n", (unsigned long)AP_HAL::millis());
    // hal.console->printf("pwms: %3d\n", board_config.get_pwm_count() );
    ins_counter++;
    avg_counter++;
    ins.update();       
    Vector3f acc = ins.get_accel( 0 ); // specify which IMU, 0 or 1
    Vector3f gyr = ins.get_gyro( 0 );
    accmsr[0] = acc.x;
    accmsr[1] = acc.y;
    accmsr[2] = acc.z;
    gyrmsr[0] = gyr.x;
    gyrmsr[1] = gyr.y;
    gyrmsr[2] = gyr.z;
    
    // if ins is running @2000 and ap.update @ 500
    // then we average 4 measurements for each call to acsAP.update
    // this may introduce slight latency but more smoothness?
    for(uint8_t i=0; i<3; i++) {
        accsum[i] += accmsr[i];
        accavg[i] = accsum[i] / avg_counter;
        gyrsum[i] += gyrmsr[i];
        gyravg[i] = gyrsum[i] / avg_counter;
    }
    
    if( !fly ) {                     // pre launch - 2 ways do detect launch
       if( (ins_counter > 99) || accavg[2] < (double)-14.8 ) {
            fly = true;
            ins_counter = 0;
            for(uint8_t i=0; i<3; i++) {
                accbia[i] = accavg[i];
                gyrbia[i] = gyravg[i];
                accsum[i] = 0.0;
                gyrsum[i] = 0.0;
            }
        }
    } else {                          // post launch
        // do integrations
        // integrators inherently average, 
        // so we don't use gyravg, accavg
        for(uint8_t i=0; i<3; i++) {
            // TODO: setup dt's better
            rpy[i] += 0.0005L * (gyrmsr[i]-gyrbia[i]);
            pos[i] += 0.0005L * vel[i];
            vel[i] += 0.0005L * (accmsr[i]-accbia[i]);
        }
    } 
    //hal.console->printf("%7.2f%7.2f%7.2f", accavg[0], accavg[1], accavg[2] );
}

void SchedTest::acs_update(void)
{
    double tNow = (double)AP_HAL::millis() / (double)1000.0;
    //hal.console->printf("tNow: %7.2f calling acs_update \n", tNow );
    //hal.console->printf("%7.3f %6.3f %6.3f %6.3f \n", tNow, rpy[0], rpy[1], rpy[2] );

    acs.update( tNow, gyravg, rpy );
    acs_counter++;

    // reset these for next 4 cycles
    avg_counter = 0;
    for(uint8_t i=0; i<3; i++) {
        accsum[i] = 0.0;
        gyrsum[i] = 0.0;
    }
}

/*  print something once a second  */
void SchedTest::one_hz_print(void)
{
    uint32_t looprate = scheduler.get_loop_rate_hz();
    hal.console->printf("looprate: %3d\n", looprate );

    double tNow = (double)AP_HAL::millis() / (double)1000.0;
    hal.console->printf("one_hz: tNow=%6.3f ins: %u acs: %u\n", 
        tNow, ins_counter, acs_counter ); //(unsigned long)AP_HAL::millis());
}

/*   print something every 5 seconds  */
void SchedTest::five_second_call(void)
{
    double tNow = (double)AP_HAL::millis() / (double)1000.0;
    hal.console->printf("five_sec task: tNow=%6.3f ins_ctr=%u\n", tNow, (unsigned)ins_counter);
    //hal.console->printf("pwms: %3d\n", board_config.get_pwm_count() );
}

/*
  compatibility with old pde style build
 */
void setup(void);
void loop(void);

void setup(void)
{
    schedtest.setup();
}
void loop(void)
{
    schedtest.loop();
}
AP_HAL_MAIN();
