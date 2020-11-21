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
#include <fstream>
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InertialSensor/AP_InertialSensor_Invensense.h>
#include <AP_InertialSensor/AP_InertialSensor_LIS331.h>
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

//namespace fs = std::filesystem;

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static AP_InertialSensor ins;

// board specific config************************************************
static AP_BoardConfig BoardConfig;

static acsAP acs;

AP_InertialSensor_LIS331 *lis;

void setup(void);
void loop(void);
void removeBias(void);
void integrate(void);
void logToMem(void);
void xferToSD(void);
void print_file(std::string name);
void runFileViewer(void);
void showFiles(void);

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
// float_t stor[32600]; // 1 sec @1khz record all raw INS data
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
    //hal.console->printf("Initializing ACS\n");
    acs.init();
    //hal.console->printf("ACS initialized\n");

#if USE_SD
//  sdcard_init();
//  fd = open( "/APM/ins2.dat", O_WRONLY|O_CREAT );
//  ft = open( "/APM/test.txt", O_WRONLY|O_CREAT );
//  hal.scheduler->delay(1);
#endif

    //Uncommenting lis.begin freezes operation at "Charging 1"
    //If lis.begin, lis.read, and lis.write are reworked for ChibiOS everything else should work with minimum updating.
    /*
    hal.console->printf("Starting 400G Accelerometer\n");
    lis.setI2CAddr(0x19);
    lis.begin(AP_InertialSensor_LIS331::USE_I2C);
    hal.console->printf("Started 400G Accelerometer\n");
    */

    //Take two
    //hal.console->printf("Starting 400G Accelerometer\n");
    //lis->setI2CAddr(0x19);
    //lis->start();
    //lis->fakeStart();

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

    if (memcnt + 6 > sizeof(stor)/sizeof(stor[0])) {
        mode = POST;
    }
    //hal.console->printf("logToMem tof%f memcnt%d\n", tof, memcnt);
}

void xferToSD(void){
    // dump stored data to SD Card
    sdcard_init();
    float temp;
    //int fd = -1;
    std::string path;
    int num = 1;
    struct stat buf;
    bool exists;
    do {
        path = "/APM/imudata_";
        path += std::to_string(num);
        path += ".csv";
        num++;
        exists = stat(path.c_str(), &buf) != -1;
    } while(exists);

    //fd = open( "/APM/v001.dat", O_WRONLY|O_CREAT );//09302019_0936
    //fd = open( "/APM/v002.dat", O_WRONLY|O_CREAT );//09302019_1026
    //fd = open( "/APM/v004.dat", O_WRONLY|O_CREAT );//09302019_1026
    //120psi, ext power, small maneuvers, watch LEDs
    //fd = open( "/APM/v005.dat", O_WRONLY|O_CREAT );//10022019_1449
    //fd = open( "/APM/v006ft.dat", O_WRONLY|O_CREAT );//10032019_1100
    //fd = open( "/APM/v008bt.dat", O_WRONLY|O_CREAT );//10042019_1130
    //fd = open( "/APM/v008ft.dat", O_WRONLY|O_CREAT );//10032019_1230 no one
    //fd = open( "/APM/v009bt.dat", O_WRONLY|O_CREAT );//10242019_1220 bench test
    //fd = open( path, O_WRONLY|O_CREAT|O_logRUNC );//10242019_1240
    //fd = open( "/APM/v011bt.dat", O_WRONLY|O_CREAT );//10252019_0850
    
    /*
    comments in:
    /opt/gcc-arm-none-eabi-6-2017-q2-update/arm-none-eabi/include/wchar.h
        lines 73-79
    /opt/gcc-arm-none-eabi-6-2017-q2-update/arm-none-eabi/include/c++/6.3.1/bits/basic_string.h
        lines 5466-5530
    */

    std::ofstream fd;
    fd.open(path);

    fd << "time,gyrx,gyry,gyrz,rol,pit,yaw,accx,accy,accz,vCmds" << "\n";

    uint8_t nThisLine = 0;
    for( int k=0; k<memcnt; k++ ) { // memcnt
        temp = stor[k];

        fd << std::to_string(temp) << ",";
        hal.console->printf("%8.3f ", temp );
        if( nThisLine++ >= 5 )
        {
            fd << "\n";
            hal.console->printf("\n");
            nThisLine = 0;
            hal.scheduler->delay(4);
        }
    }

    /*
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
    close( fd );*/
    hal.console->printf("Closing log file\n");
    fd.close();
    /*
    while( true ) { // do forever
        continue;
    }*/
}

void print_file(std::string name) {

    std::string path = "/APM/" + name;
    std::ifstream fd;
    fd.open(path);
                
    std::string line;
    while(getline(fd, line)){  //read data from file object and put it into string.
        line += "\n";
        hal.console->printf(line.c_str());   //print the data of the string
        hal.scheduler->delay(4);
    }

    fd.close();
}

void showFiles() {
    std::string files[50];

    hal.console->printf("\nAvailable Files:\n----------------\n");
    
    DIR *dir;
    struct dirent *ent;
    int num_files = 0;
    if ((dir = opendir ("/APM")) != NULL) {
        // print all .csv files within directory
        while ((ent = readdir (dir)) != NULL) {
            std::string name = ent->d_name;
            int len = name.length();

            if (len >= 4 && name.substr(name.length()-4) == ".csv") {
                files[num_files] = name;
                num_files++;
                hal.console->printf ("%u. %s\n\n", num_files, ent->d_name);
            }
        }
        closedir (dir);
    }

    hal.console->printf("Select number of file to print\n>");
}

void runFileViewer() {

    int user_input;

    // flush any user input
    while (hal.console->available()) {
        hal.console->read();
    }

    std::string files[50];

    hal.console->printf("\nAvailable Files:\n----------------\n");
    
    DIR *dir;
    struct dirent *ent;
    int num_files = 0;
    if ((dir = opendir ("/APM")) != NULL) {
        // print all .csv files within directory
        while ((ent = readdir (dir)) != NULL) {
            std::string name = ent->d_name;
            int len = name.length();

            if (len >= 4 && name.substr(name.length()-4) == ".csv") {
                files[num_files] = name;
                num_files++;
                hal.console->printf ("%u. %s\n\n", num_files, ent->d_name);
            }
        }
        closedir (dir);
    }

    hal.console->printf("Select number of file to print\n>");
                        
    // wait for user input
    while (!hal.console->available()) {
        hal.scheduler->delay(20);
    }

    // read in user input
    if (hal.console->available()) {
        uint8_t buf[2];
        user_input = hal.console->read(buf, (uint16_t) 2); // offset for ascii value
        hal.console->printf("%u", user_input);
        //user_input = (int) user_input - 48;

        if (0 < user_input && user_input <= num_files){
            print_file(files[user_input-1]);
        }
        else{
            hal.console->printf("Not a valid Input\n");
        }
    }

    // wait for user input
    while (!hal.console->available()) {
        hal.scheduler->delay(20);
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
//          accelf[i] = accel[i];
            gyrof[i] = gyro[i];
        #endif
            // Integrate states
            //rpy[i] += dtStep * gyrof[i];
            //pos[i] += dtStep * vel[i];
            //vel[i] += dtStep * accel[i];
        }

        /*
        //400G Accelerometer Readings
        int16_t x, y, z;
        lis->read_data_transaction_a(x,y,z);
        hal.console->printf("400G: x=%d, y=%d, z=%d\n", x, y, z);
        */

        // sequence control and one-time algorithms
        switch( mode )
        {
            case CAL  :
                if(calCnts >= 500) {
                    mode = PRE;
                    hal.console->printf(" mode=>Pre\n");
                    hal.console->printf("Press any key to show files\n>");
                }
                break;
            case PRE :
//              if( accel[0] <= gThreshold || !FLTTEST) { //|| true
                if( accel[0] <= gThreshold ) { //|| time >= 2.8) { //|| true
                    tlau = time;
                    hal.console->printf(" launch detected at %f\n", tlau);
                    launched = true;
                    mode = FLT;
                }

                if (hal.console->available()) {
                    mode = POST;
                    return;
                }
                break;

            case FLT :
                if( tof >= tofmax ) { // || time > tmax
                    mode = POST;
                    hal.console->printf(" post flight %f6.3\n", tof);
                    xferToSD();
                    return;
                }
                break;

            case POST :
                stopex = true;
                acs.shutdown();
                hal.scheduler->delay(1000);
        } // switch( mode )

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
                vCmd = acs.update( ypCmd, gyrof, rpy, time, tof );
                /////////////////////////////////////////////////////
                /////////////////////////////////////////////////////
                // logToMem();
                break;

            case FLT:
                tof = time - tlau;
                removeBias();
                integrate();
                /////////////////////////////////////////////////////
                /////////////////////////////////////////////////////
                vCmd = acs.update( ypCmd, gyrof, rpy, time, tof );
                /////////////////////////////////////////////////////
                /////////////////////////////////////////////////////
                logToMem();

                if( outcnt++ % 50 != 0 ) {
                    continue;
                }
                hal.console->printf("t%6.3f  tof%6.3f  q%7.3f  r%7.3f  p%7.3f  y%7.3f  gb1 %7.3f  gb2%7.3f cnt%6d  acc%6.2f\n",
                    time, tof, gyrof[1], gyrof[2], rpy[1]*R2D, rpy[2]*R2D, gyrBia[1], gyrBia[2], outcnt, accel[0] );

                break;

            case POST:

                while(true) {
                    runFileViewer();
                }
                break;
        }  // switch case

    } // while( !stopex )

}// end of "loop" method

AP_HAL_MAIN();
