#pragma once

int    const freq      = 500;
double const dtStep    = 0.002;
//int    const nConsole  = 250;  // counts per console output

double const omega0    = 2.0 * 3.14159 * 3.0;   // 4.5
double const zeta      = 0.8;
double const iyy       = 0.0254; 
double const xAcs      = 0.0076; //  0.3 * IN2M
double const xCg       = 0.2032; // 16/2 * IN2M

/*double const tankF0    = 10.0; //4.65; //18.6;
double const maxACS    = 10.0; //4.65; //10.0;
double const tankSlope = 0.0; //-1.5;  //-6.12; */

// 02 05 2018 
/*double const tankF0    = 5.0; //4.65; //18.6;
double const maxACS    =  2.5; //4.65; //10.0;
double const tankSlope = -1.0; //1.5;  //-6.12; */

double const tankF0    = 5.0; // 5.0; //4.65; //18.6;
double const maxACS    = 5.0; // 2.5; //4.65; //10.0;
double const tankSlope = 0.0; //-1.0; //1.5;  //-6.12;

double const deadZone  = 0.5;

double const durCal    = 3.0;   // cycles for INS Cal

double const cmdLau    = 10.01; //4.1 * 1;   // scripted launch cmd time

double const gThreshold= -9.8*2; //nominally -9.8 sitting on launcher
double const durSrm    = 0.25; // duration of ignition pulse
double const durPara   = 0.25; // duration of parachture squib pulse

// temp for test klh 10/22/2020
int    const toneQty   = 24; // 64; // total acs tone pulses flight test, 0.2 sec

int    const chargdelay= 3000; //150; //15000


#define AUTOLAU true
#if AUTOLAU
double const tSrm      = 1.000; //0.5; ///0.050; // don't fire
#else
double const tSrm      = 0.25; //0.5; ///0.050; // don't fire
#endif

#define FLTTEST false  // !FLTTEST causes automatic launc in seq
#if FLTTEST
  double const tofmax    = 2.5; 
  double const tmax      = 3.0;  // time to stop execution loop
  double const tPara     = 1.5;  // time to eject parachute
  #define OPENLOOP false
#else
  double const tofmax    = 2.5;
  double const tmax      = 3.0; // time to stop execution loop
  double const tPara     = 1.5; // time to eject parachute
  #define OPENLOOP false
#endif

#define origChannels false
#if origChannels
  int const iSRM  = 51;
  int const iPARA = 50;
  int const iVALV = 52;
#else 
  int const iVALV = 50;
  int const iSRM  = 54;
  int const iPARA = 55;
#endif
  
