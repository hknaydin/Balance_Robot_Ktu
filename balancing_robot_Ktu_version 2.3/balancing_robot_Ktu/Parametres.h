#ifndef _Parametres_h_
#define _Parametres_h_
#define ENABLELEFT  3 
#define ENABLERIGHT  11 
#define MOTORR0  5
#define MOTORR1  6
#define MOTORL0  9
#define MOTORL1  10
#define LEDYELLOW 7 // Calibrations for mpu
#define LEDRIGHT 8
#define LEDLEFT 12
#define LED_PIN 13

#define Preamp 35
#define DeadZone 15
#define ModeAngle 50

int speedR=0;
int speedL=0;
int count=1; //  for mpu calibration
float calibrationValues[3];

float Kp=4.5;
float Kd=0.5;
float Ki=5.5;
float error=0;
float Aangle=0;
float Lastangle=0;
float LLastangle=0;
float target=0; //target
float Pid =0;
int hizlanR=0;
int hizlanL=0;
int AngleUp=1;
int SngleDown=1;
int AngleDown=1;
#endif
