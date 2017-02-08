#include <SoftwareSerial.h>

/////////////////////////////////////////////////////////
//// Balancing Robot for KTU Computer Engineering ///////
////          Tasarım Projesi         ///////////////////
////   HAKAN AYDIN -243939- 1.OGRETIM  //////////////////
////   GÖKAY YILMAZ -243947- 1.OGRETIM  //////////////////
/////////////////////////////////////////////////////////


#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Parametres.h"
SoftwareSerial mySerial(0, 1); //RX, TX 
/*
// ================================================================
// ---------- Define Variables -------------
// ---------- define motor parameters  ------
// ================================================================
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


// ================================================================
// ---------- Define motors attack, angle includes and deadzone ---
// ================================================================
#define Preamp 35
#define DeadZone 15
#define ModeAngle 50
int speedR=0;
int speedL=0;
int count=1; //  for mpu calibration
float calibrationValues[3];
*/
// ================================================================
//  ------Functions-------------------
// ================================================================
void LedLightingForward();
void LedLightingBackward();
void SetMotorSpeed(int speedR, int speedL);
void motor_forwardLeft();
void motor_forwardRight();
void motor_backLeft();
void motor_backRight();
void calibration();



// ================================================================
// ----------- Bluetooth command -----------
// ================================================================
char command='S';
boolean state;
byte ComStep=0;


/*
// ================================================================
// ----------- Define PID Controller Variables ---------
// ================================================================
float Kp=4.5;//4.5 bu degerler harikadır..
float Kd=0.5;
float Ki=5.5;//5.5 // bu degerler harikadır..
float error=0;
float Aangle=0;
float Lastangle=0;
float LLastangle=0;
float target=0; //target
float Pid =0;*/


// ================================================================
// ---------- Define MPU 6050 Variables ---------
// ================================================================
MPU6050 mpu;
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer // orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorInt16 aa; // [x, y, z] accel sensor measurements
VectorInt16 aaReal; // [x, y, z] gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z] world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z] gravity vector
VectorInt16 aaOffset; // [x, y, z] accel sensor measurements offset
float euler[3]; // [psi, theta, phi] Euler angle container
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
float yprOffset[] = {0, 0, 0}; // [yaw, pitch, roll] offset yaw/pitch/roll container and gravity vector


// ================================================================
// --------- INTERRUPT DETECTION ROUTINE ----------
// ================================================================
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
mpuInterrupt = true;
}


// ================================================================
// ----------- INITIAL SETUP ----------
// ================================================================
void setup() {
Wire.begin();  // join I2C bus (I2Cdev library doesn't do this automatically)
Serial.begin(115200);  // initialize serial communication
Serial.println(F("Initializing I2C devices..."));
mpu.initialize(); // initialize mpu6050
Serial.println(F("Testing device connections...")); // verify connection
Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
Serial.println(F("Initializing DMP...")); // load and configure the DMP
devStatus = mpu.dmpInitialize();
if (devStatus == 0) {  // make sure it worked (returns 0 if so)
Serial.println(F("Enabling DMP..."));  // turn on the DMP, now that it's ready
mpu.setDMPEnabled(true);
// enable Arduino interrupt detection
Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
attachInterrupt(0, dmpDataReady, RISING);
mpuIntStatus = mpu.getIntStatus();
// set our DMP Ready flag so the main loop() function knows it's okay to use it
Serial.println(F("DMP ready! Waiting for first interrupt..."));
dmpReady = true;
// get expected DMP packet size for later comparison
packetSize = mpu.dmpGetFIFOPacketSize();
} else {
// ERROR!
// 1 = initial memory load failed
// 2 = DMP configuration updates failed
// (if it's going to break, usually the code will be 1)
Serial.print(F("DMP Initialization failed (code "));
Serial.print(devStatus);
Serial.println(F(")"));
}
// configure LED for output
pinMode(LED_PIN, OUTPUT);
pinMode(LEDYELLOW, OUTPUT);
pinMode(LEDRIGHT, OUTPUT);
pinMode(LEDLEFT, OUTPUT);

////////////////////////////////////////
// initialize motor pins///
pinMode(MOTORL0, OUTPUT);
pinMode(MOTORL1, OUTPUT);
pinMode(MOTORR0, OUTPUT);
pinMode(MOTORR1, OUTPUT);
pinMode(ENABLELEFT, OUTPUT);
pinMode(ENABLERIGHT, OUTPUT);
}

// ================================================================
// ---------- MAIN PROGRAM LOOP ----------
// ================================================================
void loop() {
// if MPU 6050 initialization failed exit.
if (!dmpReady) return;
// check if MPU 6050 ready
if (!mpuInterrupt && fifoCount < packetSize) {

 if (mySerial.available() > 0) {
// read the incoming byte:
command = mySerial.read();
switch (command) {
  
 case 'P':
 Kp=Kp+0.01;
 break;
 
 case 'p':
 Kp=Kp-0.01;
 break;
 
 case 'I':
 Ki=Ki+0.01;
 break;
 
 case 'i':
 Ki=Ki-0.01;
 break;
 
 case 'D':
 Kd=Kd+0.03;
 break;
 
 case 'd':
 Kd=Kd-0.03;
 break;
 
 case 'R':
 Kp=4.5;
 Kd=0.5;
 Ki=5.5;
 break;
 
 case 'F'://HİZLAN
hizlanR=hizlanR+2;
hizlanL=hizlanL+2;
 break;
 
 case 'S':
hizlanR=hizlanR-2;
hizlanL=hizlanL-2;
 break;
 
  case 'U':
target=AngleUp*0.05;
 AngleUp++;
 break;
 
  case 'W':
target=AngleDown*0.05;
AngleDown--;
 break;
 
  case 'r':
target=0;
 break;
 
default:
;
} // end of switch
} // end of if
// ================================================================
// ---------- Control Part ----------
// ================================================================
LLastangle=Lastangle;
Lastangle=Aangle;
Aangle=ypr[1]*180/M_PI;
Serial.println(Aangle);
// error is the Desired angle - actual angle
error=target-Aangle;
// PID controller at 50 HZ
Pid=Pid-Kp*(Aangle-Lastangle)+Ki/50*error-Kd*50*(Aangle-2*Lastangle+LLastangle);
// constrain the Control low command to a valid range
Pid=constrain(Pid, -255, 255);
// before updating the motors commad chek the angle of the robot if it is in the desire range:
if (abs(Aangle)<ModeAngle) {
speedR= (int) Pid +hizlanR;
speedL= (int) Pid +hizlanL;
SetMotorSpeed(speedR, speedL);
} else{

Pid=0;
speedR= 0;
speedL= 0;
SetMotorSpeed(speedR, speedL);
}
// end main code
}

// reset interrupt flag and get INT_STATUS byte
mpuInterrupt = false;
mpuIntStatus = mpu.getIntStatus();
// get current FIFO count
fifoCount = mpu.getFIFOCount();
// check for overflow (this should never happen unless our code is too inefficient)
if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
// reset so we can continue cleanly
mpu.resetFIFO();
//Serial.println(F("FIFO overflow!"));
// otherwise, check for DMP data ready interrupt (this should happen frequently)
} else if (mpuIntStatus & 0x02) {
// wait for correct available data length, should be a VERY short wait
while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
// read a packet from FIFO
mpu.getFIFOBytes(fifoBuffer, packetSize);
// track FIFO count here in case there is > 1 packet available
// (this lets us immediately read more without waiting for an interrupt)
fifoCount -= packetSize;
// Get Yaw Pitch Roll
mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetGravity(&gravity, &q);
mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
if(count==1)
calibration();
// Set offset
ypr[0] -= calibrationValues[0];
ypr[1] -=calibrationValues[1];
ypr[2] -=calibrationValues[2];
// blink LED to indicate activity
blinkState = !blinkState;
digitalWrite(LED_PIN, blinkState);
count++;
}
// end main code
}

// ================================================================
// ---------- SetMotor Function ----------
// ================================================================
void SetMotorSpeed(int speedR, int speedL) {
if (speedR>0) {
// set forward
  motor_forwardRight();
  LedLightingForward();
}
else{
// set reverse
   motor_backRight();
  LedLightingBackward();
// change baluse to positive
  speedR=-speedR;
}
if (speedL>0) {
// set forward
  motor_forwardLeft();
  LedLightingForward();
}
else {
// set reverse
  motor_backLeft();
  LedLightingBackward();
// change valuse to positive
  speedL=-speedL;
}
if (speedR<DeadZone){
  speedR=0;
}
else{
  speedR=speedR+Preamp;
}
if (speedL<DeadZone){
  speedL=0;
}
else{
  speedL=speedL+Preamp;
}

  speedR=constrain(speedR, 0, 255);
  speedL=constrain(speedL, 0, 255);
  analogWrite(ENABLERIGHT, speedR);
  analogWrite(ENABLELEFT, speedL);

}
void LedLightingForward(){
  digitalWrite(LEDRIGHT, LOW);
  digitalWrite(LEDLEFT, HIGH);
}
void LedLightingBackward(){
  digitalWrite(LEDRIGHT, HIGH);
  digitalWrite(LEDLEFT, LOW);
}
void motor_forwardLeft(){
digitalWrite(MOTORL0,LOW); 
digitalWrite(MOTORL1,HIGH); 
}

void motor_backLeft(){
digitalWrite(MOTORL0,HIGH); 
digitalWrite(MOTORL1,LOW); 
}
void motor_forwardRight(){
digitalWrite(MOTORR0,LOW); 
digitalWrite(MOTORR1,HIGH);
}
void motor_backRight(){
digitalWrite(MOTORR0,HIGH); 
digitalWrite(MOTORR1,LOW);
}
void calibration(){
digitalWrite(LEDYELLOW, HIGH);
  
  int  i;
  
  for (i = 0; i < 5; i++){
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    }
  }
    
  calibrationValues[0] = ypr[0];
  calibrationValues[1] = ypr[1];
  calibrationValues[2] = ypr[2];
  

  
  for (i = 0; i < 10000; i++){
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    }
      
    calibrationValues[0] = (calibrationValues[0] + ypr[0]) / 2;
    calibrationValues[1] = (calibrationValues[1] + ypr[1]) / 2;
    calibrationValues[2] = (calibrationValues[2] + ypr[2]) / 2;
  
  
  }  
    
  digitalWrite(LEDRIGHT, LOW);
  digitalWrite(LEDLEFT, LOW);
  digitalWrite(LEDYELLOW, LOW);


}
