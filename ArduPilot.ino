#include <Wire.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <PinChangeInt.h>
#include <PID_v1.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define SERIAL_BUFFER_SIZE 128

// Assign your channel in pins
#define AILERONS_IN_PIN 5
#define ELEVATOR_IN_PIN 6

// Assign your channel out pins
#define AILERONS_OUT_PIN 8
#define ELEVATOR_OUT_PIN 9

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define AILERONS_FLAG 1
#define ELEVATOR_FLAG 2

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
volatile uint16_t aileronsInShared;
volatile uint16_t elevatorInShared;

// local copies
float aileronsIn;
float elevatorIn;

float aileronsInLast;
float elevatorInLast;

float aileronsOut = 0;
float elevatorOut = 0;

// These are used to record the rising edge of a pulse in the calcInput functions
uint16_t aileronsInStart;
uint16_t elevatorInStart;

uint32_t ulVariance = 0;
uint32_t ulGetNextSampleMillis = 0;
uint16_t unMaxDifference = 0;

// Servos
Servo aileronsServo;
Servo elevatorServo;

// MPU variables
MPU6050 mpu;                           // mpu interface object

uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t devStatus;                     // device status    
uint16_t packetSize;                   // estimated packet size  
uint16_t fifoCount;                    // fifo buffer size   
uint8_t fifoBuffer[64];                // fifo buffer 

Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;                   // gravity vector for ypr
float ypr[3] = {0.0f,0.0f,0.0f};       // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};

volatile bool mpuInterrupt = false;    //interrupt flag

// Flight parameters
#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30

#define RC_ROUNDING_BASE 50

// PID configuration
#define PITCH_P_VAL 1.2
#define PITCH_I_VAL 0
#define PITCH_D_VAL 0

#define ROLL_P_VAL 0.8
#define ROLL_I_VAL 0
#define ROLL_D_VAL 1

// PID variables
PID pitchReg(&ypr[1], &elevatorOut, &elevatorIn, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID rollReg(&ypr[2], &aileronsOut, &aileronsIn, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, REVERSE);

void setup(){
  
  Serial.begin(115200);
  pinMode(13, OUTPUT);

  initServos();
  initInterrupts();
  initMPU();
  initRegulators();

  for(int i = 0; i<5; i++){
   digitalWrite(13, HIGH);
   delay(1000);
   digitalWrite(13, LOW);
   delay(1000);
  }

  digitalWrite(13, HIGH);
}

void loop(){
   
  while(!mpuInterrupt && fifoCount < packetSize){
    // Do nothing while MPU is not working
  } 

  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
    
    if(bUpdateFlags & AILERONS_FLAG)
    {
      aileronsIn = aileronsInShared;
    }
    
    if(bUpdateFlags & ELEVATOR_FLAG)
    {
      elevatorIn = elevatorInShared;
    }
     
    // clear shared copy of updated flags as we have already taken the updates
    bUpdateFlagsShared = 0;
   
    interrupts();
  }

  getYPR();                          
  computePID();
  updateServos();

  bUpdateFlags = 0;
}

// simple interrupt service routine
void calcAilerons(){
  
  if(PCintPort::pinState)
  {
    aileronsInStart = TCNT1;
  }
  else
  {
    aileronsInShared = (TCNT1 - aileronsInStart)>>1;

    bUpdateFlagsShared |= AILERONS_FLAG;
  }
}

void calcElevator(){
  
  if(PCintPort::pinState)
  {
    elevatorInStart = TCNT1;
  }
  else
  {
    elevatorInShared = (TCNT1 - elevatorInStart)>>1;
    bUpdateFlagsShared |= ELEVATOR_FLAG;
  }
}

void computePID(){
  
 elevatorIn = floor(elevatorIn/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
 aileronsIn = floor(aileronsIn/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;

 elevatorIn = map(elevatorIn, 1000, 2000, PITCH_MIN, PITCH_MAX);
 aileronsIn = map(aileronsIn, 1000, 2000, ROLL_MIN, ROLL_MAX);

 elevatorIn = -elevatorIn;
 aileronsIn = -aileronsIn;
 
 if((elevatorIn < PITCH_MIN) || (elevatorIn > PITCH_MAX)) elevatorIn = elevatorInLast;
 if((aileronsIn < ROLL_MIN) || (aileronsIn > ROLL_MAX)) aileronsIn = aileronsInLast;
 
 elevatorInLast = elevatorIn;
 aileronsInLast = aileronsIn;
 
 ypr[1] = ypr[1] * 180/M_PI;
 ypr[2] = ypr[2] * 180/M_PI;
 
 if(abs(ypr[1]-yprLast[1])>30) ypr[1] = yprLast[1];
 if(abs(ypr[2]-yprLast[2])>30) ypr[2] = yprLast[2];
 
 yprLast[1] = ypr[1];
 yprLast[2] = ypr[2];

 pitchReg.Compute();
 rollReg.Compute();

}

void initMPU(){
 
 Wire.begin();
 mpu.initialize();
 devStatus = mpu.dmpInitialize();
 if(devStatus == 0){
 
   mpu.setDMPEnabled(true);
   attachInterrupt(0, dmpDataReady, RISING);
   mpuIntStatus = mpu.getIntStatus();
   packetSize = mpu.dmpGetFIFOPacketSize();
    
 }
}

void getYPR(){
  
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
    
  if((mpuIntStatus & 0x10) || fifoCount >= 1024){ 
     
    mpu.resetFIFO(); 
    
  }else if(mpuIntStatus & 0x02){
    
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
    mpu.getFIFOBytes(fifoBuffer, packetSize);
      
    fifoCount -= packetSize;
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
  }
}

inline void initInterrupts(){
  
  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(AILERONS_IN_PIN, calcAilerons, CHANGE);
  PCintPort::attachInterrupt(ELEVATOR_IN_PIN, calcElevator, CHANGE);
  
}

inline void initRegulators(){
  
  //PID-Regler rechnet in Grad, die Aussteuerungsgrenzen sind daher in Grad anzugeben.

  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(PITCH_MIN, PITCH_MAX);
  
  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(ROLL_MIN, ROLL_MAX);

}

inline void initServos(){
  
  aileronsServo.attach(AILERONS_OUT_PIN);
  elevatorServo.attach(ELEVATOR_OUT_PIN);

  aileronsServo.writeMicroseconds(1500);
  elevatorServo.writeMicroseconds(1500);

}

inline void dmpDataReady() {
  mpuInterrupt = true;
}

inline void updateServos() {

  aileronsServo.writeMicroseconds(map(aileronsOut, ROLL_MIN, ROLL_MAX, 1100.0, 1900.0));
  elevatorServo.writeMicroseconds(map(elevatorOut, PITCH_MIN, PITCH_MAX, 1000.0, 2000.0));
  
}
