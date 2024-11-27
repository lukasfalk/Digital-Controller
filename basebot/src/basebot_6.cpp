/***************************************************************************
*   Copyright (C) 2024 by DTU                             *
*   jcan@dtu.dk                                                    *
*
*   Base Teensy firmware
*   build for Teensy 4.1,
*   intended for digital control course
* 
* The MIT License (MIT)  https://mit-license.org/

* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the “Software”), to deal in the Software without restriction, 
* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
* is furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies 
* or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
* THE SOFTWARE. */


#include <stdbool.h>
#include <stdio.h>
#include <math.h>
// Support functions
#include "src/udisplay.h"
#include "src/urobot.h"
#include "src/uusb.h"
#include "src/umotor.h"
#include "src/uimu2.h"
#include "src/uencoder.h"

// ////////////////////////////////////////

// define a structure for log elements
typedef struct
{
float time;
float distance;
float heading;
int state;
float x, y;
float velocity;
float gyro[3];
float acc[3];
float velref;
float turnref;
float turnVel;
} LogData;

int logsMax; // available
LogData * logs; // pointer
int logsCnt; // used

const float wheelBase = 0.13; // meters

/**
* Accumulated pose data */
float distance = 0; // in meters
float heading = 0;  // in radians
float position[2];  // x,y in meters
void resetPose()
{ // reset the pose (and log)
  logsCnt = 0;
  distance = 0;
  heading = 0;
  position[0] = 0;
  position[1] = 0;
}

void setup()   // INITIALIZATION
{ // to be able to print to USB interface
  Serial.begin(12000000);
  // initialize sensors, safety and display
  robot.setup(); // alive LED, display and battery
  motor.setup(); // motor voltage
  encoder.setup(); // motor encoders to velocity
  imu2.setup(); // gyro and accelerometer - include tilt angle
  motor.setPWMfrq(80000);
  // The log data is allocated 
  // using 'malloc', as this utilizes memory
  // that is not usable for normal 
  // local or global variables or arrays.
  logsMax = 300000 / sizeof(LogData); 
  Serial.print("% maximum log samples ");
  Serial.print(logsMax);
  Serial.println();
  logs = (LogData*)malloc(logsMax * sizeof(LogData));
  logsCnt = 0;
}

void printlog()
{
  LogData * d = logs;
  for (int i = 0; i < logsCnt; i++)
  {
    Serial.print(" ");
    Serial.print(d->time,1); // 1
    Serial.print(" ");
    Serial.print(d->state);  // 2
    Serial.print(" ");
    Serial.print(d->velref,2);  // 3
    Serial.print(" ");
    Serial.print(d->turnref,2);  // 4
    Serial.print(" ");
    Serial.print(d->turnVel,3);  // 5
    Serial.print(" ");
    Serial.print(d->distance,4); // 6
    Serial.print(" ");
    Serial.print(d->heading,5);  // 7
    Serial.print(" ");
    Serial.print(d->x,4);  // 8
    Serial.print(" ");
    Serial.print(d->y,4);  // 9
    Serial.print(" ");
    Serial.print(d->gyro[0],4);  // 10
    Serial.print(" ");
    Serial.print(d->gyro[1],4);  // 11
    Serial.print(" ");
    Serial.print(d->gyro[2],4);  // 12
    Serial.print(" ");
    Serial.print(d->acc[0],4);  // 13
    Serial.println();
    d++;
  }
}

/**
 * Global variables for test sequence */
int state = 0;               // actual state
uint32_t startTime = 0;      // time, when 'start' was pressed (in milliseconds)
float desiredValue = 0;      // desired (reference) value send to controller
uint32_t sampleTimeUs = 500; // sample time for the whole system
float gear = 9.6;           // gear ratio
float wheelradius = 0.03;  // wheel radius in meters
bool forward = false;
bool backward = false;
bool rotate_left = false;
bool rotate_right = false;
//float distance = 0.0;      // distance in meters
/**
 * Test sequence,
 * that is what should happen when */
void testSequence()
{ // this function is called at every sample time
  // and should never wait in a loop.
  // Update variables as needed and return.
  bool button = false;
  //
  // this is a state machine
  // state 0: wait for start button press
  // other states are part of a sequence
  switch (state)
  { // run mission, initial value
    case 0:
      button = digitalReadFast(PIN_START_BUTTON);
      if (button or robot.missionStart)
      { // starting a sequence
        //
        startTime = millis();
        // update of display takes too long time for fast sampling
        // so, disable during sequence.
        display.useDisplay = false;
        // go to next state
        desiredValue = 0;
        state = 1;
      }
      break;
    case 1:
      // first 1 meter
      if (distance < 1)// test if this state is finished
      { // change to next values
        desiredValue = 3;
        forward = true;
      } else {
        forward = false;
        state = 2;
      }
      break;
    case 2:
      // circular path
      if (distance >= 1 && distance < 4.14)// test if this state is finished
      { // change to next values
        desiredValue = 3;
        //motor.setPWMfrq(2000);
      } else {
        backward = true;
        state = 3;
      }
      break;
    case 3:
      if (heading > -3.145/2)// test if this state is finished
      { // change to next values
        desiredValue = 3;
        //motor.setPWMfrq(2020);
        state = 99;
      }
      break;

    default:
      // back to start
      state = 0;
      desiredValue = 0;
      motor.motorVoltage[0] = 0; // left motor
      motor.motorVoltage[1] = 0;  // right motor
      display.useDisplay = true;
      // print som results
      Serial.print("% Set Sample time ");
      Serial.print(sampleTimeUs);
      Serial.print(" usec. Measured Sample time ");
      Serial.print(encoder.sampleTime_us);
      Serial.print(" usec.");
      Serial.println();
      break;
  }
}

/**
 * make the control */
void controlUpdate()
{ // do control during a mission only.
  if (state > 0)
  { // running a sequence, so do control
   // Stuff from group work 5, regarding P-controller
    float ref[2] = {-desiredValue, desiredValue};
    const float maxVoltage = 9.0;
    float e[2]; // velocity error (2 motors)
    float u[2]; // controller output
    float kp = 0.116;
    for (int m = 0; m < 2; m++)
    {
      e[m] = ref[m] - encoder.motorVelocity[m];
      u[m] = kp * e[m];
      // limit
      if (u[m] > maxVoltage)
        u[m] = maxVoltage;
      else if (u[m] < -maxVoltage)
        u[m] = -maxVoltage;
      // set actuators
      motor.motorVoltage[m] = u[m];
    }
  

    float ref = desiredValue;
    //
    // set actuators
    int a0 = analogRead(A0);
    int a1 = analogRead(A1);
    if (forward)
    {
      motor.motorVoltage[0] = -ref[0]; // left motor
      motor.motorVoltage[1] = ref[1];  // right motor
    } else if (rotate_left)
    {
      motor.motorVoltage[0] = -ref[0]; // left motor
      motor.motorVoltage[1] = -ref[1];  // right motor
    } else if (rotate_right)
    {
      motor.motorVoltage[0] = ref[0]; // left motor
      motor.motorVoltage[1] = ref[1];  // right motor
    } else if (backward)
    {
      motor.motorVoltage[0] = ref[0]; // left motor
      motor.motorVoltage[1] = -ref[1];  // right motor
    }
    
    const float toms = wheelradius/gear;
    if (logsCnt < logsMax)
    {
    logs[logsCnt].time = float(micros() - startTime*1000)/1000.0;
    logs[logsCnt].distance = distance;
    logs[logsCnt].heading = heading;
    logs[logsCnt].state = state;
    logs[logsCnt].x = position[0];
    logs[logsCnt].y = position[1];
    float v = (-encoder.motorVelocity[0] + encoder.motorVelocity[1])*toms/2;
    logs[logsCnt].velocity = v;
    logs[logsCnt].gyro[0] = imu2.gyro[0];
    logs[logsCnt].gyro[1] = imu2.gyro[1];
    logs[logsCnt].gyro[2] = imu2.gyro[2];
    logs[logsCnt].acc[0] = imu2.acc[0];
    logs[logsCnt].acc[1] = imu2.acc[1];
    logs[logsCnt].acc[2] = imu2.acc[2];
    logs[logsCnt].velref = desiredValue;
    //logs[logsCnt].turnref = desiredTurnValue;
    float turnRate = (encoder.motorVelocity[1] - (-encoder.motorVelocity[0]))*toms/wheelBase;
    logs[logsCnt].turnVel = turnRate;
    logsCnt++;
    }
    //
    if (false)
    { // see the result
      Serial.print(float(micros() - startTime*1000)/1000.0);
      Serial.print(" ");
      Serial.print(ref[0]);
      Serial.print(" ");
      Serial.print(-encoder.motorVelocity[0]);
      Serial.print(" ");
      Serial.print(encoder.motorVelocity[1]);
      Serial.print(" ");
      Serial.print(a0);
      Serial.print(" ");
      Serial.print(a1);
      Serial.println();
    }
  }
}

//////////////// Controllers //////////////////////
float integrator(float taui, float e, float & xk, float & uk){
  const float T = sampleTimeUs / 1e6;
  float x = e/taui;
  float u = T/2*(x + xk) + uk;
  xk = x;
  uk = u;
  return u;
}

float velocityController(float ref, float vel){
  const float Kp = 0.03;
  const float taui = 0.0855;
  float ekp = Kp * (ref - vel);
  return ekp + integrator(taui, ekp, x1k, u1k);
}

float turnController(float ref, float vel){
  const float Kp = 0.03;
  const float taui = 0.0855;
  float ekp = Kp * (ref - vel);
  return ekp + integrator(taui, ekp, x2k, u2k);
}

//old values
float x1k = 0, x2k = 0, u1k = 0, u2k = 0;

void update(){
  const float Kp = 0.1458;
  const float taui = 0.1357;
  float ref = desiredValue*gear/wheelradius;
  //motor 1
  float ekp1 = Kp * (-ref - encoder.motorVelocity[0]);
  float u1 = ekp1 + integrator(taui, ekp1, x1k, u1k);
  motor.motorVoltage[0] = u1;
  //motor 2
  float ekp2 = Kp*(ref - encoder.motorVelocity[1]);
  float u2 = ekp2 + integrator(taui, ekp2, x2k, u2k);
  motor.motorVoltage[1] = u2;
}



//float heading = 0;
int32_t encoderLast[2];
void updatePose(){
  //distance for each encoder tick
  //12 magnets 2 sensors each 2 flanks
  const float distPerTick = wheelradius/gear/12.0/4.0*2.0*M_PI;
  //save encoder values
  int32_t enc[2] = {encoder.encoder[0], encoder.encoder[1]};
  float distwheel[2]; //forward distance
  distwheel[0] = -float(enc[0] - encoderLast[0])*distPerTick;
  distwheel[1] = float(enc[1] - encoderLast[1])*distPerTick;
  float ddist = (distwheel[0] + distwheel[1])/2.0;
  distance += ddist;
  //positive angle in CCV (in radians)
  heading += (distwheel[1] - distwheel[0])/wheelBase;
  encoderLast[0] = enc[0];
  encoderLast[1] = enc[1];
  //position update
}

/**
* Main loop
* */
void loop ( void )
{ // init sample time
  uint32_t nextSample =0;
  while ( true )
  { // main loop
    // get time since start in microseconds
    uint32_t us = micros();
    // loop until time for next sample
    if (us > nextSample) // start of new control cycle
    { // advance time for next sample
      nextSample += sampleTimeUs;
      // read sensors
      imu2.tick();
      encoder.tick();
      updatePose();
      // advance test sequence (default is wait for start button)
      testSequence();
      // make control actions
      controlUpdate();
      // give value to actuators
      motor.tick();
      // support functions
      robot.tick(); // measure battery voltage etc.
      display.tick(); // update O-LED display
    }
    usb.tick(); // listen to incoming from USB
  }
}