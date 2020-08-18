/* 
*** IMU Head tracking w/ windows Joystick output 
*** Utilizes Teensyduino 2.0 and 
*** Code using Adafruit NXP 9DOF breakout board adapted by Albien Fezga
*** Original authors/contriburtors :
*** Paul Greg : https://github.com/paulgreg/headtracke
*** Adafruit & PJRC :  Written by PJRC, adapted by Limor Fried for Adafruit Industries.
*** Planetkris : https://planetkris.com/2014/12/easier-better-arduino-imu-head-tracker/
*/

#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout

#define UPDATE_RATE 100
#define LED_PIN 11
uint32_t timestamp;

//uncomment if in serial mode for debugging
// #define SERIAL_DBG 

Adafruit_Madgwick filter;  
//Adafruit_Mahony filter;

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

int initialHeading; 
bool initialRun;
float newhead;

void setup() {
#if defined(SERIAL_DBG)
  Serial.begin(115200);
  while (!Serial) yield();
#endif
  
  // set pin output
  pinMode(LED_PIN, OUTPUT);
  
  bool initsens = init_sensors();
  if(!cal.begin()) {
    problemLoopLED(2);
  }
  if(!initsens) {
    problemLoopLED(1);
  }
  setup_sensors();
  filter.begin(UPDATE_RATE);
  Wire.setClock(400000); // 400KHz
  timestamp = millis();
  initialRun = true;
  initialHeading = 0;

  #if defined(SERIAL_DBG)
  Serial.print("Init Run"); Serial.println(initialRun);
  #endif
}

void loop() {
  float roll, pitch, yaw;
  float gx, gy, gz;
  static uint8_t counter = 0; // for debug stuff
  if ((millis() - timestamp) < (1000 / UPDATE_RATE)) {
    return;
  }
  // Read the motion sensors

  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);

  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  filter.update(gx, gy, gz, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();


// wait 5s before utilizing values. place unit on table and do not move  
if((millis() - timestamp) > 5000) {
  if(initialRun) {
    initialHeading = yaw;
    initialRun = false;
    #if defined(SERIAL_DBG)
      Serial.println(initialHeading);
    #endif
  }


  // start sending joystick data

  newhead = yaw - initialHeading;
  #if defined(SERIAL_DBG) 
  Serial.print("New heading "); Serial.println(newhead); 
  #endif

  // adjust heading(yaw) to 0 to (+)180 or 0 to (-)180
  if (newhead > 180) {
      newhead = -360 + newhead;
  }
  else if (newhead < -180) {
      newhead = 360 + newhead;
  }

  #if defined(SERIAL_DBG) 
  Serial.print("Normalized heading "); Serial.println(newhead); 
  #endif
  // scale the angle value into the appropriate range
  // joystick output range is from 0 to 1023. 512 is resting position (middle)
  if(newhead < 0) {
    newhead = fscale(-90, 0, 0, 512, newhead, 3); 
  }
  else {
    newhead = fscale(0, 90, 512, 1023, newhead, 3);
  }

  #if defined(SERIAL_DBG) 
  Serial.print("Scaled heading "); Serial.println(newhead); 
  #endif
  // if new scaled value is beyond than joystick limit. set to the limit
  if(newhead < 0) {
    newhead = 0;
  }
  if(newhead > 1023) {
    newhead = 1023;
  }

  #if defined(SERIAL_DBG) 
  Serial.print("Final joy output "); Serial.println(newhead); 
  #endif

  Joystick.Z(newhead);

   #if defined(SERIAL_DBG)
  Serial.print("Roll: "); Serial.println(roll);
  Serial.print("Pitch: "); Serial.println(pitch);
  Serial.print("Yaw: "); Serial.println(yaw);
  Serial.println(" ");
  #endif   
  }
   if (millis() - timestamp > 10) {
    timestamp = millis();
    if (roll < 0) {
      roll = fscale(-25, 0, 0, 512, roll, 0);
    }
    else {
      roll = fscale(0, 25, 512, 1023, roll, 0);
    }
    if (roll < 0) {
      roll = 0;
    }
    if (roll > 1023) {
      roll = 1023;
    }
    Joystick.X(roll);
    
    if (pitch < 0) {
      pitch = fscale(-25, 0, 0, 512, pitch, 0);
    }
    else {
      pitch = fscale(0, 25, 512, 1023, pitch, 0);
    }
    if (pitch < 0) {
      pitch = 0;
    }
    if (pitch > 1023) {
      pitch = 1023;
    }
    Joystick.Y(pitch);
  
}

}
 
void problemLoopLED(int type) {
  // if we are here there is problem with sensor. turn off and restart manually
  if(type == 1) {
    // unable to find sensors
    while(1==1) {
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(250);
    } 
  }
  if(type == 2) {
    while (1==1) {
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(1500);
    }
  }
}

float fscale( float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve) {
  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;


  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin) {
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0) {
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  }
  else // invert the ranges
  {
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;
}
