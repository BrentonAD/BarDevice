/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/brent/OneDrive/Documents/_UNIVERSITY/_MASTEROFAI/2022Tri1/SIT730-EmbeddedSystemDevelopment/Project/ParticleFirmware/BarDevice/src/BarDevice.ino"
/*
 * Project BarDevice
 * Description:
 * Author:
 * Date:
 */

#include <math.h>
#include <../lib/SparkFunMMA8452Q/src/SparkFunMMA8452Q.h>
void setup();
void loop();
#line 10 "c:/Users/brent/OneDrive/Documents/_UNIVERSITY/_MASTEROFAI/2022Tri1/SIT730-EmbeddedSystemDevelopment/Project/ParticleFirmware/BarDevice/src/BarDevice.ino"
#define PI 3.141592654

pin_t LED_PIN = D5;

// Assume device starts flat
float pitchFilteredOld = 0;
float pitchRawNew;
float pitchFiltered;

SerialLogHandler logHandler;
MMA8452Q accel;

void setup() {
  pinMode(LED_PIN,OUTPUT);

  accel.begin(SCALE_2G, ODR_50);
}

void loop()
{
	// accel.available() will return 1 if new data is available, 0 otherwise
    if (accel.available()) {
		  // To update acceleration values from the accelerometer, call accel.read();
      accel.read();

      // Calculate pitch based off acceleration of x-axis and z-axis
      // Note: convert from radians to degrees for readability
      pitchRawNew = atan2(accel.cx,accel.cz)*180.0/PI;
      pitchFiltered = 0.95*pitchFilteredOld + 0.05*pitchRawNew;
      
      Log.info("Pitch: %f", pitchFiltered);

      // Fault tolerance: Filter out vibrations with a low pass filter


      if (abs(pitchFiltered)>20) {
        digitalWrite(LED_PIN, HIGH);
      }
      else {
        digitalWrite(LED_PIN, LOW);
      }

      pitchFilteredOld=pitchFiltered;
    }
	
	// No need to delay, since our ODR is set to 1Hz, accel.available() will only return 1
	// about once per second.
}