/*
 * Project BarDevice
 * Description:
 * Author:
 * Date:
 */

#include <math.h>
#include <../lib/SparkFunMMA8452Q/src/SparkFunMMA8452Q.h>
#include <../lib/Adafruit_SH1106_0/src/Adafruit_SH1106.h>
#define PI 3.141592654

#define OLED_DC     A4
#define OLED_CS     A5
#define OLED_RESET  A0

Adafruit_SH1106 display(OLED_DC, OLED_RESET, OLED_CS);

SerialLogHandler logHandler;
pin_t LED_PIN = D5;

MMA8452Q accel;

float scaleFactor = display.height()/2.0;

void drawAxis() {
  // Draw Y-axis
  display.drawLine(0,0,0,display.height()-2,WHITE);
  // Draw X-axis
  display.drawLine(0,display.height()-2,display.width(),display.height()-2,WHITE);

  // Draw constant line
  int i;
  for (i=2; i<display.width()-4; i++){
    if(i%3 != 0){
      display.drawPixel(i,1*scaleFactor,WHITE);
    }
  }

  display.display();
}
// Assume device starts flat
float pitchFilteredOld=0;
system_tick_t timeInit=millis();
int dtOld = 0;
int yOld = display.height()-2;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  
  display.begin(SH1106_SWITCHCAPVCC);

  Log.info("Width: %d, Height: %d", display.width(), display.height());

  display.display();
  delay(2000);
  // Clear the buffer.
  display.clearDisplay();
  drawAxis();
  
  accel.begin(SCALE_2G, ODR_400);
}

void loop()
{
  // Declare variables
  float pitchRawNew;
  float pitchFiltered;
  float magAccel;
  system_tick_t timeCurrent;
  int y;
  int dt;

	// accel.available() will return 1 if new data is available, 0 otherwise
    if (accel.available()) {
      // To update acceleration values from the accelerometer, call accel.read();
      accel.read();
      // Calculate pitch based off acceleration of x-axis and z-axis
      // Note: convert from radians to degrees for readability
      pitchRawNew = atan2(accel.cx,accel.cz)*180.0/PI;
      // Fault tolerance: Filter out vibrations with a low pass filter
      pitchFiltered = 0.95*pitchFilteredOld + 0.05*pitchRawNew;
      if (abs(pitchFiltered)>20) {
        digitalWrite(LED_PIN, HIGH);
      }
      else {
        digitalWrite(LED_PIN, LOW);
      }
      pitchFilteredOld=pitchFiltered;

      magAccel = sqrt(pow(accel.cx,2)+pow(accel.cy,2)+pow(accel.cz,2));
      timeCurrent = millis();
      dt = (int)timeCurrent - (int)timeInit;
      y = display.height()-floor(magAccel*scaleFactor)-2;
      if (dt<=display.width()-4){
        display.drawLine(dtOld,yOld,dt,y,WHITE);
        //display.drawPixel(dt,y,WHITE);
        display.display();
        dtOld=dt;
        yOld=y;
      }
      else{
        display.clearDisplay();
        drawAxis();
        display.drawPixel(2,y,WHITE);
        display.display();
        timeInit=timeCurrent;
        dtOld=2;
        yOld=y;
      }
    }

	// No need to delay, since our ODR is set to 1Hz, accel.available() will only return 1
	// about once per second.
}