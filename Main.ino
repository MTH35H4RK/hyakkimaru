#include <Wire.h>
#include "ColorDetector.h"

void setup() {
  Serial.begin(9600);
  PrepareColorSensor();
}

void loop() {
  if(DetectRed()){
    Serial.println("red detected");
  }else{
    Serial.println("no red detected");
  }
}