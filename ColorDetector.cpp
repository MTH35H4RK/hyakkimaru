#include "ColorDetector.h"
#include <Adafruit_TCS34725.h>
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void PrepareColorSensor() {
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1)
      ;
  }
}

int DetectRed() {
  uint16_t clear, red, green, blue;

  tcs.getRawData(&red, &green, &blue, &clear);

  // Calculate total color intensity
  uint32_t sum = clear;

  // Calculate the ratio of red to the total color intensity
  float redRatio = (float)red / sum;

  // You can adjust this threshold based on your specific application
  float redThreshold = 0.5;  // Adjust this threshold as needed

  // Check if the ratio of red is above the threshold
  if (redRatio > redThreshold) {
    return 1;
  } else {
    return 0;
  }
}