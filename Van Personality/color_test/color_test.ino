#include <Adafruit_NeoPixel.h>
#include "Color_Definitions.h"

#define PIN 4
#define LED_COUNT 300

// Create an instance of the Adafruit_NeoPixel class called "leds".
// That'll be what we refer to from here on...
Adafruit_NeoPixel leds = Adafruit_NeoPixel(LED_COUNT, PIN, NEO_GRB + NEO_KHZ800);

void setup()
{
  leds.begin();  // Call this to start up the LED strip.
  clearLEDs();   // This function, defined below, turns all LEDs off...
  leds.show();   // ...but the LEDs don't actually update until you call this
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(unsigned long i=0; i<=WHITE; i+=10) {
    for(int j=0; j<LED_COUNT; j++) {
      leds.setPixelColor(j, (i, HEX));
    }
    leds.show();
    Serial.println(i, HEX);
    delay(10);
  }
}

void clearLEDs()
{
  for (int i=0; i<LED_COUNT; i++)
  {
    leds.setPixelColor(i, 0);
  }
}
