// SparkFun Serial LCD example 1
// Clear the display and say "Hello World!"

// This sketch is for Arduino versions 1.0 and later
// If you're using an Arduino version older than 1.0, use
// the other example code available on the tutorial page.

// Use the softwareserial library to create a new "soft" serial port
// for the display. This prevents display corruption when uploading code.
//#include <SoftwareSerial.h>

// Attach the serial display's RX line to digital pin 2
//SoftwareSerial mySerial(4,3); // pin 2 = TX, pin 3 = RX (unused)

void setup()
{
  Serial1.begin(9600); // set up serial port for 9600 baud
  delay(500); // wait for display to boot up
}

void loop()
{
  Serial1.write(254); // move cursor to beginning of first line
  Serial1.write(128);

  Serial1.write("                "); // clear display
  Serial1.write("                ");

  Serial1.write(254); // move cursor to beginning of first line
  Serial1.write(128);
 
  Serial1.write("Hello, world!");

  while(1); // wait forever
}

