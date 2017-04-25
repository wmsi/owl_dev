// SparkFun Serial LCD example 1
// Clear the display and say "Hello World!"

/*
 * The Adafruit Feather M0 does not support softwareSerial;
 * instead, wire the Rx pin of the LCD to the Feather's
 * Tx pin (DIO 1).
 */

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

