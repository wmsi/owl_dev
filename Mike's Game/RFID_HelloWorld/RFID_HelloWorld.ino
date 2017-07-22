/* Hello World for RFID MultiScanner.  Takes in an RFID and 
 *  wait until one is read.  Then it Prints Hello World.
 */

#include <SoftwareSerial.h> //Used for transmitting to the device

SoftwareSerial softSerial(2, 3); //RX, TX

#include "SparkFun_UHF_RFID_Reader.h" //Library for controlling the M6E Nano module
RFID nano; //Create instance

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);
  Serial.println();
  Serial.println(F("Initializing"));

  if(setupNano(38400) == false){
    Serial.println("fail");
    while (1);
  }
  nano.setRegion(REGION_NORTHAMERICA);

  nano.setReadPower(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(F("Press a key to scan for a tag"));
  while(!Serial.available());
  Serial.read();
  byte myEPC[12];
  byte myEPClength;
  byte responseType = 0;

  while (responseType != RESPONSE_SUCCESS){
    myEPClength = sizeof(myEPC);

    responseType = nano.readTagEPC(myEPC, myEPClength, 500);
    Serial.println(F("Searching for tag"));
  }
  Serial.print(F("Hello World \n"));
  delay(3000);
  
}

//Gracefully handles a reader that is already configured and already reading continuously
//Because Stream does not have a .begin() we have to do this outside the library
boolean setupNano(long baudRate)
{
  nano.begin(softSerial); //Tell the library to communicate over software serial port

  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered
  softSerial.begin(baudRate); //For this test, assume module is already at our desired baud rate
  while(!softSerial); //Wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while(softSerial.available()) softSerial.read();
  
  nano.getVersion();

  if (nano.msg[0] == ERROR_WRONG_OPCODE_RESPONSE)
  {
    //This happens if the baud rate is correct but the module is doing a ccontinuous read
    nano.stopReading();

    Serial.println(F("Module continuously reading. Asking it to stop..."));

    delay(1500);
  }
  else
  {
    //The module did not respond so assume it's just been powered on and communicating at 115200bps
    softSerial.begin(115200); //Start software serial at 115200

    nano.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg

    softSerial.begin(baudRate); //Start the software serial port, this time at user's chosen baud rate
  }

  //Test the connection
  nano.getVersion();
  if (nano.msg[0] != ALL_GOOD) return (false); //Something is not right

  //The M6E has these settings no matter what
  nano.setTagProtocol(); //Set protocol to GEN2

  nano.setAntennaPort(); //Set TX/RX antenna ports to 1

  return (true); //We are ready to rock
}

