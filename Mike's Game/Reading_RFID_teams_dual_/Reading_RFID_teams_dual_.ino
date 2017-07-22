/*
  Reading multiple RFID tags, simultaneously!
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 3rd, 2016
  https://github.com/sparkfun/Simultaneous_RFID_Tag_Reader

  Read the user writeable data from a detected tag
*/

#include <SoftwareSerial.h> //Used for transmitting to the device

SoftwareSerial softSerial(2, 3); //RX, TX

#include "SparkFun_UHF_RFID_Reader.h" //Library for controlling the M6E Nano module
RFID nano; //Create instance

int Red_score = 0;
int Blue_score = 0;
int count = 0;
byte red_user[30];
byte blue_user[30];
int red_used = 0;
int blue_used = 0;
int start_time = 0;
int stop_time = 0;
void setup()
{
  Serial.begin(115200);

  while (!Serial);
  Serial.println();
  Serial.println("Initializing...");

  if (setupNano(38400) == false) //Configure nano to run at 38400bps
  {
    Serial.println("Module failed to respond. Please check wiring.");
    while (1); //Freeze!
  }

  nano.setRegion(REGION_NORTHAMERICA); //Set to North America

  nano.setReadPower(500); //5.00 dBm. Higher values may cause USB port to brown out
  //Max Read TX Power is 27.00 dBm and may cause temperature-limit throttling
}

void loop()
{
  while (count <= 0){
    Serial.println(F("Press a key to read user data"));
    while (!Serial.available()); //Wait for user to send a character
    Serial.read(); //Throw away the user's character
    count++;
  }

  //Read the data from the tag
  byte responseType;
  byte myData[64];
  byte myDataLength = sizeof(myData); //Tell readUserData to read up to 64 bytes
  
  responseType = nano.readUserData(myData, myDataLength, 1000); //readUserData will modify myDataLength to the actual # of bytes read

  if (responseType == RESPONSE_SUCCESS)
  {
    UpdateScore(myData);
  }
  else
    Serial.println(F("Error reading tag data"));
  delay(1000);

}

//Gracefully handles a reader that is already configured and already reading continuously
//Because Stream does not have a .begin() we have to do this outside the library
boolean setupNano(long baudRate)
{
  nano.begin(softSerial); //Tell the library to communicate over software serial port

  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered
  softSerial.begin(baudRate); //For this test, assume module is already at our desired baud rate
  while (!softSerial); //Wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while (softSerial.available()) softSerial.read();

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
void UpdateScore(byte myData[]){

  int Red_count = 0;
  int Blue_count = 0;
  if ( myData[0] == 48){
    Blue_count++;
    }
   else if (myData[0] == 49){
    Red_count++;
   }
  if (checkRed(myData) == false){
    UpdateRedScore(Red_count);
  }
  UpdateBlueScore(Blue_count);
}

void UpdateRedScore(int rc){
  if (rc > 0){
    Red_score++;
    Serial.println("Red Score: " + String(Red_score));
  }
}

void UpdateBlueScore(int bc){
  if(bc > 0){
    Blue_score++;
    Serial.println("Blue Score: " + String(Blue_score));
  }  
}
bool checkRed(byte myData[]){
  bool match = false;
  if (myData[0] == 49){
    for (int i = 0; i < red_used; i++){
      if (myData[1] == red_user[i]){
        match = true;
      }
    }
    if (match == false){
      red_user[red_used] = myData[1];
      red_used++;
      start_time = millis();
    }
  }
  stop_time = millis();
  if (start_time != 0 && (stop_time - start_time) > 5000){
    red_used = 0;
  }
  return match;
}


bool checkBlue(byte myData[]){
  bool match = false;
  if (myData[0] == 49){
    for (int i = 0; i < blue_used; i++){
      if (myData[1] == blue_user[i]){
        match = true;
      }
    }
    if (match == false){
      blue_user[blue_used] = myData[1];
      blue_used++;
      start_time = millis();
    }
  }
  stop_time = millis();
  if (start_time != 0 && (stop_time - start_time) > 5000){
    blue_used = 0;
  }
  return match;
}

