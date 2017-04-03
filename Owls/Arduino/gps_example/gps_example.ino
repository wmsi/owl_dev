// See the hook-up guide for wiring instructions:
// https://learn.sparkfun.com/tutorials/rfm69hcw-hookup-guide

// Uses the RFM69 library by Felix Rusu, LowPowerLab.com
// Original library: https://www.github.com/lowpowerlab/rfm69
// SparkFun repository: https://github.com/sparkfun/RFM69HCW_Breakout


#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define SENDDELAY    1000

#define GPSRX         3
#define GPSTX         4
#define GPS           true
#define DATE          false
#define TIME          true
#define GPSBAUD       9600

// Create a TinyGPS++ object called "gps"
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(GPSRX, GPSTX);

void setup()
{
  // Open a serial port so we can send keystrokes to the module:
  Serial.begin(9600);

  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBAUD);

}

void loop()
{
  static String send_string;
  static long send_time = -1;
  static bool got_location = false;
  
  if(send_time == -1 || millis() - send_time > SENDDELAY) {
    if(GPS && !got_location) {
      while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
          send_string = addGPSData();
          Serial.println(send_string);
          got_location = true;
        }
      }
    }
  }
  
    send_time = millis();
    send_string = "";
    got_location = false;
  
}

String addGPSData() {
  String send_string = "LOC:";
  if(gps.location.isValid()) {
    send_string = String(send_string + String(gps.location.lat(), 5));
    send_string = String(send_string + ",");
    send_string = String(send_string + String(gps.location.lng(), 5));
  } else {
    send_string = String(send_string + "ERR");
  }

  if(DATE) {
    send_string = String(send_string + ";DAT:");
    if(gps.date.isValid()) {
      send_string = String(send_string + String(gps.date.month())); send_string = String(send_string + "/");
      send_string = String(send_string + String(gps.date.day())); send_string = String(send_string + "/");
      send_string = String(send_string + String(gps.date.year())); send_string = String(send_string + "; ");
    } else {
      send_string = String(send_string + "ERR;"); 
    }
  }

  if (TIME) {
    send_string = String(send_string + "TIM:");
    if(gps.time.isValid()) {
      if (gps.time.hour() < 10) send_string = String(send_string + "0"); 
      send_string = String(send_string + String(gps.time.hour()));
      send_string = String(send_string + ":");
      if (gps.time.minute() < 10) send_string = String(send_string + "0"); 
      send_string = String(send_string + String(gps.time.minute()));
      send_string = String(send_string + ";");
//      if (gps.time.second() < 10) send_string = String(send_string + "0"); 
//      send_string = String(send_string + String(gps.time.second()));
//      send_string = String(send_string + ";");
    } else {
      send_string = String(send_string + "ERR;");
    }
  }
  return send_string;
}


