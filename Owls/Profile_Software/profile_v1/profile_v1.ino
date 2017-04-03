/* Owl Software for Profile School, Version 1.0
    By Marc Bucchieri at White Mountain Science, Inc/

    This sketch builds off examples for the Adafruit Feather M0 with
  LoRa Radio, found in their hook-up guide here: 
  https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/overview
  Software updates will be posted to our github repository at:
  https://github.com/wmsi/owl_dev/tree/master/Profile%20Software

    This sketch is designed for use with a modular transmitter/ receiver
  Owl. The mode of each individual Owl can be changed by sliding the 
  switch on the inside left side of the unit, as indicated by the SELECT
  MODE label. After changing the mode switch the unit must be manually 
  reset (switch off then back on) for the mode to change. 
  
    Once powered on, the Owl will either begin transmitting or listening for 
  signals from other Owls. In transmit mode, the fourth LED (from the left)
  will flash every time a new message is sent out. If it gets a return signal 
  from another owl, the third LED will also flash. In receive mode, the four
  LEDs will flash back and forth until the first signal is received from the 
  transmitter. Once a transmission is received, the four LEDs will stop 
  flashing and 1-4 LEDs flash the signal strength of each incoming transmission. 
  
    The signal strength can also be indicated with the internal speaker by
  using the "hoot" feature. Simply hold down the red button in the center 
  of the Owl to hear the signal strength of incoming transmissions. The
  beeps will increase in speed and pitch for higher signal strength (aka
  closer) transmissions.
  
*/

// include libraries for talking to the radio
#include <SPI.h>
#include <RH_RF95.h>

/* radio settings for feather m0  */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// battery voltage checker
#define VBATPIN       A7

// radio frequency and broadcast power
#define RF95_FREQ 915.0
//#define TXPOWER   23
int tx_power = 5;

// define send rate, sensor IDs and pins
#define SENDDELAY     1000
static long send_time = -1;

// Define sensor IDs and pin numbers for data logging
#define LIGHTID       "01"
#define LIGHTPIN      A0
#define LIGHTSENSOR   false

#define TEMPID        "02"
#define TEMPPIN       A1
#define TEMPSENSOR    false

// indicator LEDs
const int LED[4] = {13,10,11,12};

// check battery voltage
#define VBAT          A0
#define VMAX          4.0
#define VMIN          2.8     // from datasheet for 3.7V, 400mAh LiPo battery


// Create an object for the radio
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Resonant frequency for internal frequency. Audio feedback will be 
// centered (pitch-wise) around this frequency for maximum volume
#define CENTER_FREQ 2000
#define SPEAKER 5         // speaker output pin

// Pin for the mode select switch
#define MODESELECT    6
#define TRANSMIT      0
#define RECEIVE       1
static bool mode;

// Change this to true only if you add an LCD screen. Right now the only
// supported screen is a 16x2 character Serial-enabled screen like this:
// https://www.sparkfun.com/products/9066
#define LCD   false

// Change this to true only if the device will be plugged in a computer
// with Serial monitor
#define SERIAL false

// Has the first message been received?
static bool first_rx = false;

/*
 * Setup runs once to get the Owl woken up and ready to go. This involves
 * setting up pins as inputs/ outputs, initalizing the radio, and activating
 * Serial output and/ or the LCD screen. This is also where we choose the 
 * mode as transmit/ receive and flash the LEDs to indicate battery voltage.
 */
void setup() 
{    
  pinMode(RFM95_RST, OUTPUT);
  pinMode(SPEAKER, OUTPUT);
  pinMode(MODESELECT, INPUT);
  for(int i=0;i<4;i++)
    pinMode(LED[i], OUTPUT);
  digitalWrite(13, LOW);
  digitalWrite(RFM95_RST, HIGH);

  if(SERIAL) {
    while (!Serial);
    Serial.begin(9600);
    delay(100);
  }

  // set up Serial1 for the LCD
  if(LCD) {
    Serial1.begin(9600);
    delay(100);
    clearLCD();
  }
    
  // Reset and initialize the radio the radio, then set the transmit power
  // as high as possible
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    while (1);
  }

  if (!rf95.setFrequency(RF95_FREQ)) {
    while (1);
  }

  rf95.setTxPower(tx_power, false);

  // indicate remaining voltage on the battery by flashing 1-4 LEDs
  flashVoltage();

  // choose transmit or receive mode
  String display_mode;
  if(digitalRead(MODESELECT)) {
    mode = RECEIVE;
    display_mode = "Receive Mode";
  } else {
    mode = TRANSMIT;
    display_mode = "Transmit Mode";
  }
  
  if(LCD) 
    writeLine(display_mode,1,0);
  if(SERIAL)
    Serial.println(display_mode);
}


/*
 * Once setup() is over, loop() runs on repeat until the Owl is turned off.
 * 
 * In Transmit mode, the Owl builds a message, sends it, and looks for a 
 * reply from the receiver. The message usually consists of the time (in
 * milliseconds) that the Owl has been running, data from any sensors that 
 * are connected, and GPS location data if there is a GPS. Messages are 
 * sent at regular intervals, with the time in between defined by the
 * SENDDELAY constant up top.
 * 
 * In Receiver mode, the Owl patiently listens, pulsing the red LED, until
 * the first message is received from the transmitter. Then the Owl flashes
 * 1-4 LEDs each time a message is received to indicate signal strength. You
 * can also hear signal strength via the internal speaker by pushing the red
 * button in the center of the Owl. Beeps will become faster and higher-
 * pitched as signal strength increases.
 */
void loop() {
  if(mode == TRANSMIT) {
    static String send_string;
    static int send_length = 0;
    static char send_buffer[RH_RF95_MAX_MESSAGE_LEN];
  
    send_string = String(String(millis(), DEC) + ";");
    if(send_time == -1 || millis() - send_time > SENDDELAY) {
      if(LIGHTSENSOR) {
        float light_reading = analogRead(LIGHTPIN) * (1000.0/1023);
        send_string = addSensorData(send_string, LIGHTID, light_reading, 0);
      }
    
      if(TEMPSENSOR) {
        float voltage = (analogRead(TEMPPIN) * 0.00322581);
        float tempC = (voltage - 0.5) * 100.0;
        send_string = addSensorData(send_string, TEMPID, tempC, 2);
      }
      // add more sensors here
    
      // send transmission
      digitalWrite(LED[3], HIGH);
      send_length = min(RH_RF95_MAX_MESSAGE_LEN, send_string.length());
      for (byte i = 0; i < send_length; i++) {
        send_buffer[i] = send_string[i];
      }
    
      rf95.send((uint8_t *)send_buffer, send_length);
      
      delay(20);
      rf95.waitPacketSent();
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
    
      if (rf95.waitAvailableTimeout(250)) { 
        // Should be a reply message for us now   
        if (rf95.recv(buf, &len)) {
          if(SERIAL) {
            Serial.print("Millis: ");
            Serial.print(send_string);
            Serial.print("            Reply RSSI: ");
            Serial.println(rf95.lastRssi(), DEC);  
          } else {
            digitalWrite(LED[2], HIGH);
            delay(250);
          }
//          checkTxPower(rf95.lastRssi());
        } else {
          if(SERIAL)
            Serial.println("Receive failed");
        }
      } else if(SERIAL) {
        Serial.println("No reply.");
      }
      digitalWrite(LED[2], LOW);
      digitalWrite(LED[3], LOW);
    
      send_time = millis();
      send_string = "";
    }

  // This next section of code handles Receive mode.
  } else {   
    while(!first_rx && !rf95.available()) {
      pulseLEDs();             
    }

    if (rf95.available()) {
      first_rx = true;
      
      if(LCD) 
        clearLCD();
      
      // Should be a message for us now   
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      
      if (rf95.recv(buf, &len)) {
        String recd_string = String((char*)buf);
        if(SERIAL) {
          RH_RF95::printBuffer("Received: ", buf, len);
          Serial.print("Got: ");
          Serial.println((char*)buf);
          Serial.print("RSSI: ");
          Serial.println(rf95.lastRssi(), DEC);
        }
        BlinkSS(rf95.lastRssi(), 250);
        delay(10);
        // Send a reply
        uint8_t data[] = "ACK";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();

        if (LCD) {
          writeLine(String("RSSI:" + String(rf95.lastRssi())), 1, 4);
          delay(100);
          writeLine(recd_string, 2, 0);
        }
        audioFeedback(rf95.lastRssi());

      }
    }
  }
}

/*
 * add data from a sensor to the upcoming transmission. 
 * Transmission format is [sensor id]:[sensor value],...,[sensor value];
 * 
 * send_string: existing message String
 * id: sensor ID number
 * reading: sensor value
 * dec_places: decimal places to display
 */
String addSensorData(String send_string, String id, float reading, int dec_places) {
  send_string = String(send_string + id);
  send_string = String(send_string + ":");
  send_string = String(send_string + String(reading, dec_places));
  send_string = String(send_string + ";");
  return send_string;
}

/*
 * Pulse the four LEDs back and forth while waiting for the first transmission.
 */
void pulseLEDs() {
  for(int i=0; i<4; i++) {
    digitalWrite(LED[i], HIGH);
    delay(100);
    digitalWrite(LED[i], LOW);
  }
  for(int i=2; i>0; i--) {
    digitalWrite(LED[i], HIGH);
    delay(100);
    digitalWrite(LED[i], LOW);
  }
}

// Blink an LED for a given number of ms
void Blink(byte PIN, int DELAY_MS)
{
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

// Blink 4 LEDs to show signal strength
void BlinkSS(int rssi, int delay_ms) {
  int num_leds = 1;
  if(rssi > -50) {
    num_leds = 4; 
  } else if(rssi > -70) {
    num_leds = 3;
  } else if(rssi > -90) {
    num_leds = 2;
  }
  for(int i=num_leds-1; i>=0; i--) 
    digitalWrite(LED[i], HIGH);

  delay(delay_ms);
  
  for(int i=0; i<4; i++)
    digitalWrite(LED[i], LOW);
}

/*
 * Beep the speaker to indicate signal strength.
 * 
 * rssi: signal strength in dBm (usually -20 - -120)
 */
void audioFeedback(int rssi) {
  int scale_factor = 10;
  int center_rssi  = 75;

  int output_freq = CENTER_FREQ + (center_rssi + rssi)*scale_factor;
  tone(SPEAKER, output_freq);
}

// display the battery voltage on startup by flashing 1-4 LEDs
void flashVoltage() {
  // check and print battery power
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  int num_leds = map(measuredvbat, VMIN, VMAX, 1, 4);
  
  if(SERIAL)
    Serial.print("VBat: " ); Serial.println(measuredvbat);  

  for(int i=num_leds-1; i>=0; i--) 
    digitalWrite(LED[i], HIGH);

  delay(2000);
  
  for(int i=0; i<4; i++)
    digitalWrite(LED[i], LOW);
}

/*
 * This function get called to clear the 162 LCD screen between writing 
 * to it. If you don't do this, new characters get written on top of the old
 * ones and things get messy.
 */
void clearLCD() {
  Serial1.write(254); // move cursor to beginning of first line
  Serial1.write(128);

  Serial1.write("                "); // clear display
  Serial1.write("                ");
  delay(50);
}

/*
 * Use this function to write to the LCD screen. The function must be called
 * once for each line you want to write. 
 * 
 * str: String to be displayed on the screen (1-16 characters)
 * line: Line of the screen to write to (1-2)
 * pos: position on the screen to start writing at
 * 
 * So if you call writeLine("Hello World!", 3, 2); 
 * your screen would look like this:
 * 
 *    |                |
 *    |  Hello World!  |
 */
void writeLine(String str, int line, int pos) {
  char temp[16] = "               ";
  int num_chars = str.length();
  Serial1.write(254); // move cursor to beginning of first line
  switch(line) {
    case 2:
      Serial1.write(191+pos);
    case 1:
      Serial1.write(128+pos);
  }
  delay(10);
  for(int i=0; i<num_chars; i++) {
    temp[i] = str[i];
  }
  Serial1.write(temp);
  delay(100);
}

// set the Tx power appropriately based on return RSSI. If the receiver 
// is close (high return rssi) we want to attenuate the transmit power
void checkTxPower(int rssi) {
  int last_tx_power = tx_power;
  Serial.print("checking rssi: "); Serial.println(rssi);
  if(rssi >-65) {
    tx_power = 5;
  } else if(rssi > -80) {
    tx_power = 7;
  } else if(rssi > -90) {
    tx_power = 9;
  } else {
    tx_power = 20;
  }
  
  if(last_tx_power != tx_power) {
    if(SERIAL) {
      Serial.print("setting power level: "); Serial.println(tx_power);
    }
    rf95.setTxPower(tx_power);
  }
}

