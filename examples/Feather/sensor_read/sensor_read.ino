/* sensor_read.ino
 *  
 *  This sketch demos the Owl's ability to transmit and receive
 *  sensor values. On startup, each Owl boots into transmit 
 *  or receive mode, as determined by the MODESELECT switch.
 *  In transme mode, sensors are activitated as defined in the 
 *  section of code above setup() and readingas are regularly 
 *  transmitted every SENDDELAY milliseconds while the sketch is
 *  running. In receive mode the Owl gathers new data in the 
 *  message String. This String could then be parsed and processed
 *  depending on the specific application (game design, sensor
 *  network, etc.)
 *  
 *  This sketch includes the basic setup procedure for the RFM69
 *  breakout board, as discussed in Sparkfun's hookup guide:
 * https://learn.sparkfun.com/tutorials/rfm69hcw-hookup-guide
 */

// Include the RFM69 and SPI libraries for using the RFM board
#include <SPI.h>
#include <RH_RF95.h>

/* for feather m0  */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Create an instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
#define TXPOWER   23

// mode select switch
#define MODESELECT    6
#define TRANSMIT      0
#define RECEIVE       1
static bool mode;

// define send rate, sensor IDs and pins
#define SENDDELAY     500
unsigned long last_send = 0;

#define LIGHTID       "01"
#define LIGHTPIN      A0
#define LIGHTSENSOR   true

#define TEMPID        "02"
#define TEMPPIN       A1
#define TEMPSENSOR    false

#define ACCELID       "03"
#define ACCELPINX     A0
#define ACCELPINY     A1
#define ACCELPINZ     A2
#define ACCEL         true

// all-purpose button. In this sketch it used to send a 
// "button pressed!" message every time it is pressed
#define BUTTON        6
 
void setup() {
  Serial.begin(9600);

  pinMode(RFM95_RST, OUTPUT);
  pinMode(MODESELECT, INPUT);
//  pinMode(SPEAKER, OUTPUT);

  // Enable the built-in pullup resistor and read a button press
  // any time it gets connected to ground
  pinMode(BUTTON, INPUT_PULLUP);
  
  radioSetup();
}

void loop() {
  String send_string;
  String message;
  if(mode == TRANSMIT) {
    if(millis() - last_send > SENDDELAY) {
      send_string = String(String(millis(), DEC) + ";");
      send_string = readSensors(send_string);
  
      radioSend(send_string); 
      last_send = millis();
    }
  } else {
    // In this section, we'll check with the RFM69HCW to see
    // if it has received any packets:
    if (rf95.available()) {
      message = printMessage();
    }
    /*
     * At this point sensor data may be processed by the receiver
     * or stored for later use. You can use message.indexOf() to
     * parse the String by searching for colons, semicolons, and commas
     */
  }
}

/*
 * Read all of the sensors designated at the top of the sketch and
 * package their data into send_string. Note that some massaging
 * of data will be necessary (i.e. map(reading,0,1023,0,newMax))
 * in order to fit all of the information into one radio transmission.
 */
String readSensors(String send_string) {
  if(LIGHTSENSOR) {
    String light_reading = String(map(analogRead(LIGHTPIN),0,1023,0,100));
    send_string = addSensorData(send_string, LIGHTID, light_reading);
  }
  
  if(TEMPSENSOR) {
    float voltage = (analogRead(TEMPPIN) * 0.00322581);
    float tempC = (voltage - 0.5) * 100.0;
    String temp_reading = String(tempC,2);
    send_string = addSensorData(send_string, TEMPID, temp_reading);
  }

  if(ACCEL) {
    int accelx = map(analogRead(ACCELPINX),0,1023,0,100);
    int accely = map(analogRead(ACCELPINY),0,1023,0,100);
    int accelz = map(analogRead(ACCELPINZ),0,1023,0,100);
    String accel_reading = String(String(accelx) + "," + String(accely) + "," + String(accelz));
    send_string = addSensorData(send_string, ACCELID, accel_reading);
  }
  return send_string;
}

/*
 * add data from a sensor to the upcoming transmission. 
 * Transmission format is [sensor id]:[sensor value],...,[sensor value];
 */
String addSensorData(String send_string, String id, String reading) {
  send_string = String(send_string + id);
  send_string = String(send_string + ":");
  send_string = String(send_string + reading);
  send_string = String(send_string + ";");
  return send_string;
}

/*
   If a new message has been received on the LoRa board,
   convert it to a String and send an acknowledgment
   message. Return the received message, cast as a String
*/
String printMessage() {
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  String message = "";
  
  if (rf95.recv(buf, &len)) {
    message = String((char*)buf);
    Serial.print("Received: "); Serial.println(message);
    Serial.print("RSSI: "); Serial.println(rf95.lastRssi(), DEC);

    // Send a reply
    uint8_t data[] = "ACK";
    rf95.send(data, sizeof(data));
    rf95.waitPacketSent();
  }
  return message;
}

/*
   Broadcast a message via the radio board. Note that, unlike the 
   Arduino-based Owls, The LoRa boards do not have acknowledgments
   built in to their protocol. Therefore in order for the transmit
   unit to receive a reply, the receiver must process the incoming 
   message and send a reply.

   From RHGenericDriver.h:
   waitAvailableTimeout(uint16_t timeout); 
    Starts the receiver and blocks until a received message is 
    available or a timeout
    \param[in] timeout Maximum time to wait in milliseconds.
    \return true if a message is available
*/
bool radioSend(String send_string) {
  int send_length = min(RH_RF95_MAX_MESSAGE_LEN, send_string.length());
  char send_buffer[RH_RF95_MAX_MESSAGE_LEN];
  bool reply_recd = false;
  
  for (byte i = 0; i < send_length; i++) {
    send_buffer[i] = send_string[i];
  }

  Serial.print("sending message: "); Serial.println(send_string);
  rf95.send((uint8_t *)send_buffer, send_length);
  
  delay(20);
  rf95.waitPacketSent();
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(250)) { 
    if (rf95.recv(buf, &len)) {
      Serial.print("Reply RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);  
      reply_recd = true;
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply.");
  }
}

/*
 * Initialize the radio board into transmit or receive mode.
 * By default, radio.setTxPower() is used to set the output
 * power as high as possible.
 * 
 */
void radioSetup() {
  if(digitalRead(MODESELECT)) {
    mode = RECEIVE;
    Serial.println("Receive Mode Selected");
  } else {
    mode = TRANSMIT;
    Serial.println("Transmit Mode Selected");
  }
  
  // manuallly reset the radio module
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  rf95.init();
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(TXPOWER, false);
}

