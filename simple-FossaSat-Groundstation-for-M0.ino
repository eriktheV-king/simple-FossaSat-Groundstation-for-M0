/*
 * filename simple-FossaSat-Groundstation-for-M0
 * version: 3
 * Vking's FOSSA MODIFIED Ground Station Example
 * by Erikthevking http://lab.vking.earth
 * 
 * this is based on the FOSSA Ground Station Example 
 * but I desactivated the transmitting part of the code
 * so that we do not interfere with the FOSSASAT-1 satellite setup process
 * 
 * to enable send mode, uncomment line :  // printControls(); 
 *                       and uncomment : // process serial command
 *                        in loop function
 * 
 * tested on SAMD-21G18 Adafruit Feather M0 with SX1278 featherwing and SSD1306 128x32 Oled display featherwing
 *                                                                         (Uses pins 20, 21 for SDA, SCL)
 * 
 * added Adafruit Featherwing with SSD1306 Oled 128x32
 * so that we don't need to have a serial monitor all the time.
 * On reception, the display will show a counter counting upwards on every successful reception 
 * on error, errorcode will shop up on display, and error counter will count upwards on every error
 * On succesful reception, received parameters will be shown in serial monitor only
 *
 */

/*
 * FOSSA Ground Station Example
 *
 * Tested on Arduino Uno and SX1278, can be used with any LoRa radio
 * from the SX127x or SX126x series. Make sure radio type (line 21)
 * and pin mapping (lines 26 - 29) match your hardware!
 *
 * References:
 *
 * RadioLib error codes:
 * https://jgromes.github.io/RadioLib/group__status__codes.html
 *
 * FOSSASAT-1 Communication Guide:
 * https://github.com/FOSSASystems/FOSSASAT-1/blob/master/FOSSA%20Documents/FOSSASAT-1%20Comms%20Guide.pdf
 *
 */

// include all libraries
#include <RadioLib.h>
#include <FOSSA-Comms.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include "Adafruit_SSD1306.h"
#include <string.h>

#define RADIO_TYPE        SX1278  // type of radio module to be used
//#define RADIO_SX126X            // also uncomment this line when using SX126x!!!

// SX1262 has the following connections:
// NSS pin:   10
// DIO1 pin:  2
// BUSY pin:  9

/*
// pin definitions
#define CS                10
#define DIO0              2
#define DIO1              3
#define BUSY              9
*/

/* Adafruit Feather m0 w/wing*/
#define CS   10 // RFM95_CS 10  // "B"
#define DIO0 6  // RFM95_INT 6  // "D"
#define DIO1 11 // RFM95_RST 11 // "A"

/* Adafruit Feather 32u4 w/wing
#define RFM95_RST 11 // "A"
#define RFM95_CS 10 // "B"
#define RFM95_INT 2 // "SDA" (only SDA/SCL/RX/TX have IRQ!)
*/

/* Adafruit Feather32u4 without Featherwing shield
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7 
 */

// modem configuration
#define FREQUENCY         436.7   // MHz
#define BANDWIDTH         125.0   // kHz
#define SPREADING_FACTOR  11
#define CODING_RATE       8       // 4/8
#define SYNC_WORD_7X      0xFF    // sync word when using SX127x
#define SYNC_WORD_6X      0x0F0F  //                      SX126x

// set up radio module
#ifdef RADIO_SX126X
  RADIO_TYPE radio = new Module(CS, DIO0, BUSY);
#else
  RADIO_TYPE radio = new Module(CS, DIO0, DIO1);
#endif

// flags
volatile bool interruptEnabled = true;
volatile bool transmissionReceived = false;

// satellite callsign
char callsign[] = "FOSSASAT-1";

int16_t packetnum = 0; // packet counter, we increment per xmission
int16_t faultynum = 0; // packet counter, we increment per xmission

Adafruit_SSD1306 display = Adafruit_SSD1306();

void cDsply() {
   display.clearDisplay();
   display.display();
   display.setCursor(0,0);
}

// radio ISR
void onInterrupt() {
  if(!interruptEnabled) {
    return;
  }

  transmissionReceived = true;
}

// function to print controls
void printControls() {
  Serial.println(F("------------- Controls -------------"));
  Serial.println(F("p - send ping frame"));
  Serial.println(F("i - request satellite info"));
  Serial.println(F("l - request last packet info"));
  Serial.println(F("r - send message to be retransmitted"));
  Serial.println(F("------------------------------------"));
}

void sendPing() {
  Serial.print(F("Sending ping frame ... "));

  // data to transmit
  uint8_t functionId = CMD_PING;

  // build frame
  uint8_t len = FCP_Get_Frame_Length(callsign);
  uint8_t* frame = new uint8_t[len];
  FCP_Encode(frame, callsign, functionId);

  // send data
  int state = radio.transmit(frame, len);
  delete[] frame;

  // check transmission success
  if (state == ERR_NONE) {
    Serial.println(F("sent successfully!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}

void requestInfo() {
  Serial.print(F("Requesting system info ... "));

  // data to transmit
  uint8_t functionId = CMD_TRANSMIT_SYSTEM_INFO;

  // build frame
  uint8_t len = FCP_Get_Frame_Length(callsign);
  uint8_t* frame = new uint8_t[len];
  FCP_Encode(frame, callsign, functionId);

  // send data
  int state = radio.transmit(frame, len);
  delete[] frame;

  // check transmission success
  if (state == ERR_NONE) {
    Serial.println(F("sent successfully!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}

void requestPacketInfo() {
  Serial.print(F("Requesting last packet info ... "));

  // data to transmit
  uint8_t functionId = CMD_GET_LAST_PACKET_INFO;

  // build frame
  uint8_t len = FCP_Get_Frame_Length(callsign);
  uint8_t* frame = new uint8_t[len];
  FCP_Encode(frame, callsign, functionId);

  // send data
  int state = radio.transmit(frame, len);
  delete[] frame;

  // check transmission success
  if (state == ERR_NONE) {
    Serial.println(F("sent successfully!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}

void requestRetransmit() {
  Serial.println(F("Enter message to be sent:"));
  Serial.println(F("(max 32 characters, end with LF or CR+LF)"));

  // get data to be retransmited
  char optData[32];
  uint8_t bufferPos = 0;
  while(bufferPos < 32) {
    while(!Serial.available());
    char c = Serial.read();
    Serial.print(c);
    if((c != '\r') && (c != '\n')) {
      optData[bufferPos] = c;
      bufferPos++;
    } else {
      break;
    }
  }

  // wait for a bit to receive any trailing characters
  delay(100);

  // dump the serial buffer
  while(Serial.available()) {
    Serial.read();
  }

  Serial.println();
  Serial.print(F("Requesting retransmission ... "));

  // data to transmit
  uint8_t functionId = CMD_RETRANSMIT;
  optData[bufferPos] = '\0';
  uint8_t optDataLen = strlen(optData);

  // build frame
  uint8_t len = FCP_Get_Frame_Length(callsign, optDataLen);
  uint8_t* frame = new uint8_t[len];
  FCP_Encode(frame, callsign, functionId, optDataLen, (uint8_t*)optData);

  // send data
  int state = radio.transmit(frame, len);
  delete[] frame;

  // check transmission success
  if (state == ERR_NONE) {
    Serial.println(F("sent successfully!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}

void setup() {

  //check if the display was initialised correctly (and therefore the constant SSD1306_LCDHEIGHT was set correctly)
  if(SSD1306_LCDHEIGHT != 32){ //or 64 for 128x64 OLED display
    asm volatile ("  jmp 0");  //software reset
  }

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // Show image buffer splashscreen.
  display.display();
  delay(2000);

  cDsply();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.println(F("FOSSA  Ground Station"));
  display.println(F(""));
  display.println(F(" FOSSASAT-1 receiver "));    
  display.print  (F("   initializing...   "));
  display.display();
  delay(2000);
  
  Serial.begin(9600);
  delay(2000);
  Serial.println(F("FOSSA Ground Station Demo Code"));

  // initialize the radio
  #ifdef RADIO_SX126X
  int state = radio.begin(FREQUENCY,
                          BANDWIDTH,
                          SPREADING_FACTOR,
                          CODING_RATE,
                          SYNC_WORD_6X);
  #else
  int state = radio.begin(FREQUENCY,
                          BANDWIDTH,
                          SPREADING_FACTOR,
                          CODING_RATE,
                          SYNC_WORD_7X);
  #endif

  cDsply();
    display.println(F("FOSSA  Ground Station"));
    display.println();
  
  if(state == ERR_NONE) {
    Serial.println(F("Radio initialization successful!"));
    display.println(F("   initialization    "));
    display.println(F("     successful !    "));
    display.display();
    delay(3000);   
  } else {
    Serial.print(F("Failed to initialize radio, code: "));
    Serial.println(state);
    display.println(F("Failed to initialize "));
    display.print(F("radio, code: "));
    display.print(state);
    display.display();
    while(true);
  }

  // attach the ISR to radio interrupt
  #ifdef RADIO_SX126X
  radio.setDio1Action(onInterrupt);
  #else
  radio.setDio0Action(onInterrupt);
  #endif

  // begin listening for packets
  radio.startReceive();

  // printControls();  // uncomment to enable send controls
  
  Serial.println(F("Radio waiting for data reception from FOSSASAT-1 ..."));
    cDsply();
    display.println(F("FOSSA  Ground Station"));
    display.println(F("  Radio waiting for  "));
    display.println(F(" data reception from "));
    display.println(F("    FOSSASAT-1 ...   "));
    display.display();
    delay(2000);
     
}

void loop() {
  // check serial data
  if(Serial.available()) {
    // disable reception interrupt
    interruptEnabled = false;
    detachInterrupt(digitalPinToInterrupt(DIO1));

    // get the first character
    char serialCmd = Serial.read();

    // wait for a bit to receive any trailing characters
    delay(50);

    // dump the serial buffer
    while(Serial.available()) {
      Serial.read();
    }

/*    // process serial command
    switch(serialCmd) {
      case 'p':
        sendPing();
        break;
      case 'i':
        requestInfo();
        break;
      case 'l':
        requestPacketInfo();
        break;
      case 'r':
        requestRetransmit();
        break;
      default:
        Serial.print(F("Unknown command: "));
        Serial.println(serialCmd);
        break;
    }
*/

    // set radio mode to reception
    #ifdef RADIO_SX126X
    radio.setDio1Action(onInterrupt);
    #else
    radio.setDio0Action(onInterrupt);
    #endif
    radio.startReceive();
    interruptEnabled = true;
  }

  // check if new data were received
  if(transmissionReceived) {
    // disable reception interrupt
    interruptEnabled = false;
    transmissionReceived = false;

    // read received data
    size_t respLen = radio.getPacketLength();
    uint8_t* respFrame = new uint8_t[respLen];
    int state = radio.readData(respFrame, respLen);

    // check reception success
    if (state == ERR_NONE) {
      // print raw data
      Serial.print(F("Received "));
      Serial.print(respLen);
      Serial.println(F(" bytes:"));
      PRINT_BUFF(respFrame, respLen);
      
      packetnum =  (packetnum + 1);      
      cDsply();
      display.println(F("FOSSA  Ground Station"));
      display.print(F("  "));
      display.print(respLen);
      display.println(F(" bytes"));
      display.println(F("    data received    "));
      display.print(F("    counter : "));
      display.print(packetnum);
      display.display();
      delay(1000);

      // get function ID
      uint8_t functionId = FCP_Get_FunctionID(callsign, respFrame, respLen);
      Serial.print(F("Function ID: 0x"));
      Serial.println(functionId, HEX);

      // check optional data
      uint8_t* respOptData = nullptr;
      uint8_t respOptDataLen = FCP_Get_OptData_Length(callsign, respFrame, respLen);
      Serial.print(F("Optional data ("));
      Serial.print(respOptDataLen);
      Serial.println(F(" bytes):"));
      if(respOptDataLen > 0) {
        // read optional data
        respOptData = new uint8_t[respOptDataLen];
        FCP_Get_OptData(callsign, respFrame, respLen, respOptData);
        PRINT_BUFF(respFrame, respLen);
      }

      // process received frame
      switch(functionId) {
        case RESP_PONG:
          Serial.println(F("Pong!"));
          break;

        case RESP_SYSTEM_INFO:
          Serial.println(F("System info:"));

          Serial.print(F("batteryChargingVoltage = "));
          Serial.println(FCP_Get_Battery_Charging_Voltage(respOptData));

          Serial.print(F("batteryChargingCurrent = "));
          Serial.println(FCP_Get_Battery_Charging_Current(respOptData), 4);

          Serial.print(F("batteryVoltage = "));
          Serial.println(FCP_Get_Battery_Voltage(respOptData));

          Serial.print(F("solarCellAVoltage = "));
          Serial.println(FCP_Get_Solar_Cell_Voltage(0, respOptData));

          Serial.print(F("solarCellBVoltage = "));
          Serial.println(FCP_Get_Solar_Cell_Voltage(1, respOptData));

          Serial.print(F("solarCellCVoltage = "));
          Serial.println(FCP_Get_Solar_Cell_Voltage(2, respOptData));

          Serial.print(F("batteryTemperature = "));
          Serial.println(FCP_Get_Battery_Temperature(respOptData));

          Serial.print(F("boardTemperature = "));
          Serial.println(FCP_Get_Board_Temperature(respOptData));

          Serial.print(F("mcuTemperature = "));
          Serial.println(FCP_Get_MCU_Temperature(respOptData));

          Serial.print(F("resetCounter = "));
          Serial.println(FCP_Get_Reset_Counter(respOptData));

          Serial.print(F("powerConfig = 0b"));
          Serial.println(FCP_Get_Power_Configuration(respOptData), BIN);
          break;

        case RESP_LAST_PACKET_INFO:
          Serial.println(F("Last packet info:"));

          Serial.print(F("SNR = "));
          Serial.print(respOptData[0] / 4.0);
          Serial.println(F(" dB"));

          Serial.print(F("RSSI = "));
          Serial.print(respOptData[1] / -2.0);
          Serial.println(F(" dBm"));
          break;

        case RESP_REPEATED_MESSAGE:
          Serial.println(F("Got repeated message:"));
          for(uint8_t i = 0; i < respOptDataLen; i++) {
            Serial.write(respOptData[i]);
          }
          Serial.println();
          break;

        default:
          Serial.println(F("Unknown function ID!"));
          break;
      }

      printControls();
      if(respOptDataLen > 0) {
        delete[] respOptData;
      }

    } else {
      Serial.println(F("Reception failed, code "));
      Serial.println(state);
      
      faultynum =  (faultynum + 1);      
      cDsply();
      display.println(F("FOSSA  Ground Station"));
      display.println();
      display.println(F("   error received !  "));
      display.print(F("    counter : "));
      display.print(faultynum);
      display.display();
      delay(500);
     
    }

    // enable reception interrupt
    delete[] respFrame;
    radio.startReceive();
    interruptEnabled = true;
  }



      
  
}
