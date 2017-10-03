#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>

#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_RI  7

#define LOCK 12
#define LED  13

const int NUM_SMS_SLOTS = 30;

char replyBuffer[255];// this is a large buffer for replies
char callerIDbuffer[32];  //we'll store the SMS sender number in here

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t type;

void wipeAllSmsSlots(void) {
  //wipe all other messages
  for (int i=0; i <= NUM_SMS_SLOTS; i++) {
    fona.deleteSMS(i);
  }
}

void setup() {
  // initialize digital pin 12 and 13 as an output.
  pinMode(LOCK, OUTPUT);
  pinMode(LED, OUTPUT);

  Serial.begin(115200);
  Serial.println(F("Initializing....(May take 3 seconds)"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    //TODO: LED BLINK CODE TO TELL US IF THE FONA IS DEAD
    while (1); //BLOCKING
  }
  type = fona.type();
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  switch (type) {
    case FONA800L:
      Serial.println(F("FONA 800L")); break;
    case FONA800H:
      Serial.println(F("FONA 800H")); break;
    case FONA808_V1:
      Serial.println(F("FONA 808 (v1)")); break;
    case FONA808_V2:
      Serial.println(F("FONA 808 (v2)")); break;
    case FONA3G_A:
      Serial.println(F("FONA 3G (American)")); break;
    case FONA3G_E:
      Serial.println(F("FONA 3G (European)")); break;
    default: 
      Serial.println(F("???")); break;
  }
  
  // Print module IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }
  //Additional options not needed at this point:
  //fona.setGPRSNetworkSettings(F("your APN"), F("your username"), F("your password"));
  //fona.setHTTPSRedirect(true);

  printMenu();

  for (int i=0; i <= 30; i++) {
    uint8_t n = fona.getNetworkStatus();
    Serial.print(F("Network status "));
    Serial.print(n);
    Serial.print(F(": "));
    if (n == 0) Serial.println(F("Not registered"));
    if (n == 1) Serial.println(F("Registered (home)"));
    if (n == 2) Serial.println(F("Not registered (searching)"));
    if (n == 3) Serial.println(F("Denied"));
    if (n == 4) Serial.println(F("Unknown"));
    if (n == 5) Serial.println(F("Registered roaming"));
    if (n == 1) {
      break;//exit loop when network is ready.
    }
    delay(1000);
  }
  
  Serial.println(F("Fona Ready"));
  wipeAllSmsSlots();//remove message history on boot so we don't get in a state that is broken while powered off.
  
  //TODO: change this to a function call, non-blocking (without delays)
  for (int i=0; i <= 10; i++){
    digitalWrite(LED,  HIGH);
    delay(100);
    digitalWrite(LED,  LOW);
    delay(100);
  }
  
}

void printMenu(void) {
  //TODO: customize menu.  We do not need all of these options, and should add other options.
  Serial.println(F("-------------------------------------"));
  Serial.println(F("[?] Print this menu"));

  Serial.println(F("[1] Open lock for 5 seconds"));
  
  Serial.println(F("[a] read the ADC 2.8V max (FONA800 & 808)"));
  Serial.println(F("[b] read the Battery V and % charged"));
  Serial.println(F("[C] read the SIM CCID"));
  Serial.println(F("[i] read RSSI"));
  Serial.println(F("[n] get Network status"));

  // SMS
  Serial.println(F("[N] Number of SMSs"));
  Serial.println(F("[R] Read All SMS"));
  Serial.println(F("[u] Send USSD"));
  
  // Time
  Serial.println(F("[y] Enable network time sync (FONA 800 & 808)"));
  Serial.println(F("[Y] Enable NTP time sync (GPRS FONA 800 & 808)"));
  Serial.println(F("[t] Get network time"));

  // GPRS
  Serial.println(F("[G] Enable GPRS"));
  Serial.println(F("[g] Disable GPRS"));
  Serial.println(F("[l] Query GSMLOC (GPRS)"));


  // GPS
  if ((type == FONA3G_A) || (type == FONA3G_E) || (type == FONA808_V1) || (type == FONA808_V2)) {
    Serial.println(F("[O] Turn GPS on (FONA 808 & 3G)"));
    Serial.println(F("[o] Turn GPS off (FONA 808 & 3G)"));
    Serial.println(F("[L] Query GPS location (FONA 808 & 3G)"));
    if (type == FONA808_V1) {
      Serial.println(F("[x] GPS fix status (FONA808 v1 only)"));
    }
    Serial.println(F("[E] Raw NMEA out (FONA808)"));
  }
  
}

char fonaInBuffer[64];          //for notifications from the FONA

void sendResponse(const char* response)
{
  Serial.println(response);
  fona.sendSMS(callerIDbuffer, response);
}

void processCommand(char command) {
  Serial.println(command);
  char responseStr[256];
  switch (command) {
    case '?': {
        printMenu();
        break;
      }
    case '1': {
        //TODO: make this into a function call
        //unlock the lock for 5 seconds.
        digitalWrite(LOCK, HIGH); // turn the LOCK on (HIGH is the voltage level)
        digitalWrite(LED,  HIGH); // turn the LED on (HIGH is the voltage level)
        delay(5000);              // wait for a second
        digitalWrite(LOCK, LOW);  // turn the LOCK off by making the voltage LOW
        digitalWrite(LED,  LOW);  // turn the LED off by making the voltage LOW
        sendResponse("unlocking box");
        break;
      }
    case 'a': {
        // read the ADC
        uint16_t adc;
        if (! fona.getADCVoltage(&adc)) {
          sendResponse("Failed to read ADC");
        } else {
          sprintf(responseStr, "ADC = %d mV", adc); // puts string into buffer
          sendResponse(responseStr);
        }
        break;
      }

    case 'b': {
        // read the battery voltage and percentage
        uint16_t vbat;
        if (! fona.getBattVoltage(&vbat)) {
          sendResponse("Failed to read Batt");
        } else {
          sprintf(responseStr, "VBat = %d mV", vbat); // puts string into buffer
          sendResponse(responseStr);
        }

        if (! fona.getBattPercent(&vbat)) {
          sendResponse("Failed to read Batt");
        } else {
          sprintf(responseStr, "VPct = %d %%", vbat); // puts string into buffer
          sendResponse(responseStr);
        }

        break;
      }

    case 'C': {
        // read the CCID
        fona.getSIMCCID(replyBuffer);  // make sure replybuffer is at least 21 bytes!
        sprintf(responseStr, "SIM CCID = %s", replyBuffer);
        sendResponse(responseStr);
        break;
      }

    case 'i': {
        // read the RSSI
        uint8_t n = fona.getRSSI();
        int8_t r;
        if (n == 0) r = -115;
        if (n == 1) r = -111;
        if (n == 31) r = -52;
        if ((n >= 2) && (n <= 30)) {
          r = map(n, 2, 30, -110, -54);
        }
        sprintf(responseStr, "RSSI = %d: %d dBm", n, r);
        sendResponse(responseStr);
        break;
      }

    case 'n': {
        // read the network/cellular status
        uint8_t n = fona.getNetworkStatus();
        char* statusStr = "";
        if (n == 0) statusStr = "Not registered";
        if (n == 1) statusStr = "Registered (home)";
        if (n == 2) statusStr = "Not registered (searching)";
        if (n == 3) statusStr = "Denied";
        if (n == 4) statusStr = "Unknown";
        if (n == 5) statusStr = "Registered roaming";

        sprintf(responseStr, "Network status %d: %s", n, statusStr);
        sendResponse(responseStr);
        break;
      }
    
    /*** SMS ***/
    case 'N': {
        // read the number of SMS's!
        int8_t smsnum = fona.getNumSMS();
        if (smsnum < 0) {
          sendResponse("Could not read # SMS");
        } else {
          sprintf(responseStr, "%d SMS's on SIM card!", smsnum);
          sendResponse(responseStr);
        }
        break;
      }
    case 'R': {
        // read all SMS
        int8_t smsnum = fona.getNumSMS();
        uint16_t smslen;
        int8_t smsn;

        if ( (type == FONA3G_A) || (type == FONA3G_E) ) {
          smsn = 0; // zero indexed
          smsnum--;
        } else {
          smsn = 1;  // 1 indexed
        }

        for ( ; smsn <= smsnum; smsn++) {
          if (!fona.readSMS(smsn, replyBuffer, 250, &smslen)) {  // pass in buffer and max len!
            sendResponse("Failed!");
            break;
          }
          // if the length is zero, its a special case where the index number is higher
          // so increase the max we'll look at!
          if (smslen == 0) {
            sendResponse("[empty slot]");
            smsnum++;
            continue;
          }
          sprintf(responseStr, "***SMS #%d (%d) bytes *** %s ***", smsn, smslen, replyBuffer);
          sendResponse(responseStr);
        }
        break;
      }

    /*** Time ***/

    case 'y': {
        // enable network time sync
        if (!fona.enableNetworkTimeSync(true))
          sendResponse("Failed to enable");
        break;
      }

    case 'Y': {
        // enable NTP time sync
        if (!fona.enableNTPTimeSync(true, F("pool.ntp.org")))
          sendResponse("Failed to enable");
        break;
      }

    case 't': {
        // read the time
        char buffer[23];

        fona.getTime(buffer, 23);  // make sure replybuffer is at least 23 bytes!
        sprintf(responseStr, "Time = %s", buffer);
        sendResponse(responseStr);
        break;
      }


    /*********************************** GPS (SIM808 only) */

    case 'o': {
        // turn GPS off
        if (!fona.enableGPS(false))
          sendResponse("Failed to turn off");
        break;
      }
    case 'O': {
        // turn GPS on
        if (!fona.enableGPS(true))
          sendResponse("Failed to turn on");
        break;
      }
    case 'x': {
        int8_t stat;
        // check GPS fix
        stat = fona.GPSstatus();
        char* statString = "";
        if (stat < 0)  statString = "Failed to query";
        if (stat == 0) statString = "GPS off";
        if (stat == 1) statString = "No fix";
        if (stat == 2) statString = "2D fix";
        if (stat == 3) statString = "3D fix";

        sendResponse(statString);
        break;
      }

    case 'L': {
        // check for GPS location
        char gpsdata[120];
        fona.getGPS(0, gpsdata, 120);
        if (type == FONA808_V1)
          sprintf(responseStr, "Reply in format: mode,longitude,latitude,altitude,utctime(yyyymmddHHMMSS),ttff,satellites,speed,course: %s", gpsdata);
        else 
          sprintf(responseStr, "Reply in format: mode,fixstatus,utctime(yyyymmddHHMMSS),latitude,longitude,altitude,speed,course,fixmode,reserved1,HDOP,PDOP,VDOP,reserved2,view_satellites,used_satellites,reserved3,C/N0max,HPA,VPA: %s", gpsdata);
        sendResponse(responseStr);

        break;
      }

    /*********************************** GPRS */

    case 'g': {
        // turn GPRS off
        if (!fona.enableGPRS(false))
          sendResponse("Failed to turn off");
        break;
      }
    case 'G': {
        // turn GPRS on
        if (!fona.enableGPRS(true))
          sendResponse("Failed to turn on");
        break;
      }
    case 'l': {
        // check for GSMLOC (requires GPRS)
        uint16_t returncode;

        if (!fona.getGSMLoc(&returncode, replyBuffer, 250))
          sendResponse("Failed!");
        if (returncode == 0) {
          sendResponse(replyBuffer);
        } else {
          sprintf(responseStr, "Fail code #%d", returncode);
          sendResponse(responseStr);
        }

        break;
      }

    default: {
        sendResponse("Unknown command");
        printMenu();
        break;
      }
  }
}

void loop() {
  
  char* bufPtr = fonaInBuffer;    //handy buffer pointer

  if (fona.available())      //any data available from the FONA?
  {
    int slot = 0;            //this will be the slot number of the SMS
    int charCount = 0;
    //Read the notification into fonaInBuffer
    do  {
      *bufPtr = fona.read();
      Serial.write(*bufPtr);
      delay(1);
    } while ((*bufPtr++ != '\n') && (fona.available()) && (++charCount < (sizeof(fonaInBuffer)-1)));
    
    //Add a terminal NULL to the notification string
    *bufPtr = 0;
    
    //Scan the notification string for an SMS received notification.
    //  If it's an SMS message, we'll get the slot number in 'slot'
    if (1 == sscanf(fonaInBuffer, "+CMTI: \"SM\",%d", &slot)) {
      Serial.print("slot: "); Serial.println(slot);
      
      if (! fona.getSMSSender(slot, callerIDbuffer, 31)) {
        Serial.println("Didn't find SMS message in slot!");
      }
      Serial.print(F("FROM: ")); Serial.println(callerIDbuffer);

      // Retrieve SMS value.
      uint16_t smslen;
      if (! fona.readSMS(slot, replyBuffer, 250, &smslen)) { // pass in buffer and max len!
        Serial.println("Failed to read sms!\r\n");
      }
      //TODO: Evaluate removing this response, it is an added SMS, we should be efficient
      //Send back an automatic response
      Serial.println("Sending reponse...");
      if (!fona.sendSMS(callerIDbuffer, "Processing command")) {
        Serial.println(F("Failed"));
      } else {
        Serial.println(F("Sent!"));
      }
      //TODO: Add Security before sending to processCommand()
      processCommand(replyBuffer[0]);
      
      // delete the original msg after it is processed
      // otherwise, we will fill up all the slots
      // and then we won't be able to receive SMS anymore
      
      if (fona.deleteSMS(slot)) {
        Serial.println(F("OK!"));
      } else {
        Serial.println(F("Couldn't delete"));
      }

      wipeAllSmsSlots();
      
    }
  } 
}

