#include "Adafruit_FONA.h"
#define ADMIN_PHONE "410XXXXXXX"
#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_RI  7

#define LOCK 12
#define LED  13

// this is a large buffer for replies
char replybuffer[255];
char callerIDbuffer[32];  //we'll store the SMS sender number in here
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

uint8_t type;

void flushSerial() {
  while (Serial.available())
    Serial.read();
}

char readBlocking() {
  while (!Serial.available());
  return Serial.read();
}

uint16_t readnumber() {
  uint16_t x = 0;
  char c;
  while (! isdigit(c = readBlocking())) {
    //Serial.print(c);
  }
  Serial.print(c);
  x = c - '0';
  while (isdigit(c = readBlocking())) {
    Serial.print(c);
    x *= 10;
    x += c - '0';
  }
  return x;
}

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;

  while (true) {
    if (buffidx > maxbuff) {
      //Serial.println(F("SPACE"));
      break;
    }

    while (Serial.available()) {
      char c =  Serial.read();

      //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);

      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)   // the first 0x0A is ignored
          continue;

        timeout = 0;         // the second 0x0A is the end of the line
        timeoutvalid = true;
        break;
      }
      buff[buffidx] = c;
      buffidx++;
    }

    if (timeoutvalid && timeout == 0) {
      //Serial.println(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;  // null term
  return buffidx;
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
      if (!fona.sendSMS(ADMIN_PHONE, "Fona Ready")) {
        Serial.println(F("Status Message Failed"));
        //TODO: Blink error code
      } else {
        Serial.println(F("Status Message Sent"));
      }
      break;
    }
    delay(1000);
    
    //TODO: change this to a function call, non-blocking (without delays)
    for (int i=0; i <= 10; i++){
      digitalWrite(LED,  HIGH);
      delay(100);
      digitalWrite(LED,  LOW);
      delay(100);
    }
    for (int i=0; i <= 1; i++){
      digitalWrite(LOCK, HIGH);
      delay(1000);
      digitalWrite(LOCK, LOW);
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
  Serial.println(F("[U] Unlock SIM with PIN code"));
  Serial.println(F("[i] read RSSI"));
  Serial.println(F("[n] get Network status"));

  // SMS
  Serial.println(F("[N] Number of SMSs"));
  Serial.println(F("[r] Read SMS #"));
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
  
  Serial.println(F("[S] create Serial passthru tunnel"));
  Serial.println(F("-------------------------------------"));
  Serial.println(F(""));

}

char fonaInBuffer[64];          //for notifications from the FONA

void SendResponse(const char* response)
{
  Serial.println(response);
  fona.sendSMS(callerIDbuffer, response);
}

void processCommand(char command) {
  Serial.println(command);
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

        break;
      }
    case 'a': {
        // read the ADC
        uint16_t adc;
        if (! fona.getADCVoltage(&adc)) {
          SendResponse("Failed to read ADC");
        } else {
          char str[32];
          sprintf(str, "ADC = %d mV", adc); // puts string into buffer
          SendResponse(str);
        }
        break;
      }

    case 'b': {
        // read the battery voltage and percentage
        uint16_t vbat;
        if (! fona.getBattVoltage(&vbat)) {
          SendResponse("Failed to read Batt");
        } else {
          char str[32];
          sprintf(str, "VBat = %d mV", vbat); // puts string into buffer
          SendResponse(str);
        }

        if (! fona.getBattPercent(&vbat)) {
          SendResponse("Failed to read Batt");
        } else {
          char str[32];
          sprintf(str, "VPct = %d %%", vbat); // puts string into buffer
          SendResponse(str);
        }

        break;
      }

    case 'U': {
        // Unlock the SIM with a PIN code
        char PIN[5];
        flushSerial();
        Serial.println(F("Enter 4-digit PIN"));
        readline(PIN, 3);
        Serial.println(PIN);
        Serial.print(F("Unlocking SIM card: "));
        if (! fona.unlockSIM(PIN)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
        break;
      }

    case 'C': {
        // read the CCID
        fona.getSIMCCID(replybuffer);  // make sure replybuffer is at least 21 bytes!
        Serial.print(F("SIM CCID = ")); Serial.println(replybuffer);
        break;
      }

    case 'i': {
        // read the RSSI
        uint8_t n = fona.getRSSI();
        int8_t r;

        Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(": ");
        if (n == 0) r = -115;
        if (n == 1) r = -111;
        if (n == 31) r = -52;
        if ((n >= 2) && (n <= 30)) {
          r = map(n, 2, 30, -110, -54);
        }
        Serial.print(r); Serial.println(F(" dBm"));

        break;
      }

    case 'n': {
        // read the network/cellular status
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
        break;
      }
    
    /*** SMS ***/
    case 'N': {
        // read the number of SMS's!
        int8_t smsnum = fona.getNumSMS();
        if (smsnum < 0) {
          Serial.println(F("Could not read # SMS"));
        } else {
          
          Serial.print(smsnum);
          fona.sendSMS(callerIDbuffer, " SMS's on SIM card!");
          Serial.println(F(" SMS's on SIM card!"));
        }
        break;
      }
    case 'r': {
        // read an SMS
        flushSerial();
        Serial.print(F("Read #"));
        uint8_t smsn = readnumber();
        Serial.print(F("\n\rReading SMS #")); Serial.println(smsn);

        // Retrieve SMS sender address/phone number.
        if (! fona.getSMSSender(smsn, replybuffer, 250)) {
          Serial.println("Failed!");
          break;
        }
        Serial.print(F("FROM: ")); Serial.println(replybuffer);

        // Retrieve SMS value.
        uint16_t smslen;
        if (! fona.readSMS(smsn, replybuffer, 250, &smslen)) { // pass in buffer and max len!
          Serial.println("Failed!");
          break;
        }
        Serial.print(F("***** SMS #")); Serial.print(smsn);
        Serial.print(" ("); Serial.print(smslen); Serial.println(F(") bytes *****"));
        Serial.println(replybuffer);
        Serial.println(F("*****"));

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
          Serial.print(F("\n\rReading SMS #")); Serial.println(smsn);
          if (!fona.readSMS(smsn, replybuffer, 250, &smslen)) {  // pass in buffer and max len!
            Serial.println(F("Failed!"));
            break;
          }
          // if the length is zero, its a special case where the index number is higher
          // so increase the max we'll look at!
          if (smslen == 0) {
            Serial.println(F("[empty slot]"));
            smsnum++;
            continue;
          }

          Serial.print(F("***** SMS #")); Serial.print(smsn);
          Serial.print(" ("); Serial.print(smslen); Serial.println(F(") bytes *****"));
          Serial.println(replybuffer);
          Serial.println(F("*****"));
        }
        break;
      }

    /*** Time ***/

    case 'y': {
        // enable network time sync
        if (!fona.enableNetworkTimeSync(true))
          Serial.println(F("Failed to enable"));
        break;
      }

    case 'Y': {
        // enable NTP time sync
        if (!fona.enableNTPTimeSync(true, F("pool.ntp.org")))
          Serial.println(F("Failed to enable"));
        break;
      }

    case 't': {
        // read the time
        char buffer[23];

        fona.getTime(buffer, 23);  // make sure replybuffer is at least 23 bytes!
        Serial.print(F("Time = ")); Serial.println(buffer);
        break;
      }


    /*********************************** GPS (SIM808 only) */

    case 'o': {
        // turn GPS off
        if (!fona.enableGPS(false))
          Serial.println(F("Failed to turn off"));
        break;
      }
    case 'O': {
        // turn GPS on
        if (!fona.enableGPS(true))
          Serial.println(F("Failed to turn on"));
        break;
      }
    case 'x': {
        int8_t stat;
        // check GPS fix
        stat = fona.GPSstatus();
        if (stat < 0)
          Serial.println(F("Failed to query"));
        if (stat == 0) Serial.println(F("GPS off"));
        if (stat == 1) Serial.println(F("No fix"));
        if (stat == 2) Serial.println(F("2D fix"));
        if (stat == 3) Serial.println(F("3D fix"));
        break;
      }

    case 'L': {
        // check for GPS location
        char gpsdata[120];
        fona.getGPS(0, gpsdata, 120);
        if (type == FONA808_V1)
          Serial.println(F("Reply in format: mode,longitude,latitude,altitude,utctime(yyyymmddHHMMSS),ttff,satellites,speed,course"));
        else 
          Serial.println(F("Reply in format: mode,fixstatus,utctime(yyyymmddHHMMSS),latitude,longitude,altitude,speed,course,fixmode,reserved1,HDOP,PDOP,VDOP,reserved2,view_satellites,used_satellites,reserved3,C/N0max,HPA,VPA"));
        Serial.println(gpsdata);

        break;
      }

    /*********************************** GPRS */

    case 'g': {
        // turn GPRS off
        if (!fona.enableGPRS(false))
          Serial.println(F("Failed to turn off"));
        break;
      }
    case 'G': {
        // turn GPRS on
        if (!fona.enableGPRS(true))
          Serial.println(F("Failed to turn on"));
        break;
      }
    case 'l': {
        // check for GSMLOC (requires GPRS)
        uint16_t returncode;

        if (!fona.getGSMLoc(&returncode, replybuffer, 250))
          Serial.println(F("Failed!"));
        if (returncode == 0) {
          Serial.println(replybuffer);
        } else {
          Serial.print(F("Fail code #")); Serial.println(returncode);
        }

        break;
      }

    case 'W': {
        // Post data to website
        uint16_t statuscode;
        int16_t length;
        char url[80];
        char data[80];

        flushSerial();
        Serial.println(F("NOTE: in beta! Use simple websites to post!"));
        Serial.println(F("URL to post (e.g. httpbin.org/post):"));
        Serial.print(F("http://")); readline(url, 79);
        Serial.println(url);
        Serial.println(F("Data to post (e.g. \"foo\" or \"{\"simple\":\"json\"}\"):"));
        readline(data, 79);
        Serial.println(data);

        Serial.println(F("****"));
        if (!fona.HTTP_POST_start(url, F("text/plain"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length)) {
          Serial.println("Failed!");
          break;
        }
        while (length > 0) {
          while (fona.available()) {
            char c = fona.read();

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
            UDR0 = c;
#else
            Serial.write(c);
#endif

            length--;
            if (! length) break;
          }
        }
        Serial.println(F("\n****"));
        fona.HTTP_POST_end();
        break;
      }
    /*****************************************/

    case 'S': {
        Serial.println(F("Creating SERIAL TUBE"));
        while (1) {
          while (Serial.available()) {
            delay(1);
            fona.write(Serial.read());
          }
          if (fona.available()) {
            Serial.write(fona.read());
          }
        }
        break;
      }

    default: {
        Serial.println(F("Unknown command"));
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
      if (! fona.readSMS(slot, replybuffer, 250, &smslen)) { // pass in buffer and max len!
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
      processCommand(replybuffer[0]);
      
      // delete the original msg after it is processed
      //   otherwise, we will fill up all the slots
      //   and then we won't be able to receive SMS anymore
      if (fona.deleteSMS(slot)) {
        Serial.println(F("OK!"));
      } else {
        Serial.println(F("Couldn't delete"));
      }
    }
  } 
}

