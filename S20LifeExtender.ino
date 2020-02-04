/*
 * Copyright (C) 2020 by Chris Byrneham
 * This software was developed independently and is not supported or endorsed in any way by Orvibo (c).
 * 
This sketch synchronises the time on Orvibo S20 switches.
Orvibo the suppliers of the switches stopped the server that was called to get the time.
This arduino sketch on an ESP8266 effectively keeps the switches working for the cost of an ESP8266 ( A few pounds ). 

I wanted a bare ESP8266 that could be programmed and ready to go, just power it on and run.

You can still use the Wiwo Android App to set the program times on the switches.

Running Apache + PHP on a server, even on a Raspbery Pi seems overkill. 
You need a browser permanently on the page to refresh it, and manually check the warning status markers.
But see; 



You could add a switch on a data input pin for Daylight Saving Time, or just have two ESP8266 units for summer and winter.

To get core ESP8266 libraries for Arduino IDE 
IDE File Preferences 'Additional Board Manager Urls' add;
http://arduino.esp8266.com/stable/package_esp8266com_index.json

Then go to Tools, Board,Board Manager and install ESP8266 support.

Libraries needed;
Timemark Copyright (C) 2016, Mikael Patel https://github.com/mikaelpatel/Arduino-Timemark
ESP8266WebServer Copyright (c) 2014 Ivan Grokhotkov.
ESP8266WiFi - esp8266 Wifi support. Based on WiFi.h from Arduino WiFi shield library. Copyright (c) 2011-2014 Arduino. Modified by Ivan Grokhotkov, December 2014
ESPAsyncUDP https://github.com/me-no-dev/ESPAsyncUDP
EEPROM.cpp - esp8266 EEPROM emulation  Copyright (c) 2014 Ivan Grokhotkov. 


There are four ways of setting the necessary SSID, Password, and Timezone offset parameters;

1) Hard code them
2) Enter by using the arduino IDE serial monitor
3) Enter by using a standalone access point TEMPSSID ( no password ) on a web server on http://192.168.1.100
4) I can supply ESP8266s programmed and ready to go for £10.00 plus postage.

Paypal Contributions appreciated  chris@byrneham.com

References and thanks to;

Fernando M Silva  fcr@netcabo.pt , A PHP web app for S20s 
Andrius A Tikonas , for initial work on reverse engineering the protocol.
Plus all library developers above.
 
Building; Sketch, Upload;
You can ignore the warning;
WARNING: library Timemark claims to run on (avr) architecture(s) and may be incompatible with your current board which runs on (esp8266) architecture(s).
 
*/
#include <Arduino.h>

#include <Timemark.h>

#include <EEPROM.h>

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "ESPAsyncUDP.h"

// Max serial data we expect before a newline
const unsigned int MAX_INPUT = 50;
AsyncUDP udp;
AsyncUDP ntpUdp;

bool ssidEntered = false;
bool passEntered = false;
bool offsetEntered = false;
bool credentialsEntered = false;
// Fill the following three values if you don't want to enter via serial monitor or WiFi access point and web page
String hardCodedSsid = "";
String hardCodedPassword = "";
String hardCodedUtcOffset = "";  // Use values like 0.0 for UTC or 6.5 for Myanmar Time see https://www.timeanddate.com/time/zones/

String ssid;
String password;
String utcOffset;
// Temporary standalone access point SSID
const char * accessPoint = "TEMPSSID";

const int buflen = 64;
uint32_t Global_NTPTime = 0;
const float oneHourInSeconds = 3600.0; // an hour in seconds
IPAddress timeServerIP;          
// NTP server address
const char * NTPServerName = "pool.ntp.org";
const int NTP_PACKET_SIZE = 48;  // NTP time stamp is in the first 48 bytes of the message
byte NTPBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets
int delayTimeCompensation = 0;
int cycleCount = 0;
Timemark phaseOneTimer(45 * 1000); // 45 seconds to enter serial ssid,pass,offset
bool phaseOneState = false;
Timemark phaseTwoTimer(120 * 1000); // 120 seconds to run access point and enter web server data
bool phaseTwoState = false;
Timemark refreshTimer( 60 * 57 * 1000 ); //  29 minutes between S20 time refreshes
bool phaseThreeState = false;
bool phaseThreeRunOnce = false;
bool phaseFourState = false;
bool phaseFiveState = false;
bool phaseSixState = false;

bool eepromRead() {
  unsigned char buf[buflen];
  int idx = 0;
  unsigned char checksum = 0;
  for (idx = 0; idx < buflen; idx++ ) {
    buf[idx] = EEPROM.read(idx);
    checksum += buf[idx];
  }
  ssid = (const char *)buf;
  for (idx = 0; idx < buflen; idx++ ) {
    buf[idx] = EEPROM.read(buflen + idx);
    checksum += buf[idx];
  }
  password = (const char *)buf;
  for (idx = 0; idx < buflen; idx++ ) {
    buf[idx] = EEPROM.read(buflen*2 + idx);
    checksum += buf[idx];
  }
  utcOffset = (const char *)buf;
  if ( checksum != EEPROM.read(buflen * 3) ) {
    Serial.println("EEPROM checksum error");
    return false;
  }
  return true;
}

bool eepromSave() {
  unsigned char buf[buflen];
  int idx = 0;
  unsigned char checksum = 0;
  ssid.getBytes(buf, buflen, 0);
  for (idx = 0; idx < buflen; idx++ ) {
    checksum += buf[idx];
    EEPROM.write(idx, buf[idx]);
  }
  password.getBytes(buf, buflen, 0);
  for (idx = 0; idx < buflen; idx++ ) {
    checksum += buf[idx];
    EEPROM.write(buflen + idx, buf[idx]);
  }
  utcOffset.getBytes(buf, buflen, 0);
  for (idx = 0; idx < buflen; idx++ ) {
    checksum += buf[idx];
    EEPROM.write(buflen*2 + idx, buf[idx]);
  }
  EEPROM.write(buflen * 3, checksum );
  if (EEPROM.commit()) {
    Serial.println("EEPROM successfully committed");
    return true;
  } else {
    Serial.println("ERROR! EEPROM commit failed");
    return false;
  }
}

void tryHardCodedCredentials() {
  if ( hardCodedSsid.length() > 0 && hardCodedPassword.length() > 0 && hardCodedUtcOffset.length() > 0) {
    ssid = hardCodedSsid;
    password = hardCodedPassword;
    utcOffset = hardCodedUtcOffset;
    Serial.println("Saving Hardcoded Credentials");
    eepromSave();
    credentialsEntered = true;
    return;
  }
  Serial.println("Fatal! No Hardcoded Credentials Found! Reset and enter credentials using serial monitor or web page");
}

void setup () {
  Serial.begin (115200);
  delay(500);
  EEPROM.begin(512);
  Serial.println ();
  Serial.println ("Starting...");

  if ( eepromRead() ) {
    Serial.println ("Using eeprom values");
    credentialsEntered = true;
  }

  Serial.println ("Enter WiFi Network SSID");
  phaseOneTimer.start();
  phaseOneState = true;

} // end of setup

// here to process incoming serial data after a terminator received
void process_data (const char * data)
{
  unsigned char buf[buflen];
  // for now just display it
  // (but you could compare it to some value, convert to an integer, etc.)
  if ( !ssidEntered ) {
    ssid = data;
    ssid.getBytes(buf, buflen, 0);
    Serial.printf ("SSID: %s\n", &buf);
    Serial.println ("Enter wifi network password");
    ssidEntered = true;
  } else {
    if ( !passEntered ) {
      password = data;
      password.getBytes(buf, buflen, 0);
      Serial.printf ("Password: %s\n", &buf);
      passEntered = true;
    } else {
      if ( !offsetEntered ) {
        utcOffset = data;
        utcOffset.getBytes(buf, buflen, 0);
        Serial.printf ("TZ Offset: %s\n", &buf);
        offsetEntered = true;
        credentialsEntered = true;
        eepromSave();
      }
    }
  }
}  // end of process_data

void processIncomingByte (const byte inByte)
{
  static char input_line [MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte)
  {

    case '\n':   // end of text
      input_line [input_pos] = 0;  // terminating null byte

      // terminator reached! process input_line here ...
      process_data (input_line);

      // reset buffer for next time
      input_pos = 0;
      break;

    case '\r':   // discard carriage return
      break;

    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_INPUT - 1))
        input_line [input_pos++] = inByte;
      break;

  }  // end of switch

} // end of processIncomingByte


ESP8266WebServer server(80);
bool serverRunning;

void handleRoot() {
  unsigned char buf[buflen];

  server.arg("ssid").getBytes(buf, buflen, 0);
  ssid = (const char *)buf;
  server.arg("pass").getBytes(buf, buflen, 0);
  password = (const char *)buf;
  server.arg("offset").getBytes(buf, buflen, 0);
  utcOffset = (const char *)buf;

  Serial.print( "SSID: " );
  Serial.print( ssid );
  Serial.print( " PASSWORD: " );
  Serial.print( password );
  Serial.print( " UTC TZ OFFSET: " );
  Serial.println( utcOffset );

  char html[1000];
  snprintf ( html, 1000,
             "<html><head><title>ESP8266 WiFi Network</title></head>\
    <body><h1>ESP8266 Wi-Fi Access Point</h1>\
    <form action=\"/\" >\
    SSID:<br><input type=\"text\" name=\"ssid\" ><br>\
    Password:<br><input type=\"text\" name=\"pass\">\
    TZ Offset:<br><input type=\"text\" name=\"offset\"><br>\
    <input type=\"submit\" value=\"Submit\">\
    </form>\
    </body>\
    </html>" );
  server.send ( 200, "text/html", html );
}


uint8_t queryall[] = { 0x68, 0x64,
                       0x00, 0x06,
                       0x71, 0x61
                     };

uint8_t queryallResponse[] = { 0x68, 0x64,
                               0x00, 0x2A,
                               0x71, 0x61
                             };
uint8_t subscribe[] = {
  0x68, 0x64,
  0x00, 0x1E,
  0x63, 0x6C, // cl
  // [6] mac
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0,  // AC CF 23 24 19 C0 Mac Address (Max Length = 12 = 24bytes)
  0x20, 0x20, 0x20, 0x20, 0x20, 0x20, // Mac Address Padding (spaces)
  // [18] rmac
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0,  // RMac Address (Max Length = 12 = 24bytes)
  0x20, 0x20, 0x20, 0x20, 0x20, 0x20, // Reverse Mac Address Padding (spaces)
};

uint8_t subscribeResponse[] = {
  0x68, 0x64,
  0x00, 0x18,
  0x63, 0x6C // cl etc ...
};

void rmac( unsigned char * mac, unsigned char * rmac ) {
  for ( int i = 0; i < 6; i++ ) {
    rmac[5 - i] = mac[i];
  }
}


uint8_t countdownSet[] = {
  0x68, 0x64,
  0x00, 0x1a,
  0x63, 0x73, // cs
  // [6] mac
  0x99, 0x99, 0x99, 0x99, 0x99, 0x99,
  0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
  0x00, 0x00, 0x00, 0x00,
  // [22] Time
  0x00, 0x00, 0x00, 0x00
};

uint8_t countdownSetResponse[] = {
  0x68, 0x64,
  0x00, 0x17,
  0x63, 0x73 // cs etc ...
};

void setCountdownSetTime( unsigned int time ) {
  countdownSet[25] = (byte)((time & 0xFF000000) >> 24) ;
  countdownSet[24] = (byte)((time & 0xFF0000) >> 16) ;
  countdownSet[23] = (byte)((time & 0xFF00) >> 8) ;
  countdownSet[22] = (byte)(time & 0xFF) ;
}


void hexdump(unsigned char * buf, int len) {
  for ( int i = 0; i < len; i++ ) {
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}


struct S20Socket {
  unsigned char mac[6];
  unsigned char ip[4];
};

const int MAX_S20_SOCKETS = 25;


class KnownSockets {
  private:

    int socketIndex = -1;
    S20Socket s20Sockets[MAX_S20_SOCKETS];

  public:
  
    //void clearSockets() {
    //  for (int idx = 0; idx < MAX_S20_SOCKETS; idx++) {
    //    memset(s20Sockets[idx].mac, 0, 6);
    //  }
    //  socketIndex = -1;
    //}

    int findByMacAddress(unsigned char * mac) {
      for ( int i = 0; i <= socketIndex; i++ ) {
        if ( memcmp(s20Sockets[i].mac, mac, 6 ) == 0 ) {
          return i;
        }
      }
      return -1;
    }

    void add( AsyncUDPPacket packet ) {
      int found = findByMacAddress(packet.data() + 7);
      if ( found < 0 ) {
        // New
        Serial.println("New Socket");
        memcpy(s20Sockets[++socketIndex].mac, packet.data() + 7, 6);
        memcpy(s20Sockets[socketIndex].ip, (ip_addr_t *)(packet.remoteIP()), 4);
      } else {
        // Known, just update ip
        Serial.println("Known Socket");
        memcpy(s20Sockets[found].ip, (ip_addr_t *)(packet.remoteIP()), 4);
      }
    }

    int size() {
      return socketIndex + 1;
    }

    // Zero based index
    S20Socket * getSocket( int idx ) {
      if ( idx < 0 || idx > socketIndex ) {
        return NULL;
      }
      return &s20Sockets[idx];
    }


};

KnownSockets knownSockets;
int ntpSendCount = 0;
void loop() {
  if ( phaseOneState ) {
    if ( !phaseOneTimer.expired() ) {
      // If serial data available, process it
      while (Serial.available () > 0) {
        processIncomingByte (Serial.read ());
      }
    } else {
      phaseOneTimer.stop();
      phaseOneState = false;
      Serial.println ("Giving up trying to get network details from serial monitor. Now trying to open WiFi access point");
      phaseTwoTimer.start();
      phaseTwoState = true;
      Serial.print("Setting AP (Access Point)…");
      // Remove the password parameter, if you want the AP (Access Point) to be open
      IPAddress local_ip = IPAddress(192, 168, 1, 100); // The http server is available on this address
      IPAddress gateway = IPAddress(192, 168, 1, 1);
      IPAddress subnet = IPAddress(255, 255, 255, 0);
      WiFi.softAPConfig( local_ip,  gateway,  subnet);
      WiFi.softAP(accessPoint);
      IPAddress IP = WiFi.softAPIP();
      Serial.print("AP IP address: ");
      Serial.println(IP);
      // Print ESP8266 Local IP Address
      // Serial.println(WiFi.localIP());
      server.on ( "/", handleRoot );
      server.begin();
      serverRunning = true;
      Serial.println("HTTP server started");
    }
  } // Phase One

  if ( phaseTwoState ) {
    if ( phaseTwoTimer.expired() ) {
      phaseTwoTimer.stop();
      phaseTwoState = false;
      Serial.println("Access Point Stopped");
      WiFi.disconnect();
      serverRunning = false;
      server.stop();
      Serial.println("HTTP server stopped");

      if (!credentialsEntered) {
        tryHardCodedCredentials();
      }
      // Now start the WiFi
      phaseThreeState = true;
      phaseThreeRunOnce = true;
      WiFi.mode(WIFI_STA);
      Serial.println("Joining WiFi network...");
      WiFi.begin(ssid, password);
      if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("WiFi Failed");
        Serial.flush();
        delay(5000);
        ESP.reset();
      }
      if (udp.listenMulticast(IPAddress(224, 0, 0, 251), 10000)) {
        Serial.print("UDP Listening on IP: ");
        Serial.println(WiFi.localIP());
        udp.onPacket([](AsyncUDPPacket packet) {
          Serial.print("UDP Packet Type: ");
          Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
          Serial.print(", From: ");
          Serial.print(packet.remoteIP());
          Serial.print(":");
          Serial.print(packet.remotePort());
          Serial.print(", To: ");
          Serial.print(packet.localIP());
          Serial.print(":");
          Serial.print(packet.localPort());
          Serial.print(", Length: ");
          Serial.print(packet.length());
          Serial.print(", Data: ");
          Serial.write(packet.data(), packet.length());
          Serial.println();
          packet.printf("Got %u bytes of data", packet.length());
          // Build a list of known sockets, we just need their Mac and Ip addresses
          if ( memcmp( packet.data(), queryallResponse , 6 ) == 0 ) {
            Serial.println("Incoming QueryAll Response");
            knownSockets.add(packet);
          } else if ( memcmp( packet.data(), subscribeResponse , 6 ) == 0 ) {
            Serial.println("Ignoring Incoming Subscribe Response");
          } else if ( memcmp( packet.data(), countdownSetResponse , 6 ) == 0 ) {
            Serial.println("Ignoring Incoming CountdownSet Response");
          } else {
            Serial.println("Unknown Incoming Response");
          }
        });
      }
    } else {
      server.handleClient();
    }
  } // Phase two

  if ( phaseThreeState ) {
    if ( phaseThreeRunOnce ) {
      phaseThreeRunOnce = false;
      // send QueryAll etc
      Serial.println("Sending QueryAll");
      AsyncUDPMessage msg;
      msg.write(queryall, 6);
      udp.broadcast(msg);
    }
    // Wait a while... Are there any sockets found ( todo try again ? )
    delay(5000);
    if ( knownSockets.size() > 0 ) {
      Serial.print("Found ");
      Serial.print(knownSockets.size());
      Serial.println(" Sockets" );
      phaseThreeState = false;
      phaseFourState = true;
    } else {
      Serial.println("No Sockets found!");
      // Set up for phase Three again

    }
  } // Phase three

  if ( phaseFourState ) {
    // Get NTP time see https://tttapa.github.io/ESP8266/Chap15%20-%20NTP.html

    if (!WiFi.hostByName(NTPServerName, timeServerIP)) { // Get the IP address of the NTP server
      Serial.println("DNS lookup failed.");
      // Serial.flush();
      // ESP.reset();
    }
    Serial.print("Using Time server IP:\t");
    Serial.println(timeServerIP);

    if (ntpUdp.listen(123)) {
      Serial.print("NTP UDP Listening on IP: ");
      Serial.println(WiFi.localIP());
      ntpUdp.onPacket([](AsyncUDPPacket packet) {
        Serial.print("NTP UDP Packet Type: ");
        Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
        Serial.print(", From: ");
        Serial.print(packet.remoteIP());
        Serial.print(":");
        Serial.print(packet.remotePort());
        Serial.print(", To: ");
        Serial.print(packet.localIP());
        Serial.print(":");
        Serial.print(packet.localPort());
        Serial.print(", Length: ");
        Serial.print(packet.length());
        Serial.print(", Data: ");
        Serial.write(packet.data(), packet.length());
        Serial.println();

        packet.printf("NTP Got %u bytes of data", packet.length());

        // Combine the 4 timestamp bytes into one 32-bit number
        Global_NTPTime = (packet.data()[40] << 24) | (packet.data()[41] << 16) | (packet.data()[42] << 8) | packet.data()[43];
        // Convert NTP time to a UNIX timestamp:
        // Unix time starts on Jan 1 1970. That's 2208988800 seconds in NTP time:
        // const uint32_t seventyYears = 2208988800UL;
        // subtract seventy years:
        // uint32_t UNIXTime = NTPTime - seventyYears;
        Serial.print("UTC Time: " );
        Serial.println(Global_NTPTime, HEX);

        // So far this is UTC time, so need to 'add' an hour in seconds * decimal offset +- and in quarter hours String instring = "1.5"; float f = inString.toFloat();
        int offset = utcOffset.toFloat() * oneHourInSeconds;
        Global_NTPTime += offset;
        Serial.print("UTC +- Timezone offset: " );
        Serial.println(Global_NTPTime, HEX);

      });
      phaseFourState = false;
      phaseFiveState = true;
      ntpSendCount = 0;
    }
  } // Phase Four

  if ( phaseFiveState ) {
    if ( ntpSendCount++ < 3 ) {
      Serial.println("Sending NTP request ...");
      memset(NTPBuffer, 0, NTP_PACKET_SIZE);  // set all bytes in the buffer to 0
      // Initialize values needed to form NTP request
      NTPBuffer[0] = 0b11100011;   // LI, Version, Mode
      // send a packet requesting a timestamp:
      AsyncUDPMessage msg;
      msg.write(NTPBuffer, NTP_PACKET_SIZE);
      ntpUdp.sendTo(msg, timeServerIP, 123);
      delay(5000);
      delayTimeCompensation = 10; // todo
    } else {
      phaseFiveState = false;
      phaseSixState = true;
    }
  } // Phase Five

  if ( phaseSixState ) {
    // Sending Subscribe to each known socket
    int tries = 3;
    while ( --tries > 0 ) {
      for ( int i = 0; i < knownSockets.size(); i++ ) {
        S20Socket * pSocket = knownSockets.getSocket( i );
        Serial.print("Sending Subscribe to Socket ");
        IPAddress ip = pSocket->ip;
        Serial.print(i + 1);
        Serial.print(" ");
        Serial.println(ip);
        memcpy(subscribe + 6, pSocket->mac, 6);
        rmac(subscribe + 6, subscribe + 18);
        hexdump(subscribe, 30);
        AsyncUDPMessage msg;
        msg.write(subscribe, 30);
        ntpUdp.sendTo(msg, ip, 10000);
        delay(1000);
        delayTimeCompensation += 1;
      }
    }

    // Sending CountdownSet to each known socket
    tries = 3;
    while ( --tries > 0 ) {
      for ( int i = 0; i < knownSockets.size(); i++ ) {
        setCountdownSetTime( Global_NTPTime + delayTimeCompensation );
        S20Socket * pSocket = knownSockets.getSocket( i );
        Serial.print("Sending Time Re-Sync to Socket ");
        IPAddress ip = pSocket->ip;
        Serial.print(i + 1);
        Serial.print(" ");
        Serial.println(ip);
        memcpy(countdownSet + 6, pSocket->mac, 6);
        hexdump(countdownSet, 26);
        AsyncUDPMessage msg;
        msg.write(countdownSet, 26);
        ntpUdp.sendTo(msg, ip, 10000);
        delay(1000);
        delayTimeCompensation += 1;
      }
    }

    phaseSixState = false;
    refreshTimer.start();
    cycleCount++;
    Serial.print("Waiting for refresh timer cycle ");
    Serial.println(cycleCount);

  } // Phase Six

  if ( refreshTimer.expired() ) {
    // We reset the timer
    phaseTwoTimer.limitMillis(100);
    phaseTwoTimer.start();
    phaseTwoState = true;
  }

}
// end of loop


