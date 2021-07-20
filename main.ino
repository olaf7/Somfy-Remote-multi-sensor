#include <RemoteDebugWS.h>
#include <RemoteDebug.h>
#include <RemoteDebugCfg.h>
#include <telnet.h>

#include <Arduino.h>
#include <vector>

#include <DHT.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>

// #define DEBUG_DISABLED true

/* 
 *  Code taken from:
 *  https://github.com/cbrherms/ESP-MQTT-JSON-Multisensor/blob/8c87c4a5ac824b71de16bef0f4620344e4516924/bruh_mqtt_multisensor_github/bruh_mqtt_multisensor_github.ino
 *  and
 *  https://github.com/marmotton/Somfy_Remote.git
 *  
 *  Needs:
 *  https://github.com/adafruit/Adafruit_Sensor
 *  https://github.com/adafruit/DHT-sensor-library
 *  
 *  Note: I did not test this code on ESP32 yet.
 *  
 *  Adapted to run on ESP32 from original code at https://github.com/Nickduino/Somfy_Remote

This program allows you to emulate a Somfy RTS or Simu HZ remote.
If you want to learn more about the Somfy RTS protocol, check out https://pushstack.wordpress.com/somfy-rts-protocol/

The rolling code will be stored in non-volatile storage (Preferences), so that you can power the Arduino off.

Serial communication of the original code is replaced by MQTT over WiFi.

Modifications should only be needed in config.h.

For more information see https://projects.dehaan.net/

All the credits go to the original authors! (see links above)
*/

// Configuration of the remotes that will be emulated
struct REMOTE {
    unsigned int id;
    char const* mqtt_topic;
    unsigned int default_rolling_code;
    uint32_t eeprom_address;
};

#include "config.h"

float ldrValue;
int LDR;
float calcLDR;
float diffLDR = 25;

float diffTEMP = 0.2;
float tempValue;

float diffHUM = 1;
float humValue;

int pirValue;
int pirStatus;
String motionStatus;

char message_buff[100];

int calibrationTime = 0;

const int BUFFER_SIZE = 300;

#define MQTT_MAX_PACKET_SIZE 512

DHT dht(DHTPIN, DHTTYPE);

// GPIO macros
#ifdef ESP32
    #define SIG_HIGH GPIO.out_w1ts = 1 << PORT_TX
    #define SIG_LOW  GPIO.out_w1tc = 1 << PORT_TX
#elif ESP8266
    #define SIG_HIGH GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1 << PORT_TX)
    #define SIG_LOW  GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1 << PORT_TX)
#endif

// Store the rolling codes in NVS
#ifdef ESP32
    #include <Preferences.h>
    Preferences preferences;
#elif ESP8266
    #include <EEPROM.h>
#endif


// Wifi and MQTT
#ifdef ESP32
    #include <WiFi.h>
#elif ESP8266
    #include <ESP8266WiFi.h>
#endif

#include <PubSubClient.h>

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// Buttons
#define SYMBOL 640
#define HAUT 0x2
#define STOP 0x1
#define BAS 0x4
#define PROG 0x8

byte frame[7];

void BuildFrame(byte *frame, byte button, REMOTE remote);
void SendCommand(byte *frame, byte sync);
void receivedCallback(char* topic, byte* payload, unsigned int length);
void mqttconnect();

// Remote debugging setup \\

#ifndef DEBUG_DISABLED

  #define HOST_NAME "somfycube"
  #define WEB_SERVER_ENABLED true
  #define USE_MDNS true
  
  #if defined ESP8266
    #include <ESP8266WiFi.h>
  
    #ifdef USE_MDNS
      #include <DNSServer.h>
      #include <ESP8266mDNS.h>
    #endif // use mDNS
  
    #ifdef WEB_SERVER_ENABLED
      #include <ESP8266WebServer.h>
    #endif // web server
  
  #elif defined ESP32
    #include <WiFi.h>
  
    #ifdef USE_MDNS
      #include <DNSServer.h>
      #include "ESPmDNS.h"
    #endif // use mDNS
    
    #ifdef WEB_SERVER_ENABLED
      #include <WebServer.h>
    #endif // webserver

  #endif
  
#else

  #error "For now, RemoteDebug has sever limitation in our situation"

#endif // debug disabled

#ifdef WEB_SERVER_ENABLED
  #if defined ESP8266
    ESP8266WebServer HTTPServer(80);
  #elif defined ESP32
    WebServer HTTPServer(80);
  #endif // esp8266 / esp32
#endif // web server

#ifndef DEBUG_DISABLED
  RemoteDebug Debug;
#endif

void setup() {
    // USB serial port
    Serial.begin(115200);

    //connectWiFi();
    #ifdef ESP8266
      WiFi.hostname(HOST_NAME);
    #endif

    #ifndef DEBUG_DISABLED
      //RemoteDebug Debug;
      // Initialise the server (telnet or web socket) of RemoteDebug
      Debug.begin(HOST_NAME);
      // Options
      Debug.setResetCmdEnabled(true); // Enable the reset command
      Debug.showProfiler(true); // To show profiler - time between messages of Debug
      Debug.showColors(true);
      Debug.setSerialEnabled(true); // If you want serial echo - Only recommanded if ESP is pluged into USB

      Debug.println("RemoteDebug initialised");
      Serial.println("RemoteDebug initialised");
      #if defined USE_MDNS
        Debug.println("mDNS supported");
        Serial.println("mDNS supported");
      #else
        Debug.println("mDNS NOT supported");
        Serial.println("mDNS NOT supported");
      #endif
      #if defined USE_WEB_SERVER
        Debug.println("Webserver will be available");
        Serial.println("Webserver will be available");
      #else
        Debug.println("Webserver will NOT be available");
        Serial.println("Webserver will NOT be available");
      #endif
    #endif

    #if defined USE_MDNS && defined HOST_NAME
      if (MDNS.begin(HOST_NAME)) {
        Serial.print("* MDNS responder started. Hostname -> ");
        Serial.println(HOST_NAME);
        #ifndef DEBUG_DISABLED
          if (Debug.isActive(Debug.INFO)) {
            Debug.print("* MDNS responder started. Hostname -> ");
            Debug.println(HOST_NAME);
          }
        #endif
      }
      
      #ifdef WEB_SERVER_ENABLED
        MDNS.addService("http", "tcp", 80);   // Web Server
      #endif
      
      #ifndef DEBUG_DISABLED
        MDNS.addService("telnet", "tcp", 23); // Telnet setrver of RemoteDebug, register as telnet
        // telnet is evil...
      #endif
      
    #endif  // MDNS / hostname

    #ifdef WEB_SERVER_ENABLED
      HTTPServer.on("/", handleRoot);
      HTTPServer.onNotFound(handleNotFound);
      HTTPServer.begin();
      Serial.println("* HTTP server started");
      #ifndef DEBUG_DISABLED
        if (Debug.isActive(Debug.INFO)) {
          Debug.print("* HTTP server started. Hostname -> ");
          Debug.println(HOST_NAME);
          // also report the IP addresses (ipv4/ipv6) ? --> done elsewhere
        }
      #endif
    #endif

    pinMode(PIRPIN, INPUT);
    pinMode(DHTPIN, INPUT);
    pinMode(LDRPIN, INPUT);

    ArduinoOTA.setPort(OTAport);
    ArduinoOTA.setHostname(mqtt_id);
    ArduinoOTA.setPassword((const char *)OTApassword);

    Serial.print("calibrating sensor ");
    #ifndef DEBUG_DISABLED
    if (Debug.isActive(Debug.INFO)) {
      Debug.print("calibrating sensor ");
    }
    #endif
    
    for (int i = 0; i < calibrationTime; i++) {
      Serial.print(".");
      #ifndef DEBUG_DISABLED
      if (Debug.isActive(Debug.INFO)) {
        Debug.print(".");
      }
      #endif
      delay(1000);
    }

    ArduinoOTA.onStart([]() {
      Serial.println("Starting");
      #ifndef DEBUG_DISABLED
      if (Debug.isActive(Debug.INFO)) {
        Debug.println("Starting");
      }
      #endif
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
      #ifndef DEBUG_DISABLED
      if (Debug.isActive(Debug.DEBUG)) {
        Debug.println("\nEnd");
      }
      #endif
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      #ifndef DEBUG_DISABLED
      if (Debug.isActive(Debug.DEBUG)) {
        Debug.printf("Progress: %u%%\r", (progress / (total / 100)));
      }
      #endif
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      #ifndef DEBUG_DISABLED
      if (Debug.isActive(Debug.DEBUG)) {
        Debug.printf("Error[%u]: ", error);
      }
      #endif
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
        #ifndef DEBUG_DISABLED
        if (Debug.isActive(Debug.DEBUG)) {
          Debug.println("Auth Failed");
        }
        #endif
      }
      else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
        #ifndef DEBUG_DISABLED
        if (Debug.isActive(Debug.DEBUG)) {
          Debug.println("Begin Failed");
        }
        #endif
      }
      else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
        #ifndef DEBUG_DISABLED
        if (Debug.isActive(Debug.DEBUG)) {
          Debug.println("Connect Failed");
        }
        #endif
      }
      else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
        #ifndef DEBUG_DISABLED
        if (Debug.isActive(Debug.DEBUG)) {
          Debug.println("Receive Failed");
        }
        #endif
      }
      else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
        #ifndef DEBUG_DISABLED
        if (Debug.isActive(Debug.DEBUG)) {
          Debug.println("End Failed");
        }
        #endif
      }
    });
    ArduinoOTA.begin();
    
    // Output to 433.42MHz transmitter
    pinMode(PORT_TX, OUTPUT);
    SIG_LOW;

    // Connect to WiFi
    Serial.print("Connecting to ");
    Serial.println(wifi_ssid);
    #ifndef DEBUG_DISABLED
    if (Debug.isActive(Debug.INFO)) {
      Debug.print("Connecting to ");
      Debug.println(wifi_ssid);
    }
    #endif

    WiFi.begin(wifi_ssid, wifi_password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        #ifndef DEBUG_DISABLED
        if (Debug.isActive(Debug.INFO)) {
          Debug.print(".");
        }
        #endif
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    #ifndef DEBUG_DISABLED
    if (Debug.isActive(Debug.INFO)) {
      Debug.println("");
      Debug.println("WiFi connected");
      Debug.println("IP address: ");
      Debug.println(WiFi.localIP());
    }
    #endif

    // Configure MQTT
    mqtt.setServer(mqtt_server, mqtt_port);
    mqtt.setCallback(receivedCallback);

    // Open storage for storing the rolling codes
    #ifdef ESP32
        preferences.begin("somfy-remote", false);
    #elif ESP8266
        EEPROM.begin(1024);
    #endif

    // Clear all the stored rolling codes (not used during normal operation). Only ESP32 here (ESP8266 further below).
    #ifdef ESP32
        if ( reset_rolling_codes ) {
                preferences.clear();
        }
    #endif

    // Print out all the configured remotes.
    // Also reset the rolling codes for ESP8266 if needed.
    for ( REMOTE remote : remotes ) {
      
        Serial.print("Simulated remote number : ");
        Serial.println(remote.id, HEX);
        Serial.print("Current rolling code    : ");
        
        #ifndef DEBUG_DISABLED
        if (Debug.isActive(Debug.DEBUG)) {
          Debug.print("Simulated remote number : ");
          Debug.println(remote.id, HEX);
          Debug.print("Current rolling code    : ");
        }
        #endif
        
        unsigned int current_code;

        #ifdef ESP32
            current_code = preferences.getUInt( (String(remote.id) + "rolling").c_str(), remote.default_rolling_code);
        #elif ESP8266
            if ( reset_rolling_codes ) {
                EEPROM.put( remote.eeprom_address, remote.default_rolling_code );
                EEPROM.commit();
            }
            EEPROM.get( remote.eeprom_address, current_code );
        #endif

        Serial.println( current_code );
        #ifndef DEBUG_DISABLED
        if (Debug.isActive(Debug.DEBUG)) {
          Debug.println( current_code );
        }
        #endif
    }
    Serial.println();
    #ifndef DEBUG_DISABLED
    if (Debug.isActive(Debug.DEBUG)) {
      Debug.println();
    }
    #endif
}

/********************************** START SEND STATE*****************************************/
void sendState() {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();

  root["humidity"] = (String)humValue;
  root["motion"] = (String)motionStatus;
  root["ldr"] = (String)LDR;
  root["temperature"] = (String)tempValue;
  root["heatIndex"] = (String)dht.computeHeatIndex(tempValue, humValue, IsFahrenheit);


  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  Serial.println(buffer);
  #ifndef DEBUG_DISABLED
  if (Debug.isActive(Debug.DEBUG)) {
    Debug.println(buffer);
  }
  #endif
  
  mqtt.publish(state_topic, buffer, true);
}

void loop() {

    ArduinoOTA.handle();
      
    // Reconnect MQTT if needed
    if ( !mqtt.connected() ) {
        mqttconnect();
    }

    mqtt.loop();

    float newTempValue = dht.readTemperature(IsFahrenheit);
    float newHumValue = dht.readHumidity();

    //PIR CODE
    pirValue = digitalRead(PIRPIN); //read state of the

    if (pirValue == LOW && pirStatus != 1) {
      motionStatus = "standby";
      sendState();
      pirStatus = 1;
    }

    else if (pirValue == HIGH && pirStatus != 2) {
      motionStatus = "motion detected";
      sendState();
      pirStatus = 2;
    }

    if (checkBoundSensor(newTempValue, tempValue, diffTEMP)) {
      tempValue = newTempValue;
      sendState();
    }

    if (checkBoundSensor(newHumValue, humValue, diffHUM)) {
      humValue = newHumValue;
      sendState();
    }


    int newLDR = analogRead(LDRPIN);

    if (checkBoundSensor(newLDR, LDR, diffLDR)) {
      LDR = newLDR;
      sendState();
    }

    Debug.handle(); // Remote debug over WiFi
    // debughandle(); // Equal to SerialDebug
    
    delay(100);
}

void mqttconnect() {
    // Loop until reconnected
    while ( !mqtt.connected() ) {
        Serial.print("MQTT connecting ...");
        #ifndef DEBUG_DISABLED
        if (Debug.isActive(Debug.INFO)) {
          Debug.print("MQTT connecting ...");
        }
        #endif

        // Connect to MQTT, with retained last will message "offline"
        if (mqtt.connect(mqtt_id, mqtt_user, mqtt_password, status_topic, 1, 1, "offline")) {
            Serial.println("connected");
            #ifndef DEBUG_DISABLED
            if (Debug.isActive(Debug.INFO)) {
              Debug.println("connected");
            }
            #endif

            // Subscribe to the topic of each remote with QoS 1
            for ( REMOTE remote : remotes ) {
                mqtt.subscribe(remote.mqtt_topic, 1);
                Serial.print("Subscribed to topic: ");
                Serial.println(remote.mqtt_topic);
                #ifndef DEBUG_DISABLED
                if (Debug.isActive(Debug.DEBUG)) {
                  Debug.print("Subscribed to topic : ");
                  Debug.println(remote.mqtt_topic);
                }
                #endif
            }

            // Update status, message is retained
            mqtt.publish(status_topic, "online", true);
        }
        else {
            Serial.print("failed, status code =");
            Serial.print(mqtt.state());
            Serial.println("try again in 5 seconds");
            #ifndef DEBUG_DISABLED
            if (Debug.isActive(Debug.DEBUG)) {
              Debug.print("failed, status code = ");
              Debug.print(mqtt.state());
              Debug.println("try again in 5 seconds");
            }
            #endif
            
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void receivedCallback(char* topic, byte* payload, unsigned int length) {
    char command = *payload; // 1st byte of payload
    bool commandIsValid = false;
    REMOTE currentRemote;

    Serial.print("MQTT message received: ");
    Serial.println(topic);
    #ifndef DEBUG_DISABLED
    if (Debug.isActive(Debug.DEBUG)) {
      Debug.print("MQTT message received : ");
      Debug.println(topic);
    }
    #endif

    Serial.print("Payload: ");
    #ifndef DEBUG_DISABLED
    if (Debug.isActive(Debug.DEBUG)) {
      Debug.print("Payload : ");
    }
    #endif
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
        #ifndef DEBUG_DISABLED
        if (Debug.isActive(Debug.DEBUG)) {
          Debug.print((char)payload[i]);
        }
        #endif
    }
    Serial.println();
    #ifndef DEBUG_DISABLED
    if (Debug.isActive(Debug.DEBUG)) {
      Debug.println();
    }
    #endif

    // Command is valid if the payload contains one of the chars below AND the topic corresponds to one of the remotes
    if ( length == 1 && ( command == 'u' || command == 's' || command == 'd' || command == 'p' ) ) {
        for ( REMOTE remote : remotes ) {
            if ( strcmp(remote.mqtt_topic, topic) == 0 ){
                currentRemote = remote;
                commandIsValid = true;
            }
        }
    }

    if ( commandIsValid ) {
        if ( command == 'u' ) {
            Serial.println("Monte"); // Somfy is a French company, after all.
            #ifndef DEBUG_DISABLED
            if (Debug.isActive(Debug.DEBUG)) {
              Debug.println("Monte"); // Somfy is a French company, after all.
            }
    #endif
            BuildFrame(frame, HAUT, currentRemote);
        }
        else if ( command == 's' ) {
            Serial.println("Stop");
            #ifndef DEBUG_DISABLED
            if (Debug.isActive(Debug.DEBUG)) {
              Debug.println("Stop");
            }
            #endif
            BuildFrame(frame, STOP, currentRemote);
        }
        else if ( command == 'd' ) {
            Serial.println("Descend");
            #ifndef DEBUG_DISABLED
            if (Debug.isActive(Debug.DEBUG)) {
              Debug.println("Descend");
            }
            #endif
            BuildFrame(frame, BAS, currentRemote);
        }
        else if ( command == 'p' ) {
            Serial.println("Prog");
            #ifndef DEBUG_DISABLED
            if (Debug.isActive(Debug.DEBUG)) {
              Debug.println("Prog");
            }
            #endif
            BuildFrame(frame, PROG, currentRemote);
        }

        Serial.println("");
        #ifndef DEBUG_DISABLED
        if (Debug.isActive(Debug.DEBUG)) {
          Debug.println("");
        }
        #endif

        SendCommand(frame, 2);
        for ( int i = 0; i<2; i++ ) {
            SendCommand(frame, 7);
        }

        // Send the MQTT ack message
        String ackString = "id: 0x";
        ackString.concat( String(currentRemote.id, HEX) );
        ackString.concat(", cmd: ");
        ackString.concat(command);
        mqtt.publish(ack_topic, ackString.c_str());
    }
}

void BuildFrame(byte *frame, byte button, REMOTE remote) {
    unsigned int code;

    #ifdef ESP32
        code = preferences.getUInt( (String(remote.id) + "rolling").c_str(), remote.default_rolling_code);
    #elif ESP8266
        EEPROM.get( remote.eeprom_address, code );
    #endif

    frame[0] = 0xA7;            // Encryption key. Doesn't matter much
    frame[1] = button << 4;     // Which button did  you press? The 4 LSB will be the checksum
    frame[2] = code >> 8;       // Rolling code (big endian)
    frame[3] = code;            // Rolling code
    frame[4] = remote.id >> 16; // Remote address
    frame[5] = remote.id >>  8; // Remote address
    frame[6] = remote.id;       // Remote address

    Serial.print("Frame         : ");
    #ifndef DEBUG_DISABLED
    if (Debug.isActive(Debug.DEBUG)) {
      Debug.println("Frame          :");
    }
    #endif
    for(byte i = 0; i < 7; i++) {
        if(frame[i] >> 4 == 0) { //  Displays leading zero in case the most significant nibble is a 0.
            Serial.print("0");
            #ifndef DEBUG_DISABLED
            if (Debug.isActive(Debug.DEBUG)) {
              Debug.print("0");
            }
            #endif
        }
        Serial.print(frame[i],HEX); Serial.print(" ");
        #ifndef DEBUG_DISABLED
        if (Debug.isActive(Debug.DEBUG)) {
          Debug.print(frame[i],HEX);
          Debug.print(" ");
        }
        #endif
    }

    // Checksum calculation: a XOR of all the nibbles
    byte checksum = 0;
    for(byte i = 0; i < 7; i++) {
        checksum = checksum ^ frame[i] ^ (frame[i] >> 4);
    }
    checksum &= 0b1111; // We keep the last 4 bits only


    // Checksum integration
    frame[1] |= checksum; //  If a XOR of all the nibbles is equal to 0, the blinds will consider the checksum ok.

    Serial.println(""); Serial.print("With checksum : ");
    #ifndef DEBUG_DISABLED
    if (Debug.isActive(Debug.DEBUG)) {
      Debug.println("");
      Debug.print("With checksum : ");
    }
    #endif
    for(byte i = 0; i < 7; i++) {
        if(frame[i] >> 4 == 0) {
            Serial.print("0");
            #ifndef DEBUG_DISABLED
            if (Debug.isActive(Debug.DEBUG)) {
              Debug.print("0");
            }
            #endif
        }
        Serial.print(frame[i],HEX); Serial.print(" ");
        #ifndef DEBUG_DISABLED
        if (Debug.isActive(Debug.DEBUG)) {
          Debug.print(frame[i],HEX);
          Debug.print(" ");
        }
        #endif
    }


    // Obfuscation: a XOR of all the bytes
    for(byte i = 1; i < 7; i++) {
        frame[i] ^= frame[i-1];
    }

    Serial.println(""); Serial.print("Obfuscated    : ");
    #ifndef DEBUG_DISABLED
    if (Debug.isActive(Debug.INFO)) {
      Debug.println();
      Debug.print("Obfuscated    : ");
    }
    #endif
    for(byte i = 0; i < 7; i++) {
        if(frame[i] >> 4 == 0) {
            Serial.print("0");
            #ifndef DEBUG_DISABLED
            if (Debug.isActive(Debug.DEBUG)) {
              Debug.print("0");
            }
            #endif
        }
        Serial.print(frame[i],HEX); Serial.print(" ");
        #ifndef DEBUG_DISABLED
        if (Debug.isActive(Debug.DEBUG)) {
          Debug.print(frame[i], HEX);
          Debug.print(" ");
        }
        #endif
    }
    Serial.println("");
    Serial.print("Rolling Code  : ");
    Serial.println(code);
    #ifndef DEBUG_DISABLED
    if (Debug.isActive(Debug.DEBUG)) {
      Debug.println("");
      Debug.print("Rolling Code  : ");
      Debug.println(code);
    }
    #endif

    #ifdef ESP32
        preferences.putUInt( (String(remote.id) + "rolling").c_str(), code + 1); // Increment and store the rolling code
    #elif ESP8266
        EEPROM.put( remote.eeprom_address, code + 1 );
        EEPROM.commit();
    #endif
}

void SendCommand(byte *frame, byte sync) {
    if(sync == 2) { // Only with the first frame.
        //Wake-up pulse & Silence
        SIG_HIGH;
        delayMicroseconds(9415);
        SIG_LOW;
        delayMicroseconds(89565);
    }

    // Hardware sync: two sync for the first frame, seven for the following ones.
    for (int i = 0; i < sync; i++) {
        SIG_HIGH;
        delayMicroseconds(4*SYMBOL);
        SIG_LOW;
        delayMicroseconds(4*SYMBOL);
    }

    // Software sync
    SIG_HIGH;
    delayMicroseconds(4550);
    SIG_LOW;
    delayMicroseconds(SYMBOL);

    //Data: bits are sent one by one, starting with the MSB.
    for(byte i = 0; i < 56; i++) {
        if(((frame[i/8] >> (7 - (i%8))) & 1) == 1) {
            SIG_LOW;
            delayMicroseconds(SYMBOL);
            SIG_HIGH;
            delayMicroseconds(SYMBOL);
        }
        else {
            SIG_HIGH;
            delayMicroseconds(SYMBOL);
            SIG_LOW;
            delayMicroseconds(SYMBOL);
        }
    }

    SIG_LOW;
    delayMicroseconds(30415); // Inter-frame silence
}

/********************************** START CHECK SENSOR **********************************/
bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}

#ifdef WEB_SERVER_ENABLED
/////////// Handles

void handleRoot() {
  // Root webpage
  String message = "Welcome to ESP";
  message.concat(HOST_NAME);
  message.concat(" remote debugging");
  HTTPServer.send(200, "text/plain", message);
  #ifndef DEBUG_DISABLED
  if (Debug.isActive(Debug.DEBUG)) {
    Debug.print("Sending HTTP - 200 text/plain: ");
    Debug.print(message);
  }
  #endif
}

void handleNotFound() {
  // Page not found (404)
  String message = "File Not Found\n\n";
  message.concat("URI: ");
  message.concat(HTTPServer.uri());
  message.concat("\nMethod: ");
  message.concat((HTTPServer.method() == HTTP_GET)?"GET":"POST");
  message.concat("\nArguments: ");
  message.concat(HTTPServer.args());
  message.concat("\n");
  for (uint8_t i=0; i<HTTPServer.args(); i++){
    message.concat(" " + HTTPServer.argName(i) + ": " + HTTPServer.arg(i) + "\n");
  }
  HTTPServer.send(404, "text/plain", message);
  #ifndef DEBUG_DISABLED
  if (Debug.isActive(Debug.DEBUG)) {
    Debug.print("Sending HTTP - 404 text/plain: ");
    Debug.print(message);
  }
  #endif
}

#endif
