# Somfy control & Multisensor project

Based on the [work of Sander de Haan](https://github.com/sanderdh/Somfy-Remote-multi-sensorh).
As with many projects: once it works, it works and move over to the next project. The drawback of this is that when wants to build the project a few years later chances are you cannot build it because the code depends on functions only found in other libraries.
For this reason I am documenting the libraries used/I am using, including the versionnumbers as too new/recent can give issues!

Another problem I came across is that the project simply did not work so obviously I made a mistake, but what?
This introduces the need to debug. I did get MQTT messages but command did not work, nor did I get values from the sensors. (all null)
The classic way to debug Arduino code is to use the serial monitor.
Sadly for some reason this did not give me any(!) information. Plugin/plugout the USB cable is tricky for me as I use a VM to flash the ESP as I did not want to install a modern Arduino IDE next to the ancient one provided by Debian.
So alternatively I decided to add debugging code and implement: [Arduino RemoteDebug](https://www.arduino.cc/reference/en/libraries/remotedebug/) which gives telnet (yuk!) and a HTTP debugger/profiler interface which can be disabled once in production.


## List of used libraries
* WiFi : Built-in by Arduino, version 1.2.7
* ArduinoJson by Benoit Blanchon 5.13.2 ; note: 6.x versions do not work unless code gets refactored
* DHT sensor library by Adafruit, version 1.4.2
* Adafruit Unified Sensor by Adafruit, version 1.1.4
* PubSubClient by Nick O'Leary, version 2.8.0
* RemoteDebug by Joao Lopes, version 3.0.5
* Telnet?
* mDNS?
* ArduinoOTA?
* Webserver?

## Used board
esp8266 by ESP8266 Community version 2.7.4 (3.x did not work for me, with the Amica NodeMCU I am using)
