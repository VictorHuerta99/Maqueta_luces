

//https://github.com/acrobotic/Ai_Tips_ESP8266/blob/master/webserver_websockets/webserver_websockets.ino

/*------------------------------------------------------------------------------
  07/01/2018
  Author: Makerbro
  Platforms: ESP8266
  Language: C++/Arduino
  File: webserver_html.ino
  ------------------------------------------------------------------------------
  Description:
  Code for YouTube video demonstrating how to transfer data between a web server
  and a web client in real-time using websockets.
  https://youtu.be/ROeT-gyYZfw
  Do you like my videos? You can support the channel:
  https://patreon.com/acrobotic
  https://paypal.me/acrobotic
  ------------------------------------------------------------------------------
  Please consider buying products from ACROBOTIC to help fund future
  Open-Source projects like this! We'll always put our best effort in every
  project, and release all our design files and code for you to use.
  https://acrobotic.com/
  https://amazon.com/acrobotic
  ------------------------------------------------------------------------------
  License:
  Please see attached LICENSE.txt file for details.
-----------------------------------------

/* LSM9DS1_MS5611_t3 Basic Example Code
 by: Kris Winer
 date: November 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.

 Demonstrate basic LSM9DS1 functionality including parameterizing the register addresses, initializing the sensor,
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.

 This sketch is intended specifically for the LSM9DS1+MS5611 Add-on shield for the Teensy 3.1.
 It uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 The MS5611 is a simple but high resolution pressure sensor, which can be used in its high resolution
 mode but with power consumption od 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 4K7 resistors are on the LSM9DS1+MS5611 Teensy 3.1 add-on shield/breakout board.

 Hardware setup:
 LSM9DS1Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND

 Note: The LSM9DS1 is an I2C sensor and can use the Arduino Wire library.
 Because the sensor is not 5V tolerant, we are using either a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>

ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

#define rele1  D0  //(D0)
#define rele2  D1 // D1
#define rele3  D2 // D3
#define boton  D4 //D4


const char WiFiAPPSK[] = "science_7425";  //CONTRASEÑA
const char ssid[] = "Inventoteca 2G"; // NOMBRE DE LA RED
uint8_t pin_led = 2;

char webpage[] PROGMEM = R"=====(
<html>
<head>
  <script>
    var Socket;

    function init()
    {
      Socket = new WebSocket('ws://' + window.location.hostname + ':81/',['arduino']);

      Socket.onopen = function ()
      {
        Socket.send('Connect ' + new Date());
      }

      Socket.onmessage = function(event)
      {
        document.getElementById("rxConsole").value += event.data;
      }
    }

    function sendText()
    {
      Socket.send(document.getElementById("txBar").value);
      document.getElementById("txBar").value = "";
    }

    function sendBrightness()
    {
      Socket.send("#"+document.getElementById("brightness").value);
    }
       function sendlamp2()
    {
      Socket.send("*"+document.getElementById("lamp2").value);
    }
     function sendlamp3()
    {
      Socket.send("$"+document.getElementById("lamp3").value);
    }

  </script>
</head>
<body onload="javascript:init()">
  <div>
    <textarea id="rxConsole"></textarea>
  </div>
  <hr/>
  <div>
    <br/>texto:<input type="text" id="txBar" onkeydown="if(event.keyCode == 13) sendText();" />
  </div>
  <hr/>
  <div>
    <br/>LAMPARA:<input type="range" min="0" max="1023" value="512" id="brightness" oninput="sendBrightness()" />
  </div>
   <div>
    <br/>LAMPARA2:<input type="range" min="0" max="1" value="0" id="lamp2" oninput="sendlamp2()" />
  </div>
   <div>
    <br/>LAMPARA3:<input type="range" min="0" max="1" value="0" id="lamp3" oninput="sendlamp3()" />
  </div>
</body>
</html>
)=====";




//--------------------------------------------------------------- setup
void setup()
{
  Serial.begin(115200);
  Serial.print("Iniciando");

  //Engine channels
  pinMode(rele1, OUTPUT);
  pinMode(rele2,OUTPUT);
  pinMode(rele3,OUTPUT);
  pinMode(boton,INPUT);
  digitalWrite(rele1, HIGH);
  digitalWrite(rele2,HIGH);
  digitalWrite(rele3,HIGH);


  delay(1000);//wait for a second

  setupWiFi();


  if(MDNS.begin("esp8266"))
  {
    Serial.println("\nMDNS responder started");
  }

  // handle index
  server.on("/", []()
  {
    // send index.html
    server.send_P(200, "text/html", webpage);

  });

  server.begin();
  // start webSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Add service to MDNS
  MDNS.addService("http", "tcp", 80);
  MDNS.addService("ws", "tcp", 81);
}

//----------------------------------------------------------- loop
void loop()
{
  int boton_1;
  boton_1=digitalRead(boton);
  //Serial.println(boton_1);
  webSocket.loop();
  server.handleClient();
  if(Serial.available() > 0)
  {
    char c[] = {(char)Serial.read()};
    webSocket.broadcastTXT(c, sizeof(c));
  }
}

//-------------------------------------------------------------webSocketEvent
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length)
{
  switch(type)
    {
      case WStype_TEXT:
      //---------------------------------------------- rele 1
        if(payload[0] == '#')
        {
          uint16_t brightness = (uint16_t) strtol((const char *) &payload[1], NULL, 10);
          brightness = 1024 - brightness;
          analogWrite(pin_led, brightness);
        //  Serial.print("brightness= ");
          // Serial.println(brightness);
          if (brightness>512)
          {
            digitalWrite(rele1,HIGH);
          }
          else
            digitalWrite(rele1,LOW);
        }
       //-------------------------------------------- rele 2
        else if(payload[0]=='*')
        {
          uint16_t lamp2 = (uint16_t) strtol((const char *) &payload[1], NULL, 10);
            if (lamp2==1)
          {

              pinMode(rele2,OUTPUT);
              digitalWrite(rele2,LOW);
          }
          else {



             digitalWrite(rele2,HIGH);
             pinMode(rele2,INPUT);
             }
        }
        //---------------------------------------------- rele 3
         else if(payload[0]=='$')
        {
          uint16_t lamp3 = (uint16_t) strtol((const char *) &payload[1], NULL, 10);
            if (lamp3==1)
          {

              pinMode(rele3,OUTPUT);
              digitalWrite(rele3,LOW);
          }
          else {



             digitalWrite(rele3,HIGH);
             pinMode(rele3,INPUT);
             }
        }



        //----------------------------------------------- texto libre
        else
        {
          for(int i = 0; i < length; i++)
            Serial.print((char) payload[i]);
          Serial.println();
        }
      break;

      case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
      break;


     case WStype_CONNECTED:
        {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);

            // send message to client
            webSocket.sendTXT(num, "Connected");
        }
            break;

    }
}

//------------------------------------------------------------ setupWiFi
void setupWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, WiFiAPPSK);
  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  //WiFi.mode(WIFI_SoftAP);
  //WiFi.softAP(ssid, WiFiAPPSK);
  Serial.print("\ningenieros IP Address: ");
  Serial.println(WiFi.localIP());
  //Serial.println(WiFi.IP());
}
