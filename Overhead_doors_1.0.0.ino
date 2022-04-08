/***
              _____ _                _
             / ____| |              | |
            | (___ | |__   ___ _ __ | | __
             \___ \| '_ \ / _ \ '_ \| |/ /
             ____) | |_| |  __/ | | |   <      _   _
           /\_____/|_| |_|\___|_| |_|_|\_\    | | (_)
          /  \  _   _| |_ ___  _ __ ___   __ _| |_ _  ___  _ __
         / /\ \| | | | __/ _ \| '_ ` _ \ / _` | __| |/ _ \| '_ \
        / ____ \ |_| | || (_) | | | | | | (_| | |_| | (_) | | | |
       /_/    \_\__,_|\__\___/|_| |_| |_|\__,_|\__|_|\___/|_| |_|
             / ____|         | |
            | (___  _   _ ___| |_ ___ _ __ ___  ___
             \___ \| | | / __| __/ _ \ '_ ` _ \/ __|
             ____) | |_| \__ \ ||  __/ | | | | \__ \
            |_____/ \__, |___/\__\___|_| |_| |_|___/
                     __/ |
                    |___/

   
   Overhead Door Opener V1.0.0 Jan. 2022
   For ESP32
*/

#define DOOR_OPERATOR_VERSION "1.0.0"

// libraries
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//*************************  Set Debugging on or off **********************

// Comment the following line if not debuging
#define __DEBUG__

#if defined (__DEBUG__)
#define DEBUG_begin(val) Serial.begin(val)
#define DEBUG_print(val) Serial.print(val)
#define DEBUG_println(val) Serial.println(val)
#else
#define DEBUG_begin(val)
#define DEBUG_print(val)
#define DEBUG_println(val)
#endif

//************************* Define Variables ******************************
int openLimit_1 = 26;
int openApproach_1 = 15;
int openButton_1 = 34;
int closeLimit_1 = 27;
int closeApproach_1 = 16;
int closeButton_1 = 35;
int motorClose_1 = 33;
int motorOpen_1 = 32;
int stopButton_1 = 36;
int openLimit_2 = 17;
int openApproach_2 = 13;
int openButton_2 = 22;
int closeLimit_2 = 18;
int closeApproach_2 = 14;
int closeButton_2 = 23;
int motorClose_2 = 21;
int motorOpen_2 = 19;
int stopButton_2 = 25;
int beam = 39;

// pwm channel assinment
const int pwmOpen_1 = 0; 
const int pwnClose_1 = 1;
const int pwmOpen_2 = 3; 
const int pwnClose_2 = 4;
// pwm frequency assinment
const int frequency = 1500;
// pwm resolution assinment
const int resolution = 8;

//boolean isDoorOpen = true;

//*************************  NETWORK INFO  ********************************
const char* ssid = "FlatHillsFarm";
const char* password = "halliard";
const char* mqtt_server = "192.168.0.101";
const char* mqtt_user = "jeffrey";
const char* mqtt_password = "Dexter01";
const char* mqtt_clent = "Overhead_Door";

//*************************   MQTT String Setup ***************************

WiFiClient OTA_Tepmlate;
PubSubClient client(OTA_Tepmlate);
long lastMsg = 0;
char LStempfstring[10];
char UStempfstring[10];
int value = 0;

//***************************  Setup WIFI  *********************************
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
//*********************  END WIFI SETUP  ************************************

//***********   void callback listens for incoming mqtt messages  ***********
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

//***********************  Reconnect to MQTT  ********************************
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_clent, mqtt_user, mqtt_password)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//****************************  BEGIN SETUP  **********************************
void setup() {

  Serial.begin(115200);;
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // allow OTA update from inside Arduino IDE
  enableOTAfromIDE();

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

// pwm channel assinment
const int pwmOpen_1 = 0; 
const int pwnClose_1 = 1;
const int pwmOpen_2 = 3; 
const int pwnClose_2 = 4;
// pwm frequency assinment
const int frequency = 1500;
// pwm resolution assinment
const int resolution = 8;
  
  pinMode(openApproach_1, INPUT);
  pinMode(openLimit_1, INPUT);
  pinMode(closeApproach_1, INPUT);
  pinMode(closeLimit_1, INPUT);
  pinMode(openButton_1, INPUT);
  pinMode(closeButton_1, INPUT);
  pinMode(stopButton_1, INPUT);

  pinMode(openApproach_2, INPUT);
  pinMode(openLimit_2, INPUT);
  pinMode(closeApproach_2, INPUT);
  pinMode(closeLimit_2, INPUT);
  pinMode(openButton_2, INPUT);
  pinMode(closeButton_2, INPUT);
  pinMode(stopButton_2, INPUT);
  pinMode(beam, INPUT);

  pinMode(motorOpen_1, OUTPUT);
  pinMode(motorClose_1, OUTPUT);
  pinMode(pwmmotorOpen_2, OUTPUT);
  pinMode(motorClose_2, OUTPUT);

  ledcSetup(pwmOpen_1, frequency, resolution);
  ledAttachPin(motorOpen_1, pwmOpen_1);
  ledcSetup(pwmOpen_2, frequency, resolution);
  ledAttachPin(motorOpen_2, pwmOpen_2);
  ledcSetup(pwmClose_1, frequency, resolution);
  ledAttachPin(motorClose_1, pwmClose_1);
  ledcSetup(pwmClose_2, frequency, resolution);
  ledAttachPin(motorClose_2, pwmClose_2); 

}
//*********************  END SETUP  ************************************

//*********************  BEGIN LOOP  ***********************************
void loop() {

  /*********  Read Sensors       ****************/
  ArduinoOTA.handle();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
    ++value;
    /*
        // Publish LStempf
        dtostrf(LStempf, 2, 1, LStempfstring);
        client.publish("IOnode01/LStempf", LStempfstring);
        Serial.println("published LStempf");

        dtostrf(UStempf, 2, 1, UStempfstring);
        client.publish("IOnode01/UStempf", UStempfstring);
        Serial.println("published UStempf");
    */

  }

}
//***********************  END MAIN LOOP  ********************************

//***************** OTA programming from inside Arduino IDE **************
void enableOTAfromIDE()
{
  
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("OverHead_Doors");
  // No authentication by default
  //ArduinoOTA.setPassword((const char *)"123");
  ArduinoOTA.onStart([]()
  {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]()
  {
    Serial.println("End");
  });
  ArduinoOTA.onError([](ota_error_t error)
  {
    Serial.print("Error: ");
    Serial.println(error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}