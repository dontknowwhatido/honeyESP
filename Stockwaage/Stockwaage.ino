//Librarys einbinden
#include <FS.h> 

#include "PubSubClient.h" //Knolleary
#include <OneWire.h>                // Benötigt für DS18B20
#include <DallasTemperature.h>      //DS18B20 LIB
#include <Wire.h>                   //i2C Lib
#include <SPI.h>                    //Serialle Library
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <WiFiManager.h>
#include "HX711.h"
#include <ArduinoJson.h>



// MQTT Daten
char mqttBroker[64] = "";
char mqttPort[64] = "";

int connectionDelay = 2;
int sleeptime = 30;

// Comment the following line if not using an ESP8266.
#define ESP8266BOARD

WiFiClient client;


bool shouldSaveConfig = false;

void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

PubSubClient mqttClient( client );
WiFiManager wifiManager;


//Sensorwerte
String sensor_names[5] = {"DS18b20temperature", "BME280temperature", "humidity", "pressure", "weight"};
int number_of_sensors = 5;
float sens_values[5];

//Verfügbarkeit BME280 Sensor
int bme_available = 1;

//Start der Sensoren
#define ONE_WIRE_BUS 14           //GPIO an dem OneWire

Adafruit_BME280 bme; // I2C
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
HX711 scale;

const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

// Function to handle messages from MQTT subscription.
void mqttSubscriptionCallback( char* topic, byte* payload, unsigned int length ) {
  // Print the details of the message that was received to the serial monitor.
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// Connect to WiFi.
void connectWifi()
{
  Serial.print( "Connecting to Wi-Fi..." );
  // Loop until WiFi connection is successful
  #ifdef ESP8266BOARD
    while ( WiFi.waitForConnectResult() != WL_CONNECTED ) {
  #else
    while ( WiFi.status() != WL_CONNECTED ) {
  #endif
    bool stationConnected = wifiManager.autoConnect();
  }
  Serial.println( "Connected to Wi-Fi." );
}

// Connect to MQTT server.
void mqttConnect() {
  // Loop until connected.
  while ( !mqttClient.connected() )
  {
    // Connect to the MQTT broker.
    if ( mqttClient.connect("honey-esp") ){
      Serial.print( "MQTT to " );
      Serial.print( mqttBroker );
      Serial.print (" at port ");
      Serial.print( mqttPort );
      Serial.println( " successful." );
    } else {
      Serial.print( "MQTT connection failed, rc = " );
      // See https://pubsubclient.knolleary.net/api.html#state for the failure code explanation.
      Serial.print( mqttClient.state() );
      Serial.println( " Will try again in a few seconds" );
      delay( connectionDelay*1000 );
    }
  }
}

//Funktion um Sensorwerte aus zu lesen
void get_values(){
  sensors.requestTemperatures();
  
  sens_values[0] = sensors.getTempCByIndex(0);
  
  if(bme_available == 1){
  
    sens_values[1] = bme.readTemperature(),1;
    sens_values[2] = bme.readHumidity(),1;
    sens_values[3] = bme.readPressure() / 100.0F;
  }

  else{
    sens_values[1] = -4000;
    sens_values[2] = -4000;
    sens_values[3] = -4000;
  }

  sens_values[4] = scale.read();
}

//Funktion um Sensorwerte an MQTT-Server zu schicken
void send_data(){
  for(int i; i < number_of_sensors; i++){
    if(sens_values[i] != -4000){
      mqttClient.publish( ("honey-esp/" + sensor_names[i]).c_str() , String(sens_values[i]).c_str() );
    }
  }
  delay(1000);
}

void printData(){
  Serial.print("TemperatureDS18b20 (ºC): ");
  Serial.println(sens_values[0]);
  Serial.print("TemperatureBME (ºC): ");
  Serial.println(sens_values[1]);
  Serial.print("humidity");
  Serial.println(sens_values[2]);
  Serial.print("pressure");
  Serial.println(sens_values[3]);
  Serial.print("weight");
  Serial.println(sens_values[4]);
}

void connectToServers(){
  // Reconnect to WiFi if it gets disconnected.
  if (WiFi.status() != WL_CONNECTED) {
      connectWifi();
  }

  // Configure the MQTT client
  mqttClient.setServer( mqttBroker, atoi(mqttPort) ); 
  // Set the MQTT message handler function.
  mqttClient.setCallback( mqttSubscriptionCallback );
  // Set the buffer to handle the returned JSON. NOTE: A buffer overflow of the message buffer will result in your callback not being invoked.
  mqttClient.setBufferSize( 2048 );


  // Connect if MQTT client is not connected and resubscribe to channel updates.
  if (!mqttClient.connected()) {
    mqttConnect();
  }
  Serial.println("connected");

  // Call the loop to maintain connection to the server.
  mqttClient.loop();
}

void setup() {
  Serial.begin( 115200 );
  //set pin for configbutton
  pinMode(13, INPUT);
  pinMode(12, INPUT);
  pinMode(15, OUTPUT);
  //disable Config-Portal from autoconnect
  wifiManager.setEnableConfigPortal(false);
  // Delay to allow serial monitor to come up.
  delay(3000);



  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument doc(1024);
        deserializeJson(doc, buf.get());
        serializeJson(doc, Serial);
        if (!doc.isNull()) {
          strcpy(mqttBroker, doc["mqttBroker"]);
          strcpy(mqttPort, doc["mqttPort"]);
          Serial.println("\nparsed json");
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  WiFiManagerParameter ask_mqttBroker("mqttBroker", "mqttBroker", mqttBroker, 30);
  WiFiManagerParameter ask_mqttPort("mqttPort", "mqttPort", mqttPort, 30);

  
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  saveConfigCallback();

  wifiManager.addParameter(&ask_mqttBroker);
  wifiManager.addParameter(&ask_mqttPort);




  // Open Config-Portal or connect to Wi-Fi network.
  if(digitalRead(13) == HIGH){
    wifiManager.startConfigPortal();
    connectWifi();
  }
  
  strcpy(mqttBroker, ask_mqttBroker.getValue());
  strcpy(mqttPort, ask_mqttPort.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonDocument doc(1024);
    doc["mqttBroker"] = mqttBroker;
    doc["mqttPort"] = mqttPort;
    

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    Serial.println("check");
    serializeJson(doc, Serial);
    serializeJson(doc, configFile);
    configFile.close();
    //end save
  }

  if(analogRead(A0) > 900){
    Serial.println("Arbeitsmodus");

    connectToServers();
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

    float oldWeight = scale.read();
    digitalWrite(15, HIGH);
    Serial.println("warten");

    while(digitalRead(12) != HIGH){
      delay(1000);
    }

    Serial.println("read Data");
    float newWeight = scale.read();

    float weightDifference = newWeight - oldWeight;

    Serial.println("send Data");
    mqttClient.publish("honey-esp/lastWeightDifference", String(weightDifference).c_str() );
    Serial.println("Data send");

    digitalWrite(15, LOW);
  }

  if(digitalRead(12) == HIGH){
    connectToServers();

    if (!bme.begin(0x76)) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      bme_available = 0;
    }

    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

    get_values();
    printData();
    send_data();

    Serial.println("DeepSleep");
    ESP.deepSleep(sleeptime*60e6);
  }
}


void loop() {}