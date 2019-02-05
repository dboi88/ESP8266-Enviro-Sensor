 #include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
int chargestate = 0;


/************************* Adafruit_BMP=E280 Address *********************************/
Adafruit_BME280 bme; // I2C
unsigned long delayTime;

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "YOUR WIFI SSID"
#define WLAN_PASS       "YOUR WIFI PASSWORD"

/************************* dboi.co.uk MQTT Setup *********************************/

#define AIO_SERVER      "YOUR SERVER IP"
#define AIO_SERVERPORT  YOUR SERVER PORT                   // use 8883 for SSL
#define AIO_USERNAME    "YOUR MQTT USERNAME"
#define AIO_KEY         "YOUR MQTT PASSWORD"

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
//WiFiClient client;
// or... use WiFiFlientSecure for SSL
WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt,"places/home/inside/upstairs/bedroom/sensors/roomtemp");
Adafruit_MQTT_Publish Pressure = Adafruit_MQTT_Publish(&mqtt,"places/home/inside/upstairs/bedroom/sensors/roompressure");
Adafruit_MQTT_Publish Humidity = Adafruit_MQTT_Publish(&mqtt,"places/home/inside/upstairs/bedroom/sensors/roomhumidity");
Adafruit_MQTT_Publish battery = Adafruit_MQTT_Publish(&mqtt,"places/home/inside/upstairs/bedroom/sensors/roomtemp/battery");
Adafruit_MQTT_Publish batterystate = Adafruit_MQTT_Publish(&mqtt,"places/home/inside/upstairs/bedroom/sensors/roomtemp/batterystate");



/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();

void setup() {
  // Connect to WiFi access point.
  Serial.begin(9600);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  Serial.print("Wifi Started ");
  Serial.println(millis());
  while (WiFi.status() != WL_CONNECTED) {
    delay(50);
    Serial.println("connecting to wifi");
  }

  Serial.print("connected to  wifi");
  Serial.println(WLAN_SSID);

  // Begin Wire for bmp Ic2 Reading
  if (!bme.begin()) {  
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    chargestate = 1;
  }else{
    Serial.print("Started BME280 ");
    Serial.println(millis());
  }
  
  Serial.println("-- Weather Station Scenario --");
    Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
    Serial.println("filter off");
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
                      
    // suggested rate is 1/60Hz (1m)
    delayTime = 60000; // in milliseconds
  
}

uint32_t x=0;

void loop() {
  // read from battery
  int reading = analogRead(A0); // current voltage off the sensor
  float voltage = reading * 1; // using 3.3v input
  Serial.println(voltage);
  voltage /= 1024.0; // divide by 1024
  Serial.println(voltage);
  voltage = (voltage * 6.02);
  Serial.print("Read Voltage ");
  Serial.println(voltage);
  Serial.println(millis());

  
  
  // read from BME280 
  float tempC = bme.readTemperature();                // Get temperature from sensor
  Serial.print("Read Temperature ");
  Serial.println(tempC);
  Serial.println(millis());
  float pressure = bme.readPressure();                // Get temperature from sensor
  Serial.print("Read Pressure ");
  Serial.println(pressure);
  Serial.println(millis());
  float humidity = bme.readHumidity();                // Get temperature from sensor
  Serial.print("Read Humidity ");
  Serial.println(humidity);
  Serial.println(millis());


  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();
  
  

  // Now we can publish stuff!
  if (! temp.publish(tempC)) {
  } else {
  }

  if (! Pressure.publish(pressure)) {
  } else {
  }

  if (! Humidity.publish(humidity)) {
  } else {
  }

  if (! battery.publish(voltage)) {
  } else {
  }
  if (! batterystate.publish(chargestate)) {
  } else {
  }
  Serial.print("Published ");
  Serial.println(millis());
  ESP.deepSleep(60e6);
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

//  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 5;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
      Serial.println("MQTT Connecting");
      Serial.println(mqtt.connectErrorString(ret));
       if (retries == 0) {
         Serial.println("MQTT Server Not Found, , , going to sleep");

         // basically die and wait for WDT to reset me
         ESP.deepSleep(60e6);
       }
       Serial.print(".");
       mqtt.disconnect();
       delay(100);  // wait 5 seconds
       retries--;
  }
  Serial.print("MQTT Connected ");
  Serial.println(millis());
}
