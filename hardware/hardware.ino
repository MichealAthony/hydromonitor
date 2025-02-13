//##################################################################################################################
//##                                      ELET2415 DATA ACQUISITION SYSTEM CODE                                   ##
//##                                                                                                              ##
//##################################################################################################################

// LIBRARY IMPORTS
#include <rom/rtc.h> 
#include <math.h>  // https://www.tutorialspoint.com/c_standard_library/math_h.htm 
#include <ctype.h>

// ADD YOUR IMPORTS HERE
#include "DHT.h"
#include <FastLED.h>


#ifndef _WIFI_H 
#include <WiFi.h>
#endif

#ifndef STDLIB_H
#include <stdlib.h>
#endif

#ifndef STDIO_H
#include <stdio.h>
#endif

#ifndef ARDUINO_H
#include <Arduino.h>
#endif 
 
#ifndef ARDUINOJSON_H
#include <ArduinoJson.h>
#endif

 

// DEFINE VARIABLES
#define ARDUINOJSON_USE_DOUBLE      1 

// DEFINE THE CONTROL PINS FOR THE DHT22 
#define DHTPIN    14     // Digital pin connected to the DHT sensor
#define DATA_PIN  33
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define NUM_LEDS 7



// MQTT CLIENT CONFIG  
static const char* pubtopic       = "620161390";                    // Add your ID number here
static const char* subtopic[]     = {"620161390_sub","/elet2415"};  // Array of Topics(Strings) to subscribe to
static const char* mqtt_server    = "broker.emqx.io";                // Broker IP address or Domain name as a String 
static uint16_t mqtt_port         = 1883;

// WIFI CREDENTIALS
const char* ssid                  = "MonaConnect"; // Add your Wi-Fi ssid
const char* password              = ""; // Add your Wi-Fi password 
//const char* ssid                  = "UNTC-Connect"; // Add your Wi-Fi ssid
//const char* password              = "risenlord^19"; // Add your Wi-Fi password 
//const char* ssid                  = "DESKTOP-PJ8NO24 8386"; // Add your Wi-Fi ssid
//const char* password              = "7(3P4i45"; // Add your Wi-Fi password 



// TASK HANDLES 
TaskHandle_t xMQTT_Connect          = NULL; 
TaskHandle_t xNTPHandle             = NULL;  
TaskHandle_t xLOOPHandle            = NULL;  
TaskHandle_t xUpdateHandle          = NULL;
TaskHandle_t xButtonCheckeHandle    = NULL;  

// FUNCTION DECLARATION   
void checkHEAP(const char* Name);   // RETURN REMAINING HEAP SIZE FOR A TASK
void initMQTT(void);                // CONFIG AND INITIALIZE MQTT PROTOCOL
unsigned long getTimeStamp(void);   // GET 10 DIGIT TIMESTAMP FOR CURRENT TIME
void callback(char* topic, byte* payload, unsigned int length);
void initialize(void);
bool publish(const char *topic, const char *payload); // PUBLISH MQTT MESSAGE(PAYLOAD) TO A TOPIC
void vButtonCheck( void * pvParameters );
void vUpdate( void * pvParameters );  
bool isNumber(double number);
 

/* Declare your functions below */ 
double convert_Celsius_to_fahrenheit(double c);
double convert_fahrenheit_to_Celsius(double f);
double calcHeatIndex(double Temp, double Humid);


/* Init class Instances for the DHT22 etcc */
DHT dht(DHTPIN, DHTTYPE);
CRGB leds[NUM_LEDS];

  

//############### IMPORT HEADER FILES ##################
#ifndef NTP_H
#include "NTP.h"
#endif

#ifndef MQTT_H
#include "mqtt.h"
#endif

// Temporary Variables 
double c1 = -42.379, c2= 2.04901523, c3 = 10.14333127, c4 = -0.22475541, c5 = -6.83783 * 0.001, c6 = -5.481717 * 0.01, c7 = 1.22874 * 0.001, c8 = 8.5282 * 0.0001, c9 = -1.99 * 0.000001;

void setup() {
  Serial.begin(115200);  // INIT SERIAL  

  // INITIALIZE ALL SENSORS AND DEVICES
  dht.begin();
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed

  /* Add all other necessary sensor Initializations and Configurations here */
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  initialize();     // INIT WIFI, MQTT & NTP 
  // vButtonCheckFunction(); // UNCOMMENT IF USING BUTTONS INT THIS LAB, THEN ADD LOGIC FOR INTERFACING WITH BUTTONS IN THE vButtonCheck FUNCTION
  
}
  


void loop() {
    // put your main code here, to run repeatedly:
    //Serial.print("Free heap: ");
    //Serial.println(ESP.getFreeHeap());  // Check available heap memory
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);    
}




  
//####################################################################
//#                          UTIL FUNCTIONS                          #       
//####################################################################
void vButtonCheck( void * pvParameters )  {
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );     
      
    for( ;; ) {
        // Add code here to check if a button(S) is pressed
        // then execute appropriate function if a button is pressed  

        vTaskDelay(200 / portTICK_PERIOD_MS);  
    }
}

void vUpdate( void * pvParameters )  {
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );    
           
    for( ;; ) {
          // #######################################################
          // ## This function must PUBLISH to topic every second. ##
          // #######################################################
   
          // 1. Read Humidity and save in variable below
          double h = dht.readHumidity();
           
          // 2. Read temperature as Celsius   and save in variable below
          double t = dht.readTemperature();

          if (isnan(h) || isnan(t)) {
            Serial.println(F("Failed to read from DHT sensor!"));
            return;
          }   
          Serial.print(F("Humidity: "));
          Serial.print(h);
          Serial.print(F("%  Temperature: "));
          Serial.print(t);
          Serial.println(F("°C")); // Ensures a new line after temperature

          Serial.print(F("HeatIndex: "));
          Serial.print(calcHeatIndex(convert_Celsius_to_fahrenheit(t), h));
          Serial.println(F("°C")); // Ensures proper formatting

          if(isNumber(t)){
              // ##Publish update according to ‘{"id": "student_id", "timestamp": 1702212234, "temperature": 30, "humidity":90, "heatindex": 30}’

              // 1. Create JSon object
              JsonDocument doc; // Create JSon object
              
              // 2. Create message buffer/array to store serialized JSON object
              char message[800]   = {0};
              
              // 3. Add key:value pairs to JSon object based on above schema
              doc["id"]           = "620161390"; // Change to your student ID number
              doc["timestamp"]    = getTimeStamp();
              doc["temperature"]  = t;
              doc["humidity"]     = h;
              doc["heatindex"]    = calcHeatIndex(convert_Celsius_to_fahrenheit(t),h);

              // 4. Seralize / Covert JSon object to JSon string and store in message array
              serializeJson(doc, message);  // Seralize / Covert JSon object to JSon string and store in char* array  
               
              // 5. Publish message to a topic sobscribed to by both backend and frontend   
              publish("620161390", message);    // Publish to a topic that both the Backend and the Frontend subscribes to.             

          }

          
            
        vTaskDelay(1000 / portTICK_PERIOD_MS);  
    }
}

 

unsigned long getTimeStamp(void) {
          // RETURNS 10 DIGIT TIMESTAMP REPRESENTING CURRENT TIME
          time_t now;         
          time(&now); // Retrieve time[Timestamp] from system and save to &now variable
          return now;
}


void callback(char* topic, byte* payload, unsigned int length) {
  // ############## MQTT CALLBACK  ######################################
  // RUNS WHENEVER A MESSAGE IS RECEIVED ON A TOPIC SUBSCRIBED TO
  
  Serial.printf("\nMessage received : ( topic: %s ) \n",topic ); 
  char *received = new char[length + 1] {0}; 
  
  for (int i = 0; i < length; i++) { 
    received[i] = (char)payload[i];    
  }

  // PRINT RECEIVED MESSAGE
  Serial.printf("Payload : %s \n",received);

 
  // CONVERT MESSAGE TO JSON
  StaticJsonDocument<1000> doc;
  DeserializationError error = deserializeJson(doc, received);  

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }


  // PROCESS MESSAGE
  const char* type = doc["type"]; 

  if (strcmp(type, "controls") == 0){
    // 1. EXTRACT ALL PARAMETERS: NODES, RED,GREEN, BLUE, AND BRIGHTNESS FROM JSON OBJECT
    uint8_t     brightness    = doc["brightness"];
    uint8_t     l             = doc["leds"];
    uint8_t     r             = doc["color"]["r"];
    uint8_t     g             = doc["color"]["g"];
    uint8_t     b             = doc["color"]["b"];

    // 2. ITERATIVELY, TURN ON LED(s) BASED ON THE VALUE OF NODES. Ex IF NODES = 2, TURN ON 2 LED(s)
    for(int x=0; x<l; x++){  
      leds[x] = CRGB( r, g, b);            
      FastLED.setBrightness( brightness );       
      FastLED.show();  
      vTaskDelay(50 / portTICK_PERIOD_MS);; 
    }

    // 3. ITERATIVELY, TURN OFF ALL REMAINING LED(s).
    for(int x=l; x<NUM_LEDS; x++){  
      leds[x] = CRGB::Black;            
      FastLED.setBrightness( brightness );       
      FastLED.show();  
      vTaskDelay(50 / portTICK_PERIOD_MS);; 
    }
  }
}

bool publish(const char *topic, const char *payload){   
     bool res = false;
     try{
        res = mqtt.publish(topic,payload);
        // Serial.printf("\nres : %d\n",res);
        if(!res){
          res = false;
          throw false;
        }
     }
     catch(...){
      Serial.printf("\nError (%d) >> Unable to publish message\n", res);
     }
  return res;
}



//***** Complete the util functions below ******

double convert_Celsius_to_fahrenheit(double c){    
    // CONVERTS INPUT FROM °C TO °F. RETURN RESULTS  
    return c * (9.0/5.0) + 32;
}

double convert_fahrenheit_to_Celsius(double f){    
    // CONVERTS INPUT FROM °F TO °C. RETURN RESULT  
    return (f - 32) * (5/9); 
}

double calcHeatIndex(double Temp, double Humid){
    // CALCULATE AND RETURN HEAT INDEX USING EQUATION FOUND AT https://byjus.com/heat-index-formula/#:~:text=The%20heat%20index%20formula%20is,an%20implied%20humidity%20of%2020%25
    double T = Temp;
    double R = Humid;
  return  c1 + (c2*T) + (c3*R) + (c4*T*R) + (c5*T*T) + (c6*R*R) + (c7*T*T*R) + (c8*T*R*R) + (c9*T*T*R*R);
}
 

bool isNumber(double number){       
        char item[20];
        snprintf(item, sizeof(item), "%f\n", number);
        if( isdigit(item[0]) )
          return true;
        return false; 
} 
