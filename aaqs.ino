
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266Influxdb.h>
#include <ArduinoJson.h>
#include <SHT21.h>


#include <FS.h>
#include <WiFiUdp.h>
//#include <arduino.h>
//------CONSTANTS----------//
#define SLEEP_INTERVAL_MICROSECONDS 10000000ul//300e6


//------PIN DEFS---------//
#define TRIGGER_CONTROL_A_PIN 5
#define TRIGGER_CONTROL_B_PIN 4
#define EN_SENSE_PIN 12

#define SDA 2
#define SCL 14


//------PCA9557-DEFS---------//
#define PCA9557_I2C_ADDRESS 0x30
#define PCA9557_OUTPUT_REG 0x01
#define PCA9557_INPUT_REG 0x00
#define PCA9557_POLARITYINVERSION_REG 0x02
#define PCA9557_CONFIG_REG 0x03

#define DIVIDER_NH3_UP_PIN 0x80
#define DIVIDER_NH3_DOWN_PIN 0x40

#define DIVIDER_CO_UP_PIN 0x20
#define DIVIDER_CO_DOWN_PIN 0x10

#define DIVIDER_NO2_UP_PIN 0x08
#define DIVIDER_NO2_DOWN_PIN 0x04
#define GP2Y1010A_LED_PIN 0x02
#define STATUS_LED_PIN 0x01
//------GP2Y1010A-DEFS---------//



//InfluxDB Server

const char *DATABASE = "temp";
const char *DB_USER = "airdroplet";
const char *DB_PASSWORD = "VuRNqkK2YYftHhCQGKQE";

const char *INFLUXDB_SERVER = "influx.createyour.cloud";
uint16_t INFLUXDB_PORT = 80;       // Default InfluxDB UDP Port

char INFLUXDB_HOST[40];             // Your InfluxDB Server FQDN

char SENSOR_LOCATION[20] = "test";    // This location is used for the "device=" part of the InfluxDB update
ESP8266WiFiMulti WiFiMulti;
Influxdb influxdb (INFLUXDB_SERVER, INFLUXDB_PORT);

SHT21 sht;


Adafruit_ADS1015 ads1015;  	// Construct an ads1015 at the default address: 0x48

//WiFiUDP udp;

enum {NH3, CO, NO2};

/* struct {uint16_t */
/* }sensorData; */

void measureMICS(float *results){
  // we need to set the voltage divider via io-expander
  uint8_t currentSetting;
  uint8_t dataOut = 0;
  char* response[3];
  uint16_t adc_vals[3];
  
  Wire.beginTransmission(PCA9557_I2C_ADDRESS);
  Wire.write(PCA9557_CONFIG_REG);
  currentSetting = Wire.read();
  Wire.endTransmission();
  // first set smallest scaling factor
  // R_prescale is maxed to 100106.8kOhm
  // by disconnecting PINs by setting as input
  
  currentSetting |= DIVIDER_NH3_UP_PIN
    | DIVIDER_NH3_DOWN_PIN
    | DIVIDER_CO_UP_PIN
    | DIVIDER_CO_DOWN_PIN
    | DIVIDER_NO2_UP_PIN
    | DIVIDER_NO2_DOWN_PIN;
  
  Wire.beginTransmission(PCA9557_I2C_ADDRESS);
  Wire.write(PCA9557_CONFIG_REG);
  Wire.write(currentSetting);
  Wire.endTransmission();

 
  adc_vals[NH3] = ads1015.readADC_SingleEnded(NH3);
  adc_vals[CO]  = ads1015.readADC_SingleEnded(CO);
  adc_vals[NO2] = ads1015.readADC_SingleEnded(NO2);

  //Serial.println("1 NH3 :"+String(adc_vals[NH3]));
 Serial.print(adc_vals[0]);
 Serial.print(adc_vals[1]);
 Serial.println(adc_vals[2]);
    
  delay(200);


  // R_prescale to 106.8kOhm
  // setting up pins as outputs and high
  currentSetting &= ~(DIVIDER_NH3_UP_PIN
		      | DIVIDER_CO_UP_PIN
		      | DIVIDER_NO2_UP_PIN);
  
  Wire.beginTransmission(PCA9557_I2C_ADDRESS);
  Wire.write(PCA9557_CONFIG_REG);
  Wire.write(currentSetting);
  Wire.endTransmission();

  dataOut |= DIVIDER_NH3_UP_PIN
    | DIVIDER_CO_UP_PIN
    | DIVIDER_NO2_UP_PIN;
  
  Wire.beginTransmission(PCA9557_I2C_ADDRESS);
  Wire.write(PCA9557_OUTPUT_REG);
  Wire.write(dataOut);
  Wire.endTransmission();
  

  adc_vals[NH3] = ads1015.readADC_SingleEnded(NH3);
  adc_vals[CO]  = ads1015.readADC_SingleEnded(CO);
  adc_vals[NO2] = ads1015.readADC_SingleEnded(NO2);
 Serial.print(adc_vals[0]);
 Serial.print(adc_vals[1]);
 Serial.println(adc_vals[2]);  
  delay(200);
  

  // R_prescale to 6.8kOhm
  // setting lower pins of voltage divider to outputs and high
  currentSetting |= DIVIDER_NH3_UP_PIN
    | DIVIDER_CO_UP_PIN
    | DIVIDER_NO2_UP_PIN;

  currentSetting &=  ~(DIVIDER_NH3_DOWN_PIN
		       | DIVIDER_CO_DOWN_PIN
		       | DIVIDER_NO2_DOWN_PIN);

  dataOut = DIVIDER_NH3_DOWN_PIN
    | DIVIDER_CO_DOWN_PIN
    | DIVIDER_NO2_DOWN_PIN;

    
  Wire.beginTransmission(PCA9557_I2C_ADDRESS);
  Wire.write(PCA9557_CONFIG_REG);
  Wire.write(currentSetting);
  Wire.endTransmission();

  Wire.beginTransmission(PCA9557_I2C_ADDRESS);
  Wire.write(PCA9557_OUTPUT_REG);
  Wire.write(dataOut);
  Wire.endTransmission();
  

  adc_vals[NH3] = ads1015.readADC_SingleEnded(NH3);
  adc_vals[CO]  = ads1015.readADC_SingleEnded(CO);
  adc_vals[NO2] = ads1015.readADC_SingleEnded(NO2);
   Serial.print(adc_vals[0]);
 Serial.print(adc_vals[1]);
 Serial.println(adc_vals[2]);
  delay(200);
 
  //  Resistance can be measured with multiple prescalers
  // we need to find out the best performing scaling factor and use that value
  // U_measure = R_s/(R_s+R_prescale) * U_in
  // R_s = U_measure / U_in * (R_s + R_prescale)
  // R_s = U_measure / U_in * R_s +  U_measure / U_in *R_prescale
  // R_s*(1 - U_measure/U_in) = U_measure/U_in * R_prescale
  // R_s = (U_measure/U_in *R_prescale) / (1 - U_measure/U_in)
  // R_s = (R_prescale / U_in) / (1/U_measure - 1/U_in)
  
  // R_s = R_









  // R_prescale *(1/U_measure - 1/U_in) / U_in
  // R_s = R_prescale * (1/(U_measure*U_in) - 1/U_in^2)
  // U_in = 5V, U_measure = ADC/1024*U_in
  // R_s = R_prescale * (1/(ADC/1024 *U_in) - 1/U_in) / U_in

  
  // R_s = R_prescale * (1/U_in*(1024/ADC - 1)) *1/U_in
  // R_s = R_presclae * (ADC/1024 -1)/U_in^2
  //
  // U_in = 5V
  // R_s = (6.8kOhm | 100kOhm | 100MOhm) * (1024/ADC - 1) / 25
  // R_s = 
  // we should have best accuracy with the biggest ADC-value!
 
  // U_measure = (R_s/R_prescale)

  /* //---NH3-settings---// */
  /* // smaller than 25% of max adc value? */
  /* if (adc_vals[NH3] < 1024) { */
  /*   //    currentSetting &= ~() : */
  /*   results[NH3] = 3*rand(); */
  /* } */
  /* // < 50%? */
  /* else if (adc_vals[NH3] < 2048) { */
  /*   results[NH3] = 2*rand(); */
  /* } */
  /* else { */
  /*   results[NH3] = rand(); */
  /* } */
  /* //---CO-settings---// */
  /* // smaller than 25% of max adc value? */
  /* if (adc_vals[CO] < 1024) { */
  /*   //    currentSetting &= ~() : */
  /*   results[CO] = 3*rand(); */
  /* } */
  /* // < 50%? */
  /* else if (adc_vals[CO] < 2048) { */
  /*   results[CO] = 2*rand(); */
  /* } */
  /* else { */
  /*   results[CO] = rand(); */
  /* } */

  /* //---NO2-settings---// */
  /* // smaller than 25% of max adc value? */
  /* if (adc_vals[NO2] < 1024) { */
  /*   //    currentSetting &= ~() : */
  /*   results[NO2] = 3*rand(); */
  /* } */
  /* // < 50%? */
  /* else if (adc_vals[NO2] < 2048) { */
  /*   results[NO2] = 2*rand(); */
  /* } */
  /* else { */
  /*   results[NO2] = rand(); */
  /* } */
  
}

//WiFiServer server(80);

//ESP8266WebServer server(80);






/* uint16_t read_dust_sensor(){} */

/* void turn_on_5V_regulator(){} */
/* void turn_off_5V_regulator(){} */


void read_temperature_humidity(float *temp_hum){
  Serial.println("reading temperature");  
  temp_hum[0] = sht.getTemperature();
  Serial.println(temp_hum[0]);
  Serial.println("reading humidity");
  temp_hum[1] = sht.getHumidity();
  Serial.println(temp_hum[1]);
}


/* String create_response(int16_t no3, in16_t co, int16_t voc, int16_t temp, int16_t rh){ */
/*   //  creates a json response string cointaining all sensor values */

  
/* } */


boolean new_ap_setting_request(){
  // when control button is pressed, we want to set the module to an accesspoint
  return true;
}



void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
  Serial.println(myWiFiManager->getConfigPortalSSID());
}


//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  Serial.println(WiFi.psk());
  Serial.println(WiFi.SSID());
  shouldSaveConfig = true;
}

// TP_link 97721666

void setup(){
  //pinMode(TRIGGER_AP_MODE_PIN, INPUT);
  pinMode(EN_SENSE_PIN, OUTPUT);
  digitalWrite(EN_SENSE_PIN, HIGH);

  Serial.begin(9600);
  Wire.begin(SDA,SCL);
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
    
  //WiFiManagerParameter custom_influxdb_server("server", "InfluxDB Server", INFLUXDB_HOST, 40);
  //WiFiManagerParameter custom_influxdb_port("port", "8089", INFLUXDB_PORT, 5);
  //WiFiManagerParameter custom_sensor_location("location", "Location", SENSOR_LOCATION, 6);

  //wifiManager.addParameter(&custom_influxdb_server);
  //wifiManager.addParameter(&custom_influxdb_port);
  //wifiManager.addParameter(&custom_sensor_location);

  if(!wifiManager.autoConnect("AirDroplet", "airquality101")){
    Serial.println("did not connect :( ...resetting.");
    ESP.reset();
  }
  
  Serial.println("connected...yeey :)");
  unsigned char ssid[40];
  unsigned char passw[40];
  WiFi.SSID().getBytes(ssid,40,0);
  WiFi.psk().getBytes(passw,40,0);
  
  WiFiMulti.addAP((const char*)ssid,(const char*)passw);
  while (WiFiMulti.run() != WL_CONNECTED) {
    delay(100);
  }
  Serial.println("Ready");
  influxdb.opendb(DATABASE, DB_USER, DB_PASSWORD);
  
  Serial.println(WiFi.psk());
  Serial.println(WiFi.SSID());
   
  ads1015.begin();  // Initialize ads1015
  Serial.println(SDA);
  Serial.println(SCL);
  
}
  

  


void loop(){
  // is configuration portal requested?
  /* if ( digitalRead(TRIGGER_AP_MODE_PIN) == LOW ) { */
  
    
  /* } */


 /* get temperature and humidity */
   float temp_hum[2];
   read_temperature_humidity(temp_hum);

   //float mics_data[3];

  //digitalWrite(EN_SENSE_PIN, HIGH);
  //measureMICS(mics_data);
  //digitalWrite(EN_SENSE_PIN, LOW);
  //delay(500);
  // Writing data using FIELD object
  // Create field object with measurment name=analog_read
  //FIELD dataObj("temperature");
  //dataObj.addField("t", temp_hum[0]); // Add value field
  //dataObj.addField("rel_hum", temp_hum[1]); // Add value field
  //dataObj.addField("NO2", mics_data[NO2]); // Add value field
  //dataObj.addField("CO", mics_data[CO]); // Add value field
  //dataObj.addField("NH3", mics_data[NH3]); // Add value field
  //Serial.println(dataObj.postString());
  //Serial.println(influxdb.write(dataObj) == DB_SUCCESS ? "Object write success"
  // : "Writing failed");

  // Empty field object.
  //dataObj.empty();



  /* // Writing data with influxdb HTTP API */
  /* // https://influxdb.com/docs/v0.9/guides/writing_data.html */
  /* Serial.println("Writing data to host " + String(INFLUXDB_HOST) + ":" + */
  /* 		 INFLUXDB_PORT + "'s database=" + DATABASE); */
  /* String data = "temperature t=" + String(3.0+rand()); */
  /* influxdb.write(data); */
  /* Serial.println(influxdb.response() == DB_SUCCESS ? "HTTP write success" */
  /* 		 : "Writing failed"); */

   
  //ESP.deepSleep(SLEEP_INTERVAL_MICROSECONDS);
  //ESP.deepSleep(1000000,WAKE_NO_RFCAL);
  //system_deep_sleep_set_option(2);
  //system_deep_sleep_instant(SLEEP_INTERVAL_MICROSECONDS);
  delay(3000);


  
  /* int16_t dust, nh3, co, no2; */
  /* nh3 = ads1015.readADC_SingleEnded(0); */
  /* co1 = ads1015.readADC_SingleEnded(1); */
  /* no2 = ads1015.readADC_SingleEnded(2); */
  /* dust = ads1015.readADC_SingleEnded(3); */
}

