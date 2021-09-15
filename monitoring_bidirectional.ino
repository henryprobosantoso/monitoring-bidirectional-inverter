//========================================DS18b20 variables====================================
#include <OneWire.h>                  //inisiasi library
#include <DallasTemperature.h>        //inisiasi library
#define ONE_WIRE_BUS 4               //definisi pin sensor
OneWire oneWire(ONE_WIRE_BUS);        //pemanggilan fungsi
DallasTemperature sensors(&oneWire);  //pemanggilan fungsi
float inverterTemperature;            //deklarasi variable
//========================================voltage input variables====================================
#define DCVoltageInputPin 36
//========================================current input variables====================================
#define DCCurrentInputPin 32 // define the Arduino pin A0 as voltage input (V in)
const float VCC   = 5.04;// supply voltage
int resistorACS1 = 1000;
int resistorACS2 = 2000;
//========================================Fan variables====================================
#define fanPin 2
#define maxTemperature 25
#define pwmChannel 0
#define pwmResolution 8
#define pwmFrequency 1000
//========================================smoothing variables====================================
const int numReadings = 10;
double readingsDCVoltageInput[numReadings], readingsDCCurrentInput[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
double totalDCVoltageInput, totalDCCurrentInput;                  // the running total
double averageDCVoltageInput, averageDCCurrentInput, averageACVoltageOutput, 
        averageACCurrentOutput, averageACFrequency;                // the average
//========================================time variables====================================
unsigned long previousMillis = 0;
const long interval = 100;
unsigned long previousMillisDataNextion = 0;
const long intervalDataNextion = 1000;
unsigned long previousMillisBlynk = 0;        // will store last time LED was updated
const long intervalBlynk = 1000;       
//========================================nextion variables====================================
#define RXD2 16               //connect to PZEM tx
#define TXD2 17               //connect to PZEM rx
//========================================swith mode variables====================================
#define modeSwitchPin 21               //connect to PZEM tx
//==========================================blynk components==================================
#define BLYNK_TIMEOUT_MS  750  // must be BEFORE BlynkSimpleEsp8266.h doesn't work !!!
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
char ssid[]            = "switch";
char pass[]            = "smartswitch";
char auth[]            = "3n0hupYmcReMrox9xulHS0dhu4eN3W9E";
char server[]          = "blynk-cloud.com";
unsigned int port      = 8080;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // NEXTION
  pinMode(fanPin, OUTPUT);
  pinMode(modeSwitchPin, OUTPUT);
  pinMode(DCVoltageInputPin, INPUT);
  pinMode(DCCurrentInputPin, INPUT);
//  ledcAttachPin(fanPin, pwmChannel);
//  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  sensors.begin();                    // wait for a second
//  blynk
  WiFi.begin(ssid, pass);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readingsDCVoltageInput[thisReading] = 0;
    readingsDCCurrentInput[thisReading] = 0;
  }
}
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    getTemperature();
    Serial.print("Temperature: ");
    Serial.print(inverterTemperature);
    Serial.print("\tVDC Input: ");
    Serial.print(averageDCVoltageInput);
    Serial.print("\tIDC Input: ");
    Serial.println(averageDCCurrentInput);
  }  
  digitalWrite(modeSwitchPin, HIGH);    
  if(inverterTemperature > maxTemperature){
    digitalWrite(fanPin, LOW);    
  }
  getDCVoltageInput();
  sendDataNextion();
}
void sendDataNextion() {
  String command;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillisDataNextion >= intervalDataNextion) {
    previousMillisDataNextion = currentMillis;
    command = "t0.txt=\"" + String(averageDCVoltageInput) + "\"";
    Serial2.print(command);
    endNextionCommand();
    command = "t1.txt=\"" + String(averageDCCurrentInput) + "\"";
    Serial2.print(command);
    endNextionCommand();
    command = "t2.txt=\"" + String(averageACVoltageOutput) + "\"";
    Serial2.print(command);
    endNextionCommand();
    command = "t3.txt=\"" + String(averageACCurrentOutput) + "\"";
    Serial2.print(command);
    endNextionCommand();
    command = "t4.txt=\"" + String(averageACFrequency) + "\"";
    Serial2.print(command);
    endNextionCommand();
    command = "t5.txt=\"" + String(inverterTemperature) + "\"";
    Serial2.print(command);
    endNextionCommand();
  }
  if(WiFi.status()!=WL_CONNECTED){
    WiFi.disconnect();
    WiFi.reconnect();
    command = "vis p1,1";
    Serial2.print(command);
    endNextionCommand();
  }else{
    command = "vis p1,0";
    Serial2.print(command);
    endNextionCommand();
  }
}
void endNextionCommand()
{
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
}
void getTemperature(){
  sensors.requestTemperatures(); // Send the command to get temperatures
  inverterTemperature = sensors.getTempCByIndex(0);
}
void getDCVoltageInput() {
  totalDCVoltageInput = totalDCVoltageInput - readingsDCVoltageInput[readIndex];
  readingsDCVoltageInput[readIndex] = analogRead(DCVoltageInputPin);
  totalDCVoltageInput = totalDCVoltageInput + readingsDCVoltageInput[readIndex];
  
  totalDCCurrentInput = totalDCCurrentInput - readingsDCCurrentInput[readIndex];
  readingsDCCurrentInput[readIndex] = getDCCurrentInput();
  totalDCCurrentInput = totalDCCurrentInput + readingsDCCurrentInput[readIndex];
  
  readIndex = readIndex + 1;
  if (readIndex >= numReadings){
    readIndex = 0;
  }
  
  averageDCVoltageInput = totalDCVoltageInput / numReadings;  
  averageDCCurrentInput = totalDCCurrentInput / numReadings;  
  
  if(averageDCVoltageInput > 0){
    averageDCVoltageInput = averageDCVoltageInput * 3.3 / 4095;  
    averageDCVoltageInput = -0.009*averageDCVoltageInput*averageDCVoltageInput
                          +18.996*averageDCVoltageInput+2.9177;
  }
}
float getDCCurrentInput(){
  float FACTOR = 40.0/1000;// set sensitivity for selected model
  const float QOV    = 0.5 * VCC;// set quiescent Output voltage for selected model
  float voltage;// internal variable for voltage
  float voltage_raw = (3.3 / 4095.0)* analogRead(DCCurrentInputPin);  
  voltage_raw = voltage_raw / 0.667;
  voltage =  voltage_raw - QOV;
  voltage = abs(voltage / FACTOR) - (120.0/1000);
  return voltage;
}
