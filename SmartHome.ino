/* This is a smart home project that can be implemented in one room. 
 * In this project, motion sensor, LDR module, gas quality sensor, buzzer, 
 * temperature and humidity sensor, relay and DOIT ESP32 DEVKIT V1 is used.  
 * Data is transferred to Blynk app and light is controlled by this app, also 
 * light is on when motion is detected if room is dark. 
 Made by Alperen Güleç */
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>

#define BLYNK_PRINT Serial 
#define timeSeconds 300
#define LDR 35
#define mq135 32
#define Buzzer 25
#define DHTPIN 33    
#define RELAYPIN 26
#define DHTTYPE DHT11
const int motionSensor = 27;
float data = 0; 
float data100 = 0 ;

// Timer: Auxiliary variables.
unsigned long now = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;

// Auth Token in the Blynk App.
char auth[] = "s8R17w7R4T56SqxfQaa--gsXkPZyUok2";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "TTNET_TP-LINK_3509";
char pass[] = "2pAXaNXx";

void IRAM_ATTR detectsMovement() {
  // Checks if motion was detected, room is dark, and startTimer is 
  // false(RELAYPIN is HIGH), sets RELAYPIN LOW and starts a timer.
  if(analogRead(LDR)==4095 && startTimer==false ){
  Serial.println("1MOTION DETECTED!!!");
  digitalWrite(RELAYPIN, LOW);
  startTimer = true;
  lastTrigger = millis();
  }
  // Checks if motion was detected, and startTimer is 
  // true(RELAYPIN is LOW), sets RELAYPIN LOW and starts a timer,
  // in other words when the light is on if motion is detected, starts timer again. 
  else if (startTimer==true){
  Serial.println("2MOTION DETECTED!!!");
  digitalWrite(RELAYPIN, LOW);
  startTimer = true;
  lastTrigger = millis();
  }
}

DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;

// This function sends ESP32's up time every second to Virtual Pin (5).
void sendSensor()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // You can send any value at any time.
  Blynk.virtualWrite(V5, h);
  Blynk.virtualWrite(V6, t);
}

void getSendData()
{
data100 = analogRead(mq135);
data = (data100 * 100)/4095 ;
  Blynk.virtualWrite(V3, data); // Virtual pin V3.
  // If air pollution is above 11% level (smoke, toxic gases or alcohol is detected)
  // buzzer will alarm.
  if (data > 11 )
  {
    Blynk.notify("Smoke Detected!!!"); 
    digitalWrite(Buzzer, HIGH);
  }
  if (data < 11 )
  {
    digitalWrite(Buzzer, LOW); 
  }
  Serial.println(digitalRead(Buzzer));
  delay(1000);
}

// This part is used to control relay with Blynk app.
BLYNK_CONNECTED()
{
  if (digitalRead(RELAYPIN) == 1) Blynk.virtualWrite(V2, digitalRead(RELAYPIN));
  else Blynk.syncVirtual(V2);
}

BLYNK_WRITE(V2)
{
  int pinData = param.asInt();
  digitalWrite(RELAYPIN, pinData);
}

void setup()
{
  // Debug console.
  Serial.begin(9600);
  pinMode(RELAYPIN, OUTPUT);
  pinMode(mq135, INPUT);
  pinMode(Buzzer, OUTPUT); 
  pinMode(motionSensor, INPUT_PULLUP);
  Blynk.begin(auth, ssid, pass);
  dht.begin();

  // Setup a function to be called every second.
  timer.setInterval(1000L, sendSensor);
  timer.setInterval(1000L, getSendData);
  
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode.
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);

  // Set LIGHT to LOW.
  digitalWrite(RELAYPIN, HIGH);
}

void loop()
{
  // Here we check if there is movement.
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);
  Blynk.run();
  timer.run();
  // Current time.
  now = millis();
  // Turn off the LIGHT after the number of seconds defined
  // in the timeSeconds variable if no motion is detected.
  if(startTimer && (now - lastTrigger > (timeSeconds * 1000))) {
    Serial.println("Motion stopped...");
    digitalWrite(RELAYPIN, HIGH);
    startTimer = false;
  }
}
