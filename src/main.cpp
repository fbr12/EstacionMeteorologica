#include <WiFi.h>
#include <ThingSpeak.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include <DHT_U.h>
#include <iostream>
using namespace std; 

#define pin1 13

//Sensor Hall//
const int hallPin = 2; // Pin digital conectado al sensor Hall
volatile unsigned int revolutions = 0;
unsigned long startTime;
unsigned long elapsedTime;
float windSpeedKmh;

//Wifi//
const char* ssid= "BA ESCUELA"; 
const char* password= "";
unsigned long channelID= 1332310; 
const char* WriteAPIKey= "5SJCDAUGILT6TYMW";
WiFiClient cliente; 

//Sensores de temperatura ,y humedad, y de presion//
DHT dht1(pin1, DHT11); 
Adafruit_BMP280 bmp;

//Funcion que permite leer el BMP (sensor de presion y altitud)//
void leer_bmp(){
float temp= bmp.readTemperature(); 
float presion= bmp.readPressure();
float altitud= bmp.readAltitude();
Serial.print("Temperatura bmp: "); 
Serial.print(temp);
Serial.println("°C");
Serial.print("Presion bmp: ");
Serial.print(presion);
Serial.println("hPa");
Serial.print("Altitud bmp: ");
Serial.print(altitud);
Serial.println("Metros");
Serial.println("-------------------------");
ThingSpeak.setField(3,presion);
}

//Funcion que permite leer el DHT11 (sensor de temperatura y humedad)//
void leer_dht1(){
float t1= dht1.readTemperature(); 
float h1= dht1.readHumidity();
while (isnan(t1) || isnan(h1)){ 
Serial.println("lectura incorrecta, repetir");
delay (2000);
t1= dht1.readTemperature();
h1= dht1.readHumidity();
}
Serial.print("temperatura DHT11: ");
Serial.print(t1);
Serial.println("°C");
Serial.print("humedad DHT11: ");
Serial.print(h1);
Serial.println("%");
Serial.println("-------------------------");
ThingSpeak.setField(1,t1);
ThingSpeak.setField(2,h1);
}

//Funcion que permite contar las revoluciones//
void countRevolutions() {
  revolutions++;
}

//Funcion que permite calcular la velocidad del viento en km/h//
float calculateWindSpeed(unsigned int revs, unsigned long time) {
  float circumference = 0.1; // Circunferencia del anemómetro en metros (valor ficticio)
  float windSpeedMps = (revs * circumference) / time;
  float windSpeedKmh = windSpeedMps * 3.6;
  return windSpeedKmh;
}

//Iniciación del WIFI y el cliente de ThingSpeak//
void setup() {
  Serial.begin(115200); 
  Serial.println("Conectando a WIFI");
  WiFi.begin(ssid,password); 
  while(WiFi.status() !=WL_CONNECTED){ 
  delay(500);
  Serial.print(".");
  } 
  Serial.println("WiFi conectado");

  ThingSpeak.begin(cliente); 
  dht1.begin();
  bmp.begin(0x76);

  pinMode(hallPin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(hallPin), countRevolutions, RISING);
  startTime = millis();
}

//Inicio del Programa//
void loop() {
  elapsedTime = millis() - startTime;
  if (elapsedTime >= 1000) { // Calcular velocidad cada 1 segundo
    detachInterrupt(digitalPinToInterrupt(hallPin)); // Detener interrupciones durante el cálculo
    windSpeedKmh = calculateWindSpeed(revolutions, elapsedTime);
    Serial.print("Velocidad del viento: ");
    Serial.print(windSpeedKmh);
    Serial.println(" km/h");
    revolutions = 0;
    startTime = millis();
    attachInterrupt(digitalPinToInterrupt(hallPin), countRevolutions, RISING);
    ThingSpeak.setField(4,windSpeedKmh);
  }    
  float temp= bmp.readTemperature(); 
  float presion= bmp.readPressure();
  float altitud= bmp.readAltitude();
  delay(2000);
  leer_dht1();
  delay(2000);
  leer_bmp();
  ThingSpeak.writeFields(channelID,WriteAPIKey); 
  Serial.println("datos enviados");
  delay(14000);
}
