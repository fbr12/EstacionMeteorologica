#include <ESP8266WiFi.h>
#include <ThingSpeak.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include <DHT_U.h>
#include <iostream>
using namespace std; 

#define pin1 13

const char* ssid= "BA ESCUELA"; 
const char* password= "";
unsigned long channelID= 1332310; 
const char* WriteAPIKey= "5SJCDAUGILT6TYMW";
WiFiClient cliente; 
DHT dht1(pin1, DHT11); 
Adafruit_BMP280 bmp;

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
}

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

void setup() {
  pinMode(D3, OUTPUT);
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
}

void loop() {
  digitalWrite(D3, HIGH);
  delay(1000);
  digitalWrite(D3, LOW);
  delay(1000);
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
  ThingSpeak.setField(3,temp); 
  ThingSpeak.setField(4,presion);
  ThingSpeak.setField(5,altitud);
} 
