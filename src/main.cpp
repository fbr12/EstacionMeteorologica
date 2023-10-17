#include <WiFi.h>
#include <ThingSpeak.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include <DHT_U.h>
#include <iostream>
#include <PubSubClient.h>
#include "FS.h"
#include <Wire.h>
using namespace std;

#define pin1 2
#define pin2 
#define BMP_SCK  (22)
#define BMP_MOSI (21)
// Configuracion del servidor Mqtt//
const char *mqttServer = "broker.hivemq.com";
const int mqttPort = 1883;
const char *mqttUser = "Tu_Usuario";
const char *mqttPassword = "Tu_Contraseña";

// Topics MQTT //
const char *humidityTopic = "/ET28/65/RBM/Humidity";
const char *temperatureTopic = "/ET28/65/RBM/Temperature";
const char *pressureTopic = "/ET28/65/RBM/Pressure";
const char *windTopic = "/ET28/65/RBM/Wind";

// Sensor Hall//
const int hallPin = 2; // Pin digital conectado al sensor Hall
volatile unsigned int revolutions = 0;
unsigned long startTime;
unsigned long elapsedTime;
float windSpeedKmh;

// Wifi//
const char *ssid = "BA Escuela";
const char *password = "";
unsigned long channelID = 1332310;
const char *WriteAPIKey = "5SJCDAUGILT6TYMW";
WiFiClient cliente;
PubSubClient client(cliente);

// Sensores de temperatura ,y humedad, y de presion//
DHT dht(pin1, DHT11);
Adafruit_BMP280 bmp;

// Funcion que permite leer el BMP (sensor de presion y altitud)//
void leerBmp()
{
  float seaLevelPressure = 1013.25; 
  float pressure = bmp.readPressure() / 100.0; 
  float altitude = 44330.0 * (1.0 - pow((pressure / seaLevelPressure), 0.1903));
  float temp = bmp.readTemperature();
  Serial.print("Temperatura bmp: ");
  Serial.print(temp);
  Serial.println("°C");
  Serial.print("Presion bmp: ");
  Serial.print(pressure);
  Serial.println("hPa");
  Serial.println("-------------------------");
  ThingSpeak.setField(3, pressure);
}

// Funcion que permite leer el DHT11 (sensor de temperatura y humedad)//
void leerDht1()
{
  float t1 = dht.readTemperature();
  float h1 = dht.readHumidity();
  while (isnan(t1) && isnan(h1))
  {
    Serial.println("lectura incorrecta, repetir");
    delay(2000);
    t1 = dht.readTemperature();
    h1 = dht.readHumidity();
  }
  Serial.print("temperatura DHT11: ");
  Serial.print(t1);
  Serial.println("°C");
  Serial.print("humedad DHT11: ");
  Serial.print(h1);
  Serial.println("%");
  Serial.println("-------------------------");
  ThingSpeak.setField(1, t1);
  ThingSpeak.setField(2, h1);
}

// Funcion que permite contar las revoluciones//
void countRevolutions()
{
  revolutions++;
}

// Funcion que permite calcular la velocidad del viento en km/h//
float calculateWindSpeed(unsigned int revs, unsigned long time)
{
  float circumference = 0.9; // Circunferencia del anemómetro en metros (valor ficticio)
  float windSpeedMps = (revs * circumference) / time;
  float windSpeedKmh = windSpeedMps * 3.6;
  return windSpeedKmh;
}

// Funcion que permite leer el anemometro para utilizarla en el main loop //
void leerAnemometro()
{
  detachInterrupt(digitalPinToInterrupt(hallPin)); // Detener interrupciones durante el cálculo
  windSpeedKmh = calculateWindSpeed(revolutions, elapsedTime);
  Serial.print("Velocidad del viento: ");
  Serial.print(windSpeedKmh);
  Serial.println(" km/h");
  revolutions = 0;
  startTime = millis();
  attachInterrupt(digitalPinToInterrupt(hallPin), countRevolutions, RISING);
  ThingSpeak.setField(4, windSpeedKmh);
}

// Funcion que permite leer el anemometro para utilizarla al enviar los datos al Servidor MQTT //
float leerAnemometro2()
{
  detachInterrupt(digitalPinToInterrupt(hallPin)); // Detener interrupciones durante el cálculo
  windSpeedKmh = calculateWindSpeed(revolutions, elapsedTime);
  revolutions = 0;
  startTime = millis();
  attachInterrupt(digitalPinToInterrupt(hallPin), countRevolutions, RISING);
  return windSpeedKmh;
}

// Funcion para determinar la velocidad del viento en Km/H //
int valorAnemometro()
{
  const int windSpeed = leerAnemometro2();
  return (windSpeed);
}

// Funcion para conectarse al servidor MQTT //
void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword))
    {
      Serial.println("conectado");
    }
    else
    {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" intentando nuevamente en 5 segundos");
      delay(5000);
    }
  }
}

// Funcion para enviar los datos al servidor MQTT //
void enviarDatosMqtt()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  float pressure = bmp.readPressure();
  const int wind = leerAnemometro2();

  if (isnan(humidity) || isnan(temperature))
  {
    Serial.println("Error al leer el sensor DHT11");
    return;
  }

  String presion = String(pressure/100);
  String temperatura = String(temperature);
  String humedad = String(humidity);
  String viento = String(wind);
  client.publish(temperatureTopic, temperatura.c_str());
  client.publish(humidityTopic, humedad.c_str());
  client.publish(pressureTopic, presion.c_str());
  client.publish(windTopic, viento.c_str()); // Enviar datos cada 6 segundos
}

// Iniciación del WIFI, de los sensores y el cliente de ThingSpeak//
void setup()
{
  Serial.begin(9600);
  Serial.println("Conectando a WIFI");
  client.setServer(mqttServer, mqttPort);
  WiFi.begin(ssid, password);
  Wire.begin(21,22);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi conectado");

  ThingSpeak.begin(cliente);
  dht.begin();
  bmp.begin(0x76);

  pinMode(hallPin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(hallPin), countRevolutions, RISING);
  startTime = millis();
}

// Inicio del Programa//
void loop()
{
  delay(2000);
  leerDht1();
  delay(2000);
  leerBmp();
  ThingSpeak.writeFields(channelID, WriteAPIKey);
  enviarDatosMqtt();
  Serial.println("datos enviados");
  delay(14000);
}

