#include "FS.h"
#include <DHT.h>
#include <WiFi.h>
#include <Wire.h>
#include <DHT_U.h>
#include <iostream>
#include <Arduino.h>
#include <PubSubClient.h>
#include <Adafruit_BMP280.h>

using namespace std;

#define pin1 2
#define pin2
#define BMP_SCK (22)
#define BMP_MOSI (21)
#define SensorDeLluvia (32)

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
const char *rainTopic = "/ET28/65/RBM/Rain";

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

// Sensores de temperatura ,y humedad, y de presion atmosferica//
DHT dht(pin1, DHT11);
Adafruit_BMP280 bmp;

// Definicion de Pines y variables para el Anemometro //
const int sensorAnemometroPin = 3;    // Pin conectado al sensor del anemómetro
volatile int contadorVueltas = 0;     // Contador de vueltas
unsigned long tiempoAnterior = 0;     // Tiempo de la última medición
const float distanciaPorVuelta = 1.0; // Distancia recorrida por cada vuelta del anemómetro (en metros)

void contadorVuelta() // Funcion que permite llevar la cuenta de las vueltas//
{
  contadorVueltas++;
}

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
}

// Funcion que permite leer el DHT11 (sensor de temperatura y humedad)//
void leerDht11()
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
}

// Funcion que permite leer el anemometro para utilizarla en el main loop //
float leerAnemometro()
{
  unsigned long tiempoActual = millis();

  if (tiempoActual - tiempoAnterior >= 5000)
  {                                                                      // Muestra la velocidad cada 5 segundos
    float tiempoTranscurrido = (tiempoActual - tiempoAnterior) / 1000.0; // En segundos
    float revolucionesPorSegundo = contadorVueltas / tiempoTranscurrido;
    float velocidadMetrosPorSegundo = revolucionesPorSegundo * distanciaPorVuelta;
    float velocidadKmPorHora = velocidadMetrosPorSegundo * 3.6;

    Serial.print("Velocidad del viento: ");
    Serial.print(velocidadKmPorHora);
    Serial.println(" km/h");

    tiempoAnterior = tiempoActual;
    contadorVueltas = 0;
    return velocidadKmPorHora;
  }
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
string howMuchRain()
{
  int valorSensorLluvia = analogRead(SensorDeLluvia);
  int PocaLluvia = 3000;
  int MuchaLluvia = 2300;
  Serial.println(valorSensorLluvia);
  if (valorSensorLluvia > PocaLluvia)
  {
    return "No está lloviendo.";
  }
  else if (valorSensorLluvia > MuchaLluvia && valorSensorLluvia < PocaLluvia)
  {
    return "Está lloviendo un poco.";
  }
  else if (isnan(valorSensorLluvia))
  {
    return "Error al leer sensor de lluvia";
  }
  else if (valorSensorLluvia < MuchaLluvia)
  {
    return "Esta Lloviendo mucho";
  }

  return "No hay datos disponibles, revise el sensor";
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
  float pressure = bmp.readPressure() / 100.0;
  const int wind = leerAnemometro();
  string rain = howMuchRain();

  if (isnan(humidity) || isnan(temperature))
  {
    Serial.println("Error al leer el sensor DHT11");
    return;
  }

  String presion = String(pressure);
  String temperatura = String(temperature);
  String humedad = String(humidity);
  String viento = String(wind);
  client.publish(rainTopic, rain.c_str());
  client.publish(temperatureTopic, temperatura.c_str());
  client.publish(humidityTopic, humedad.c_str());
  client.publish(pressureTopic, presion.c_str());
  client.publish(windTopic, viento.c_str()); // Enviar datos cada 6 segundos
}
void leerSensorDeLluvia()
{
  int valorSensorLluvia = analogRead(SensorDeLluvia);
  int PocaLluvia = 3000;
  int MuchaLluvia = 2300;
  Serial.println(valorSensorLluvia);
  if (valorSensorLluvia > PocaLluvia)
  {
    Serial.println("No está lloviendo.");
  }
  else if (valorSensorLluvia > MuchaLluvia && valorSensorLluvia < PocaLluvia)
  {
    Serial.println("Está lloviendo un poco.");
  }
  else if (isnan(valorSensorLluvia))
  {
    Serial.println("Error al leer sensor de lluvia");
  }
  else if (valorSensorLluvia < MuchaLluvia)
  {
    Serial.println("Esta Lloviendo mucho");
  }
}

// Iniciación del WIFI, de los sensores y del cliente MQTT//
void setup()
{
  Serial.begin(9600);
  Serial.println("Conectando a WIFI");
  client.setServer(mqttServer, mqttPort);
  WiFi.begin(ssid, password);
  Wire.begin(21, 22);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi conectado");

  dht.begin();
  bmp.begin(0x76);
  pinMode(sensorAnemometroPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorAnemometroPin), contadorVuelta, FALLING);
}

// Inicio del Programa//
void loop()
{
  delay(1000);
  leerDht11();
  delay(1000);
  leerBmp();
  delay(1000);
  leerSensorDeLluvia();
  delay(1000);
  leerAnemometro();
  enviarDatosMqtt();
  Serial.println("datos enviados");
  delay(6000);
}
