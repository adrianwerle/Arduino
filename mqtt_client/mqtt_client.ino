#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Detalles de la red Wi-Fi
const char* ssid = "VPA2";                 // SSID
const char* password = "V@1ent1n@2OO3";    // Contraseña WiFi

// Detalles del broker MQTT
const char* mqttServer = "lnxsvr2";   // Dirección IP del broker MQTT
const int mqttPort = 1883;                 // Puerto del broker MQTT
const char* mqttUser = "usuario1";         // Usuario MQTT
const char* mqttPassword = "clave1";       // Contraseña MQTT

// Nombre de la placa
const char* nameBoard = "mkr-1";

// Asegúrate de que la dirección I2C es correcta (comúnmente 0x27 o 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Crear las instancias necesarias
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WiFiUDP udp; // Crea un objeto para el UDP
NTPClient timeClient(udp, "pool.ntp.org", -10800, 60000); // Servidor NTP, zona horaria (UTC), intervalo de actualización (60 seg)

unsigned long previousMillis = 0; // Almacena el tiempo anterior para el mensaje heartbeat
const long interval = 60000; // Intervalo de tiempo de 10 segundos

unsigned long previousDataMillis = 0; // Almacena el tiempo anterior para el mensaje de datos
const long dataInterval = 10000; // Intervalo de tiempo para el mensaje de datos (10 segundos)

String uid;

void setup() {
  Wire.begin();

  Serial.begin(9600);

  lcd.init();
  lcd.backlight(); // Activa la luz de fondo del LCD
  lcd.clear();
  lcd.setCursor(0, 0); // Coloca el cursor en la columna 0, fila 0
  lcd.print("Hola Mundo!");
  lcd.setCursor(0, 1); // Coloca el cursor en la columna 0, fila 1
  lcd.print("Arduino MKR WiFi");

  connectToWiFi();
  
  // Configura el servidor MQTT
  mqttClient.setServer(mqttServer, mqttPort);

  // Obtén la dirección MAC
  uint8_t mac[6]; // Arreglo para almacenar la dirección MAC
  WiFi.macAddress(mac);

  for (int i = 0; i < 6; i++) {
    // Convierte el octeto a hexadecimal
    String octet = String(mac[i], HEX); 
    // Asegúrate de que tenga dos caracteres, añadiendo un '0' al principio si es necesario
    if (octet.length() == 1) {
      octet = "0" + octet; // Completa con '0' si solo tiene un dígito
    }
    uid += octet; // Añade el octeto formateado a uid
  }
  uid.toUpperCase();

}

void loop() {

  // Asegúrate de que el cliente está conectado
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  
  mqttClient.loop(); // Mantiene la conexión

  // Actualiza la hora desde el servidor NTP
  timeClient.update();

  // Envía mensaje heartbeat cada 1 minuto
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sendHeartbeat();
  }

    // Envía mensaje DATA cada 10 segundos
  if (currentMillis - previousDataMillis >= dataInterval) {
    previousDataMillis = currentMillis;
    sendData();
  }

}

void connectToWiFi() {
  Serial.print("Conectando a WiFi...");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  Serial.println("Conectado a WiFi.");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
  // Reintenta conectarse al broker MQTT
  while (!mqttClient.connected()) {
    Serial.print("Intentando conectar al MQTT...");

    // Intento de conexión
    if (mqttClient.connect("ClientID", mqttUser, mqttPassword)) {
      Serial.println("conectado.");
    } else {
      Serial.print("Error en conexión MQTT, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" intentando de nuevo en 2 segundos.");
      delay(2000);
    }
  }
}

void sendHeartbeat() {
  // Obtiene el timestamp actual
  unsigned long epochTime = timeClient.getEpochTime(); // Obtiene la hora formateada
  // Convierte el epochTime a un String y concatena '000'
  String timestampString = String(epochTime) + "000";

  char message[150]; // Buffer para el mensaje

  // Obtiene la dirección IP
  IPAddress ip = WiFi.localIP();
  String ipString = String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);

  // Obtiene el valor RSSI
  long rssi = WiFi.RSSI();

  // Crea el mensaje con el timestamp
  snprintf(message, sizeof(message), 
           "{\"timestamp\": \"%s\", \"name\": \"%s\", \"rssi\": \"%ld\", \"ip\": \"%s\"}", 
           timestampString.c_str(), nameBoard, rssi, ipString.c_str());
  
  // Construye el tópico con el UID
  String topic = "HEARTBEAT/" + uid;

  // Publica en el topic "heartbeat"
  mqttClient.publish(topic.c_str(), message); 
  Serial.print("Mensaje enviado: ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(message);
}

void sendData() {
  // Obtiene el timestamp actual
  unsigned long epochTime = timeClient.getEpochTime(); // Obtiene la hora formateada
  // Convierte el epochTime a un String y concatena '000'
  String timestampString = String(epochTime) + "000";

  char message[150]; // Buffer para el mensaje

  // Genera un valor aleatorio entre 0 y 100
  String randomValue = String(random(0, 101)); // El rango es 0 a 100, el 101 es exclusivo

  // Crea el mensaje con el timestamp
  snprintf(message, sizeof(message), 
           "{\"timestamp\": \"%s\", \"value\": \"%s\"}", 
           timestampString.c_str(), randomValue.c_str());
  
  // Construye el tópico con el UID
  String topic = "DATA/" + uid;

  // Publica en el topic "heartbeat"
  mqttClient.publish(topic.c_str(), message); 
  Serial.print("Mensaje enviado: ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(message);
  lcd.clear();
  lcd.setCursor(0, 0); // Coloca el cursor en la columna 0, fila 0
  lcd.print("T: ");
  lcd.print(timestampString.c_str());
  lcd.setCursor(0, 1); // Coloca el cursor en la columna 0, fila 1
  lcd.print("Value: ");
  lcd.print(randomValue.c_str());

}
