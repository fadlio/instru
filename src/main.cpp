#include <WiFi.h>
#include <PubSubClient.h>

#define SERIALDEBUG
// #define MQTT

const char *WIFI_SSID = "Lio - HS";
const char *WIFI_PASSWORD = "l10_wifi";

const char *MQTT_SERVER_ADDRESS = "192.168.43.197"; //Lio - HS
const int MQTT_FREQUENCY = 60;

const int ADC_FREQUENCY = 300;
const int ADC_1_PIN = 34;
const int ADC_2_PIN = 35;
const int ADC_1_OFFSET = 1;
const int ADC_2_OFFSET = 2;
float _adc1Value = 0;
float _adc2Value = 0;

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi()
{

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1')
  {
    digitalWrite(BUILTIN_LED, LOW); // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  }
  else
  {
    digitalWrite(BUILTIN_LED, HIGH); // Turn the LED off by making the voltage HIGH
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("sensor/1/relay/led", "1");
      // ... and resubscribe
      client.subscribe("sensor/1/relay/led");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  pinMode(BUILTIN_LED, OUTPUT); // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  analogSetAttenuation(ADC_0db);

#ifdef MQTT
  setup_wifi();
  client.setServer(MQTT_SERVER_ADDRESS, 1883);
  client.setCallback(callback);
#endif
}

void mqtt_loop()
{
  char msg1[50];
  char msg2[50];

  snprintf(msg1, 50, "%ld", _adc1Value);
  snprintf(msg2, 50, "%ld", _adc2Value);

  client.publish("sensor/1/measure/voltage", msg1);
  client.publish("sensor/1/measure/current", msg2);
}

void adc_loop()
{
  int adc1 = analogRead(ADC_1_PIN);
  int adc2 = analogRead(ADC_2_PIN);

#ifdef SERIALDEBUG
  Serial.print("ADC 1 (Voltage): ");
  Serial.println(adc1);

  Serial.print("ADC 2 (Current): ");
  Serial.println(adc2);
#endif
}

void loop()
{
  long mqtt_millis = 0;
  long adc_millis = 0;

#ifdef MQTT
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
#endif

  while (1)
  {
    long now = millis();
    if (now - adc_millis >= 1000 / ADC_FREQUENCY)
    {
      adc_loop();
    }

#ifdef MQTT
    if (now - mqtt_millis >= 1000 / MQTT_FREQUENCY)
    {
      mqtt_loop();
    }
#endif
  }
}