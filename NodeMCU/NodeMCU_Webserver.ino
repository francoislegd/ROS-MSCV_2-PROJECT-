#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <aREST.h>

WiFiClient espClient;
PubSubClient client(espClient);

aREST arestVar = aREST(client);

/*Enter an Unique ID to identify the device for arest.io
A Maximum of 6 characters are supported. Must be unique.*/
char* device_id = "123456";  

// Enter SSID and Password of your WiFi Network
const char* ssid = "*******"; // Name of WiFi Network. 
const char* password = "******"; // Password of your WiFi Network.

void callback(char* topic, byte* payload, unsigned int length);

void setup(void)
{
  Serial.begin(115200);

  client.setCallback(callback);
  
  // Give name and ID to device
  arestVar.set_id(device_id);
  arestVar.set_name("*****");

  // Connect to WiFi 
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print("*");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  char* out_topic = arestVar.get_topic();
}

void loop() 
{
  arestVar.loop(client);
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  arestVar.handle_callback(client, topic, payload, length);
}
