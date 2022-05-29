/*
  ESP8266 serial 2 tcp wifi bridge by Stefan Denninger 29.05.2022 seen by Daniel Parnell 2nd of May 2015
 */

#define USE_WDT

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <SoftwareSerial.h>

// application config

// change the network details to match your wifi network
#define WIFI_SSID "yourSSID"
#define WIFI_PASSWORD "PASSWORD"

#define BAUD_RATE 9600
#define TCP_LISTEN_PORT 81

// if the bonjour support is turned on, then use the following as the name
#define DEVICE_NAME "ser2net"

// serial end ethernet buffer size
constexpr int BUFFER_SIZE = 128; // use fractions of 256

// hardware config
#define WIFI_LED 2
//#define CONNECTION_LED 16
#define TX_IR_LED 15
#define RX_IR_LED 14

WiFiServer server(TCP_LISTEN_PORT);

void connect_to_wifi() {
  int count = 0;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Wait for WIFI connection
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(WIFI_LED, LOW);
    delay(250);
    digitalWrite(WIFI_LED, HIGH);
    delay(250);
  }
}


void error() {
  int count = 0;
  
  while(1) {
    digitalWrite(WIFI_LED, LOW);
    delay(100);
    digitalWrite(WIFI_LED, HIGH);
    delay(100);
  }
}
SoftwareSerial serial;

void setup(void) {  
  Serial.begin(115200);
  
  pinMode(WIFI_LED, OUTPUT);
  digitalWrite(WIFI_LED, LOW);
  
  serial.begin(BAUD_RATE, SWSERIAL_8N1, RX_IR_LED, TX_IR_LED, false, 4 * BUFFER_SIZE);
  
  // Connect to WiFi network
  connect_to_wifi();

  Serial.println(WiFi.localIP());

  // Start TCP server
  server.begin();
}

WiFiClient client;
uint8 pulse = 0;
uint8 pulse_dir = 1;
int pulse_counter = 1;

//***************************************** loop
void loop(void)
{
  size_t bytes_read;
  uint8_t net_buf[BUFFER_SIZE];
  uint8_t serial_buf[BUFFER_SIZE];
  
  if(WiFi.status() != WL_CONNECTED) {
    // we've lost the connection, so we need to reconnect
    if(client) client.stop();
    connect_to_wifi();
  }
  
  // Check if a client has connected
  if (!client) {
    // eat any bytes in the serial buffer as there is nothing to see them
    while(serial.available()) {
      serial.read();
    }
      
    client = server.available();
    if(!client) {      
      //  digitalWrite(CONNECTION_LED, LOW);
      return;
    }
  }

#define min(a,b) ((a)<(b)?(a):(b))
  if(client.connected()) {

    // check the network for any bytes to send to the serial
    int count = client.available();
    if(count > 0) {      
      //digitalWrite(TX_LED, HIGH);
      
      bytes_read = client.read(net_buf, min(count, BUFFER_SIZE));
      serial.write(net_buf, bytes_read);
      serial.flush();
    } else {
      //digitalWrite(TX_LED, LOW);
    }
    
    // now check the serial for any bytes to send to the network
    bytes_read = 0;
    while(serial.available() && bytes_read < BUFFER_SIZE) {
      serial_buf[bytes_read] = serial.read();
      bytes_read++;
    }
    
    if(bytes_read > 0) {  
      //digitalWrite(RX_LED, HIGH);
      client.write((const uint8_t*)serial_buf, bytes_read);
      client.flush();
      
      digitalWrite(WIFI_LED, LOW);
      delay(100);
      digitalWrite(WIFI_LED, HIGH);
      delay(100);
      Serial.print(".");

    } else {
      //digitalWrite(RX_LED, LOW);
    }
  } else {
    client.stop();
  }

}
