#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_sleep.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// Debug mode - set to 0 for release builds
#define DEBUG_MODE 0

#if DEBUG_MODE
  #define DEBUG_BEGIN(x) Serial.begin(115200); delay(200)
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_FLUSH() Serial.flush(); delay(200)
  #define DEBUG_DELAY(x) delay(x)
  #define LIGHT_SLEEP(x) delay(x / 1000)
  #define DEEP_SLEEP(x) delay(x / 1000); goto RESTART
#else
  #define DEBUG_BEGIN(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_FLUSH()
  #define DEBUG_DELAY(x)
  #define LIGHT_SLEEP(x) esp_sleep_enable_timer_wakeup(x); esp_light_sleep_start()
  #define DEEP_SLEEP(x) esp_sleep_enable_timer_wakeup(x); esp_deep_sleep_start();
#endif

// DS18B20 pins - sensor powered only during measurement
#define SENSOR_GND_PIN 3   // D1 = GPIO3 - GND for DS18B20
#define SENSOR_DATA_PIN 4   // D2 = GPIO4 - OneWire data line
#define SENSOR_POWER_PIN 5   // D3 = GPIO5 - VCC for DS18B20

// WiFi credentials
const char* ssid = "YourNetwork";
const char* password = "YourPassword";

// UDP settings
const char* udpAddress = "10.10.10.10";   // Where to send the readings
const int udpPort = 1010;                 // Server port

// Samples to collect between transmissions
#define SAMPLE_COUNT 30
#define DEEP_SLEEP_DURATION_MS 10000

// RTC memory to persist data across deep sleep
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR float temperatureSamples[SAMPLE_COUNT];
RTC_DATA_ATTR uint32_t savedIPAddr = 0;      // Store as uint32_t
RTC_DATA_ATTR uint32_t savedGatewayAddr = 0;
RTC_DATA_ATTR uint32_t savedSubnetAddr = 0;
RTC_DATA_ATTR uint32_t savedDNSAddr = 0;
RTC_DATA_ATTR bool hasValidIP = false;

WiFiUDP udp;

// Buffer for UDP packet (100 floats = 400 bytes + 4 byte header)
#define UDP_BUFFER_SIZE (sizeof(uint32_t) + sizeof(float) * SAMPLE_COUNT)
uint8_t udpBuffer[UDP_BUFFER_SIZE];

float readTemperature() {
  // Power on the DS18B20
  pinMode(SENSOR_GND_PIN, OUTPUT);
  digitalWrite(SENSOR_GND_PIN, LOW);

  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);

  // Light sleep for sensor power stabilization
  LIGHT_SLEEP(20 * 1000);
  
  pinMode(SENSOR_DATA_PIN, INPUT);  // INPUT_PULLUP to enable internal pullup

  // Initialize OneWire and DallasTemperature
  OneWire oneWire(SENSOR_DATA_PIN);
  DallasTemperature sensor(&oneWire);
  
  sensor.begin();
  
  // Start async temperature conversion
  sensor.setWaitForConversion(false);
  sensor.requestTemperatures();
  
  DEBUG_PRINTLN("Temperature conversion started");
  
  // DS18B20 needs ~750ms for 12-bit conversion
  // Use light sleep in a loop to save power while waiting
  const unsigned long sleepDurations[] = {600 * 1000, 100 * 1000, 50 * 1000, 50 * 1000, 50 * 1000, 50 * 1000 };
  unsigned long conversionStart = millis();
  int sleepIndex = 0;
  
  while (!sensor.isConversionComplete() && sleepIndex < sizeof(sleepDurations)/sizeof(sleepDurations[0])) {
    DEBUG_PRINT("Entering light sleep... ");
    DEBUG_PRINTLN(sleepDurations[sleepIndex] / 1000);
    LIGHT_SLEEP(sleepDurations[sleepIndex]);
    sleepIndex++;
  }
  
  float temp = sensor.getTempCByIndex(0);

  DEBUG_PRINT("Conversion time: ");
  DEBUG_PRINT(millis() - conversionStart);
  DEBUG_PRINTLN("ms");
  
  // Power off the sensor
  digitalWrite(SENSOR_POWER_PIN, LOW);
  pinMode(SENSOR_POWER_PIN, INPUT);  // Float the pin to prevent current leak
  pinMode(SENSOR_DATA_PIN, INPUT);  // Disable internal pullup to prevent current leak
  pinMode(SENSOR_GND_PIN, INPUT);  // Float the pin to prevent current leak
  
  // Return error value if reading failed
  if (temp == DEVICE_DISCONNECTED_C) {
    DEBUG_PRINTLN("ERROR: Sensor disconnected!");
    return -127.0;
  }
  
  return temp;
}

void prepareUDPBuffer() {
  uint32_t sampleCount = SAMPLE_COUNT;
  size_t offset = 0;
  
  memcpy(udpBuffer + offset, &sampleCount, sizeof(uint32_t));
  offset += sizeof(uint32_t);
  
  memcpy(udpBuffer + offset, temperatureSamples, SAMPLE_COUNT * sizeof(float));
  
  DEBUG_PRINTLN("UDP buffer prepared");
}

void setup() {
  DEBUG_BEGIN(115200);
  DEBUG_DELAY(200);
  
RESTART:

  // Read and store temperature sample
  float currentTemp = readTemperature();
  temperatureSamples[bootCount] = currentTemp;
  
  bootCount++;
  DEBUG_PRINT("Boot number: ");
  DEBUG_PRINTLN(bootCount);

  DEBUG_PRINT("Temperature: ");
  DEBUG_PRINT(currentTemp);
  DEBUG_PRINTLN("°C");
  
  if (bootCount >= SAMPLE_COUNT) {
    DEBUG_PRINTLN("Counter reached SAMPLE_COUNT");
    
    prepareUDPBuffer();
    
    DEBUG_PRINTLN("Starting WiFi...");
    
    #if DEBUG_MODE
    unsigned long wifiStartTime = millis();
    #endif
    
    // Configure static IP if we have a saved one
    if (hasValidIP && savedIPAddr != 0) {
      IPAddress ip(savedIPAddr);
      IPAddress gateway(savedGatewayAddr);
      IPAddress subnet(savedSubnetAddr);
      IPAddress dns(savedDNSAddr);

      DEBUG_PRINT("Using saved IP configuration: ");
      DEBUG_PRINTLN(ip.toString());
      
      WiFi.config(ip, gateway, subnet, dns);
    }
    
    WiFi.begin(ssid, password);
    
    int maxAttempts = hasValidIP ? 40 : 100;
    int attempts = 0;
    
    while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
      delay(50);
      attempts++;
      #if DEBUG_MODE
      if (attempts % 20 == 0) {
        DEBUG_PRINT(".");
      }
      #endif
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      #if DEBUG_MODE
      unsigned long connectionTime = millis() - wifiStartTime;
      #endif
      
      DEBUG_PRINT("\nConnected to WiFi in ");
      DEBUG_PRINT(connectionTime);
      DEBUG_PRINTLN("ms");
      
      IPAddress localIP = WiFi.localIP();
      DEBUG_PRINT("IP address: ");
      DEBUG_PRINTLN(localIP.toString());

      // Save IP configuration as uint32_t for reliable RTC storage
      savedIPAddr = (uint32_t)localIP;
      savedGatewayAddr = (uint32_t)WiFi.gatewayIP();
      savedSubnetAddr = (uint32_t)WiFi.subnetMask();
      savedDNSAddr = (uint32_t)WiFi.dnsIP();
      hasValidIP = true;
      
      DEBUG_PRINTLN("IP configuration saved to RTC memory");

      udp.begin(udpPort);
      udp.beginPacket(udpAddress, udpPort);
      udp.write(udpBuffer, UDP_BUFFER_SIZE);
      udp.endPacket();

      delay(50);
      
      DEBUG_PRINT("Binary UDP packet sent (");
      DEBUG_PRINT(UDP_BUFFER_SIZE);
      DEBUG_PRINTLN(" bytes)");
      
      bootCount = 0;
    } else {
      DEBUG_PRINTLN("\nFailed to connect to WiFi");
      hasValidIP = false;
    }
    
    DEBUG_PRINT("Total WiFi-on time: ~");
    DEBUG_PRINT(millis() - wifiStartTime);
    DEBUG_PRINTLN("ms");

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
  
  DEBUG_PRINTLN("Entering deep sleep...");
  
  DEEP_SLEEP(DEEP_SLEEP_DURATION_MS * 1000);
}

void loop() {
  // Never reached due to deep sleep
  delay(1000);
  Serial.println(".");
}