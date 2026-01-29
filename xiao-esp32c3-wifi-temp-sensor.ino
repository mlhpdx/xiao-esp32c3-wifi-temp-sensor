#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_sleep.h"
#include "driver/temp_sensor.h"
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

// Each sample holds the extern/DS18B20 temp and the internal ESP32-C3 temp
struct TemperatureSample { float internal; float external; };

// RTC memory to persist data across deep sleep
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR uint32_t savedIPAddr = 0; // IPv4
RTC_DATA_ATTR uint32_t savedGatewayAddr = 0;
RTC_DATA_ATTR uint32_t savedSubnetAddr = 0;
RTC_DATA_ATTR uint32_t savedDNSAddr = 0;
RTC_DATA_ATTR volatile TemperatureSample temperatureSamples[SAMPLE_COUNT];
RTC_DATA_ATTR bool hasSensorAddress = false;
RTC_DATA_ATTR volatile uint8_t sensorAddress[8] = {0};
RTC_DATA_ATTR bool hasValidIP = false;


// Buffer for UDP packet (8 byte sensor ID + 4 byte count + samples)
#define UDP_BUFFER_SIZE (8 + sizeof(uint32_t) + sizeof(TemperatureSample) * SAMPLE_COUNT)
uint8_t udpBuffer[UDP_BUFFER_SIZE];

void readTemperatures() {
  unsigned long samplingStart = millis();

  // Power on the DS18B20
  pinMode(SENSOR_GND_PIN, OUTPUT);
  digitalWrite(SENSOR_GND_PIN, LOW);

  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);

  // Hold GPIO states during light sleep to keep sensor powered
  gpio_hold_en((gpio_num_t)SENSOR_GND_PIN);
  gpio_hold_en((gpio_num_t)SENSOR_POWER_PIN);

  // Light sleep for sensor power stabilization
  LIGHT_SLEEP(20 * 1000);
  
  pinMode(SENSOR_DATA_PIN, INPUT);  // INPUT_PULLUP to enable internal pullup

  // Initialize OneWire and DallasTemperature
  OneWire oneWire(SENSOR_DATA_PIN);
  DallasTemperature sensor(&oneWire);
  
  sensor.begin();
  
  // Get sensor address on first boot
  if (!hasSensorAddress) {
    sensor.getAddress((uint8_t*)sensorAddress, 0);
    hasSensorAddress = true;
    #if DEBUG_MODE
    Serial.print("Sensor address retrieved: ");
    for (int i = 0; i < 8; ++i) {
      if (sensorAddress[i] < 0x10) Serial.print('0');
      Serial.print(sensorAddress[i], HEX);
    }
    Serial.println("");
    #endif
  }
  
  // Start async temperature conversion
  sensor.setWaitForConversion(false);
  sensor.requestTemperatures();
  
  DEBUG_PRINTLN("Temperature conversion started on DS18B20");

  // while we wait for the DS18B20, grab the internal temperature from the ESP32-C3
  temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
  temp_sensor.dac_offset = TSENS_DAC_L2; // Default range (-10°C to 80°C)
  temp_sensor_set_config(temp_sensor);
  temp_sensor_start();
  delay(10); // Let the Sigma-Delta ADC stabilize

  float internal_temp = 0;
  temp_sensor_read_celsius(&internal_temp);  
  temperatureSamples[bootCount % SAMPLE_COUNT].internal = internal_temp;
  temp_sensor_stop();

  // DS18B20 needs ~650ms for 12-bit conversion
  LIGHT_SLEEP(650 * 1000);
  
  float temp = sensor.getTempCByIndex(0);

  DEBUG_PRINT("Conversion time: ");
  DEBUG_PRINT(millis() - samplingStart);
  DEBUG_PRINTLN("ms");
  
  // Release GPIO holds and power off the sensor
  gpio_hold_dis((gpio_num_t)SENSOR_GND_PIN);
  gpio_hold_dis((gpio_num_t)SENSOR_POWER_PIN);
  
  digitalWrite(SENSOR_POWER_PIN, LOW);
  pinMode(SENSOR_POWER_PIN, INPUT);  // Float the pin to prevent current leak
  pinMode(SENSOR_DATA_PIN, INPUT);  // Disable internal pullup to prevent current leak
  pinMode(SENSOR_GND_PIN, INPUT);  // Float the pin to prevent current leak
  
  DEBUG_PRINT("DS18B20 Temperature: ");
  DEBUG_PRINT(temp);
  DEBUG_PRINTLN("°C");

  DEBUG_PRINT("Internal Temperature: ");
  DEBUG_PRINT(internal_temp);
  DEBUG_PRINTLN("°C");

  temperatureSamples[bootCount % SAMPLE_COUNT].external = temp;
}

void prepareUDPBuffer() {
  size_t offset = 0;
  
  // Write sensor address (8 bytes)
  memcpy(udpBuffer + offset, (const void*)sensorAddress, 8);
  offset += 8;
  
  // Write sample count (4 bytes)
  uint32_t sampleCount = SAMPLE_COUNT;
  memcpy(udpBuffer + offset, &sampleCount, sizeof(uint32_t));
  offset += sizeof(uint32_t);
  
  // Write temperature samples
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    memcpy(udpBuffer + offset, (const void*)&(temperatureSamples[i].external), sizeof(float));
    offset += sizeof(float);
    memcpy(udpBuffer + offset, (const void*)&(temperatureSamples[i].internal), sizeof(float));
    offset += sizeof(float);
  }
  
  DEBUG_PRINTLN("UDP buffer prepared");
}

void setup() {
  DEBUG_BEGIN(115200);
  DEBUG_DELAY(200);
  
RESTART:

  // Read and store temperature samples before incrementing bootCount
  readTemperatures();
    
  bootCount++;
  DEBUG_PRINT("Boot number: ");
  DEBUG_PRINTLN(bootCount);
 
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

      WiFiUDP udp;
      udp.begin(udpPort);
      udp.beginPacket(udpAddress, udpPort);
      udp.write(udpBuffer, UDP_BUFFER_SIZE);
      udp.endPacket();

      delay(50);
      
      udp.stop();

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