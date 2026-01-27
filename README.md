# Power Efficient XIAO ESP32-C3 WiFi Temperature Sensor

Low-power temperature logging system that batches readings and sends them over UDP. Built for XIAO ESP32-C3 with deep sleep between samples and light sleep during temperature sensor stabilization.

## How it works

Wakes up every 10 seconds, reads temperature, stores it in RTC memory, then goes back to deep sleep. After 30 samples (roughly 5 minutes), it connects to WiFi and dumps the batch via UDP to a remote server. Then resets the counter and repeats.

The WiFi connection uses static IP caching in RTC memory to speed up reconnects - first connection uses DHCP and saves the config, subsequent connections use the saved IP. If connection fails, it falls back to DHCP.

## Key features

- **Ultra-low power**: Active for ~627ms every 10 seconds (WiFi off), most of that in light sleep
- **Long Battery Life**: Continuous operation for a year on a 1000mAh battery (~115µA average)
- **Batched transmission**: 30 samples sent as single binary UDP packet
- **Fast WiFi**: Static IP caching cuts connection time to ~63ms vs ~2s (first connection)
- **Sensor power management**: DS18B20 powered only during measurement, GPIO pins floated when off
- **Debug mode**: Flip `DEBUG_MODE` to 1 for serial debug output, 0 for production (max battery life)

## Configuration

Install required libraries via Arduino Library Manager:
- OneWire
- DallasTemperature

```cpp
const char* ssid = "YourNetwork";
const char* password = "YourPassword";

const char* udpAddress = "192.168.1.100"; // Where to send the readings
const int udpPort = 2060;
```

Adjust timing and batch size for battery life:
```cpp
#define SAMPLE_COUNT 30              // Number of readings per WiFi transmission
#define DEEP_SLEEP_DURATION_MS 10000 // Milliseconds between readings (10 seconds)
```

## UDP packet format

Binary packet structure:
- Bytes 0-3: uint32_t sample count
- Bytes 4-: floats (temperature values in °C)

Total packet size: 124 bytes with 30 samples (4 + 30×4)

## Hardware wiring

DS18B20 connections:
- GND → **GPIO3** (actively driven LOW during measurement, floated when off)
- DATA → **GPIO4** (requires 4.7kΩ pull-up resistor to VCC for reliable operation)
- VCC → **GPIO5** (power controlled - HIGH during measurement, floated when off)

**Critical:** A 4.7kΩ pull-up resistor between DATA (GPIO4) and VCC is required. The internal pull-up resistor (~30-50kΩ) is too weak for reliable OneWire communication and device enumeration.

The sensor is completely powered down between readings to minimize battery drain. All three pins are set to INPUT (high-impedance/floating) when not in use to prevent current leakage through the GPIO pins. During measurement, GPIO3 provides GND, GPIO5 provides 3.3V power, and GPIO4 handles OneWire communication.

## Temperature reading strategy

Uses async conversion with variable-interval light sleep to minimize power during the ~627ms DS18B20 conversion time. The sensor is powered on, stabilized with a 20ms light sleep, then conversion starts.

The MCU sleeps in progressively shorter intervals to optimize for both power and responsiveness:
- First check at 600ms (typical conversion completion - measured at 627ms)
- Second check at +100ms (700ms total)
- Subsequent four checks every 50ms

This adaptive polling uses fewer wakeup cycles than uniform intervals, saving battery while catching conversion completion quickly. Sensor power (GPIO5) is driven LOW and all three GPIO pins are floated immediately after reading to eliminate current leakage.

## Notes

Deep sleep means `loop()` never runs - the chip fully resets on each wake. All persistent state lives in RTC_DATA_ATTR variables which survive deep sleep cycles.

WiFi is only enabled every 30th boot to transmit data. With static IP caching, WiFi connection completes in ~63ms (vs ~2000ms on first DHCP connection). WiFi is disconnected and powered off immediately after transmission to maximize battery life.

**Debug mode behavior**: When `DEBUG_MODE` is set to 1, the sleep macros are replaced with `delay()` calls instead of actual sleep functions. This keeps the serial port stable for continuous debug output. Deep sleep is simulated using a `goto RESTART` statement that jumps back to the beginning of `setup()` - a cheeky trick that lets you debug the wake-sleep cycle without losing the serial connection!
