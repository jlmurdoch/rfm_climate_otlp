// For RFM
#include <SPI.h>
#include "src/rfm69hcw/rfm69hcw.h"
// #include "src/hardware_pins.h"

// For outbound connectivity (OTLP)
#include "src/connection_details.h"
#include "src/hwclock/hwclock.h"
#include "src/otel-protobuf/otel-protobuf.h"
#include "src/send-protobuf/send-protobuf.h"

#include <Adafruit_DPS310.h>
Adafruit_DPS310 dps;

uint8_t thisValue, lastValue, thisGap, pulses = 0;
uint32_t thisTime, lastTime = 0;
uint64_t thisdata, lastdata = 0;

// Continuous handing once consumed
#define PULSE_LEAD  0x1 // HIGH of 500ms (488-544)
#define PULSE_SHORT 0x3 // LOW of 1000ms (980-1012)
#define PULSE_LONG  0x7 // LOW of 2000ms (1956-1996)
#define BIT_COUNT   36

extern SPIClass *vspi = NULL;

void setup() {
  // Send output on serial console - need to enable CDC on boot to work
  Serial.begin(115200);

  // Takes about a second for the Serial to kick in on the M5 Atom Lite
  delay(1000);

  // Connect to WiFi before we attempt anything
  WiFi.begin(WIFI_SSID, WIFI_PSK);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("...Waiting for WiFi..."));
    delay(5000);
  }
  Serial.println(F("WiFi Connected"));

  // NTP
  setHWClock(NTP_HOST);
  Serial.println(F("NTP Synced"));

  // Digital Pressure Sensor
  dps.begin_I2C();
  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
 
  // RFM SPI setup
  vspi = new SPIClass(HSPI);
  vspi->begin(SCK, MISO, MOSI, T6);
  //vspi->begin();

  // RFM Pins
  pinMode(vspi->pinSS(), OUTPUT); // Device Select
  pinMode(T9, OUTPUT); // Reset
  pinMode(T11, INPUT);  // Data

  // Initialise the RFM, passing SPI, Reset pin and radio settings
  rfminit(vspi, T9, CONT_SYNC_OOK_NONE, 433.73, 2000, OOK_167KHZ);
}

void loop() {
  // put your main code here, to run repeatedly:
  thisValue = digitalRead(T11);

  // If there is a data change, process it
  if (thisValue != lastValue) {
    // Record the time this has changed
    thisTime = micros();
    // Round-down the ms value to 256ms chunks and recenter
    thisGap = (thisTime - lastTime - 128) >> 8;
    
    // Check the size of the pulse we have received
    switch (thisGap) {
      // Valid Data to be processed
      case PULSE_SHORT: // 1000 ms = 0
      case PULSE_LONG: // 2000 ms = 1
        // Make room for a new bit
        thisdata <<= 1;
        // Set the bit to the what the gap size represents
        thisdata |= (!(~thisGap & 0x04));
        // Increment the counter
        pulses++;
        // Wait for next bit
        break;
    
      // Valid pre-bit pulse that can be safely skipped
      case PULSE_LEAD:
        if (~thisValue) {
          break;
        }

      // Reset if we don't see a valid pulse
      default:
        thisdata = 0;
        pulses = 0;
    }
    
    // If we have enough data, work with it and reset
    if (pulses == BIT_COUNT) {
      if (thisdata == lastdata) {
        uint8_t payloadData[MAX_PROTOBUF_BYTES] = { 0 };
        Resourceptr ptr = NULL;
        ptr = addOteldata();
        addResAttr(ptr, "service.name", "jlm-home");

        uint64_t *humidity = (uint64_t *)malloc(sizeof(uint64_t));
        double *temperature = (double *)malloc(sizeof(double));
        *humidity = thisdata & 0x7F;
        *temperature = (((int16_t)((thisdata >> 8) & 0xFFF0)) >> 4) * 0.1;

        uint8_t channel = ((thisdata >> 24) & 0x3) + 1;
        uint8_t manual = (thisdata >> 26) & 0x1;
        uint8_t battery = (thisdata >> 27) & 0x1; 
        uint8_t id = (thisdata >> 28) & 0xFF;

        char device_name[8] = {0};
        sprintf(device_name, "dev-%d", id);

        addMetric(ptr, "humidity", "Relative Humidity", "%{saturation}", METRIC_GAUGE, 0, 0);
        addDatapoint(ptr, AS_INT, humidity);
        addDpAttr(ptr, "sensor", device_name);

        addMetric(ptr, "temperature", "Temperature in Celcius", "Cel", METRIC_GAUGE, 0, 0);
        addDatapoint(ptr, AS_DOUBLE, temperature);
        addDpAttr(ptr, "sensor", device_name);

        // Get a reading from the onboard sensor while we are at it
        if (dps.temperatureAvailable() && dps.pressureAvailable()) {
          sensors_event_t temp_event, pressure_event;
          dps.getEvents(&temp_event, &pressure_event);

          double *dps_temperature = (double *)malloc(sizeof(double));
          *dps_temperature = temp_event.temperature;

          addMetric(ptr, "temperature", "Temperature in Celcius", "Cel", METRIC_GAUGE, 0, 0);
          addDatapoint(ptr, AS_DOUBLE, dps_temperature);
          addDpAttr(ptr, "sensor", "dev-0");

          double *dps_pressure = (double *)malloc(sizeof(double));
          *dps_pressure = pressure_event.pressure;

          addMetric(ptr, "air_pressure", "Air Pressure", "mbar", METRIC_GAUGE, 0, 0);
          addDatapoint(ptr, AS_DOUBLE, dps_pressure);
          addDpAttr(ptr, "sensor", "dev-0");
        }
        printOteldata(ptr);
        size_t payloadSize = buildProtobuf(ptr, payloadData, MAX_PROTOBUF_BYTES);
        // Send the data if there's something there
        if(payloadSize > 0) {
          // Define OTEL_SSL if SSL is required
          sendProtobuf(OTEL_HOST, OTEL_PORT, OTEL_URI, OTEL_XSFKEY, payloadData, payloadSize);
        }
        freeOteldata(ptr);

        delay(500);
        lastdata = 0;
      } else if (((thisdata >> 7) & 0x1F) == 0x1E) {
        // First observation of something that looks good... wait for a second
        lastdata = thisdata;
      }

      thisdata = 0;
      pulses = 0;
    }
    // Record the major data for the next pass
    lastValue = thisValue;
    lastTime = thisTime;      
  }
}
