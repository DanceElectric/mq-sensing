/*
* @project:       MqSensing
* @author:        Alexander Becker
* @info:          ESP32-WROOM-32E
* @date:          02.04.2021
*
* @PullUp Res. :  ref. to +3V3 DC
* @GxEPD2 Library: DIN (23), CLK (18) are defined in GxEPD2 Library!
*/

// Define DEBUG things for Serial OUTPUT
#define DEBUG_SERIAL

// Define INIT_BQ27441 for writing gauge battery config
#define INIT_BQ27441

/* ---------- Define Libraries here ------------ */

//#include <SensirionCore.h>

// Arduino IDE Libraries
#include <Arduino.h>
#include <Wire.h>

// Sensor Libraries
#include <sps30.h>
#include <Adafruit_SHT31.h>
#include <SparkFunBQ27441.h>
#include <SensirionI2CSvm40.h>
#include <SensirionI2CScd4x.h>
#include <SensirionI2CSfa3x.h>

// Epaper Display Libraries
#include <GxEPD2_BW.h>
#include <Adafruit_GFX.h>

// FreeFonts from Adafruit_GFX
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMono12pt7b.h>
#include <Fonts/FreeMono18pt7b.h>
#include <Fonts/FreeMono24pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>

/* ---------- Define Functions here ----------- */

// Epaper Display Function for different Screens
void drawDisplay();
void drawDisplay_charging();
void drawDisplay_is_charging();
void drawDisplay_charging_complete();

// PM Sensor Function for measurement
void SPS30_setup();
void SPS30_read();
void SPS30_read_loop();
int SPS30_validate();
String SPS30_validate_text();

// Temp/Hum Sensor Function for measurement
void SHT31_setup();
void SHT31_read();
int SHT31_validate_temp();
int SHT31_validate_hum();
String SHT31_validate_text();

// Sensor Function for measurement
void SCD4x_setup();
void SCD4x_read();

// Battery Gauge Sensor Function for measurement
void BQ27441_setup();
void BQ27441_read();
boolean power_good();

/* -------------------------------------------- */

// Define Pins
#define EPD_CS 5
#define EPD_DC 17
#define EPD_RST 16
#define EPD_BUSY 4 // can set to -1 to not use a pin (will wait a fixed delay)
#define GPIO16 16
#define GPIO2_SDA 21
#define GPIO5_SCL 22
#define TOUT_ADC A0
#define SPS30_COMMS Wire // -> define communication channel to use for SPS30

// DEEP SLEEP TIME
#define uS_TO_S_FACTOR 1000000 // Conversion factor for micro seconds to seconds
#define DEEP_SLEEP_TIME 5      // Time ESP32 will go to sleep (in seconds)

// Define Pin Mapping for GxEPD Library
GxEPD2_BW<GxEPD2_420, GxEPD2_420::HEIGHT> display(GxEPD2_420(/*CS=*/EPD_CS, /*DC=*/EPD_DC, /*RST=*/EPD_RST, /*BUSY=*/EPD_BUSY));

// Base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code.
// Enable or disable GxEPD2_GFX base class:
#define ENABLE_GxEPD2_GFX 0

#define MAX_DISPLAY_BUFFER_SIZE 15000ul // ~15k is a good compromise
#define MAX_HEIGHT_3C(EPD) (EPD::HEIGHT <= (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8) ? EPD::HEIGHT : (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8))

// SPS30
#define SPS30_NUMB_OFFSET 2
#define SPS30_NUMB_READINGS 3

double sps30_MassPM1 = 0.00;
double sps30_MassPM2 = 0.00;
double sps30_MassPM4 = 0.00;
double sps30_MassPM10 = 0.00;

double sps30_MassPM1_array[SPS30_NUMB_READINGS] = {0.00};
double sps30_MassPM2_array[SPS30_NUMB_READINGS] = {0.00};
double sps30_MassPM4_array[SPS30_NUMB_READINGS] = {0.00};
double sps30_MassPM10_array[SPS30_NUMB_READINGS] = {0.00};

double sps30_NumPM0 = 0.00;
double sps30_NumPM1 = 0.00;
double sps30_NumPM2 = 0.00;
double sps30_NumPM4 = 0.00;
double sps30_NumPM10 = 0.00;

double sps30_PartSize = 0.00;

SPS30 sps30;

// SHT31
double temperature = 0.00;
double humidity = 0.00;

Adafruit_SHT31 sht31 = Adafruit_SHT31();

// SVM40
SensirionI2CSvm40 svm40;

// SCD4x
SensirionI2CScd4x scd4x;

// SFA30
SensirionI2CSfa3x sfa3x;

// BQ27441
const unsigned int BATTERY_CAPACITY = 3450; // capacity of your battery in mAh
const uint16_t TERMINATE_VOLTAGE = 2650;    // lowest operational voltage in mV
const uint16_t TAPER_CURRENT = 50;          // current at which charger stops charging battery in mA

unsigned int battery_soc = 0;
unsigned int battery_mV = 0;
int battery_current = 0;
unsigned int battery_fullCapacity = 0;
unsigned int battery_capacity = 0;
int battery_power = 0;
int battery_soh = 0;

// TIMER (millis)
unsigned int t_count = 0;
unsigned long t1 = 0, t2 = 0;

void setup()
{
  Serial.begin(74880);

  display.init(74880); // Initiate the display

#ifdef DEBUG_SERIAL
  Serial.println("Setup start");
#endif

  // Start I2C
  Wire.setClock(100000);
  Wire.begin(GPIO2_SDA, GPIO5_SCL);

  // Read Sensor Values
  SHT31_setup();
  SHT31_read();

  SCD4x_setup();
  SCD4x_read();

  // Draw Display
  drawDisplay();

  // going to DeepSleep
  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIME * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void loop()
{
  // This is not going to be called because of DeepSleep
}

void drawDisplay()
{
  display.setRotation(0); // Set orientation. Goes from 0, 1, 2 or 3

  display.setTextWrap(false); // By default, long lines of text are set to automatically “wrap” back to the leftmost column.
                              // To override this behavior (so text will run off the right side of the display - useful for
                              // scrolling marquee effects), use setTextWrap(false). The normal wrapping behavior is restored
                              // with setTextWrap(true).

  display.setFullWindow(); // Set full window mode, meaning is going to update the entire screen

  // Here we use paged drawing, even if the processor has enough RAM for full buffer
  // so this can be used with any supported processor board.
  // the cost in code overhead and execution time penalty is marginal
  display.firstPage(); // Tell the graphics class to use paged drawing mode

  do
  {
    display.fillScreen(GxEPD_WHITE);   // Clear previous graphics to start over to print new things.
    display.setTextColor(GxEPD_BLACK); // Set color for text

    display.setFont(&FreeMonoBold12pt7b); // Set font
    display.setCursor(0, 15);             // Set the position to start printing text (x,y)
    display.println("Hello World!");      // Print some text

  } while (display.nextPage());
}

void SHT31_setup()
{
  // Initialize sensor SHT31
  if (!sht31.begin(0x44))
  {
#ifdef DEBUG_SERIAL
    Serial.println("Sensor SHT31 not found");
#endif
  }
}

void SHT31_read()
{
  temperature = sht31.readTemperature();
  humidity = sht31.readHumidity();

#ifdef DEBUG_SERIAL
  Serial.print("Temperatur: ");
  Serial.print(temperature);
  Serial.println();

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println();
#endif

  if (sht31.isHeaterEnabled())
  {
    sht31.heater(false);

#ifdef DEBUG_SERIAL
    Serial.println("SHT31 Heater will be disabled.");
#endif
  }
}

void SCD4x_printUint16Hex(uint16_t value)
{
  Serial.print(value < 4096 ? "0" : "");
  Serial.print(value < 256 ? "0" : "");
  Serial.print(value < 16 ? "0" : "");
  Serial.print(value, HEX);
}

void SCD4x_printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2)
{
  Serial.print("Serial: 0x");
  SCD4x_printUint16Hex(serial0);
  SCD4x_printUint16Hex(serial1);
  SCD4x_printUint16Hex(serial2);
  Serial.println();
}

void SCD4x_setup()
{
  uint16_t error;
  char errorMessage[256];

  scd4x.begin(Wire);

  // stop potentially previously started measurement
  error = scd4x.stopPeriodicMeasurement();
  if (error)
  {
    Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

  uint16_t serial0;
  uint16_t serial1;
  uint16_t serial2;
  error = scd4x.getSerialNumber(serial0, serial1, serial2);
  if (error)
  {
    Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  else
  {
    SCD4x_printSerialNumber(serial0, serial1, serial2);
  }

  // Start Measurement
  error = scd4x.startPeriodicMeasurement();
  if (error)
  {
    Serial.print("Error trying to execute startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
}
void SCD4x_read()
{
  uint16_t error;
  char errorMessage[256];

  delay(5000);

  // Read Measurement
  uint16_t co2;
  uint16_t temperature;
  uint16_t humidity;

  error = scd4x.readMeasurement(co2, temperature, humidity);
  if (error)
  {
    Serial.print("Error trying to execute readMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  else if (co2 == 0)
  {
    Serial.println("Invalid sample detected, skipping.");
  }
  else
  {
    Serial.print("Co2:");
    Serial.print(co2);
    Serial.print("\t");
    Serial.print("Temperature:");
    Serial.print(temperature * 175.0 / 65536.0 - 45.0);
    Serial.print("\t");
    Serial.print("Humidity:");
    Serial.println(humidity * 100.0 / 65536.0);
  }
}

void SPS30_setup()
{
// Connect sps30
#ifdef DEBUG_SERIAL
  Serial.println("Trying to connect");
#endif

  // Begin communication channel
  SPS30_COMMS.begin();

  if (sps30.begin(&SPS30_COMMS) == false)
  {
#ifdef DEBUG_SERIAL
    Serial.println("Could not set I2C SPS30");
#endif
  }

  // reset SPS30 connection
  if (sps30.reset() == false)
  {
#ifdef DEBUG_SERIAL
    Serial.println("Could not reset SPS30");
#endif
  }

  // check for SPS30 connection
  if (sps30.probe() == false)
  {
#ifdef DEBUG_SERIAL
    Serial.println("could not probe / connect with SPS30");
#endif
  }
  else
  {
#ifdef DEBUG_SERIAL
    Serial.println("Detected SPS30");
#endif
  }

  // start measurement
  if (sps30.start())
  {
#ifdef DEBUG_SERIAL
    Serial.println("Measurement started SPS30");
#endif
  }
  else
  {
#ifdef DEBUG_SERIAL
    Serial.println("Could not start measurement SPS30");
#endif
  }

  if (sps30.I2C_expect() == 4)
  {
#ifdef DEBUG_SERIAL
    Serial.println("!!! Due to I2C buffersize only the SPS30 MASS concentration is available !!!");
#endif
  }
}

void SPS30_read()
{
  uint8_t ret = 0;

  t_count = 0;

#ifdef DEBUG_SERIAL
  Serial.println("Perform wakeup SPS30");
#endif

  // wakeup SPS30
  ret = sps30.wakeup();

  if (ret != ERR_OK)
  {
#ifdef DEBUG_SERIAL
    Serial.println("Could not wakeup SPS30");
#endif
  }
  else
  {
#ifdef DEBUG_SERIAL
    Serial.println("SPS30 wakes up");
#endif
  }

// as the SPS30 was in measurement mode before sleep that is restored
#ifdef DEBUG_SERIAL
  Serial.println("measurement mode SPS30");
#endif

  while (t_count < SPS30_NUMB_READINGS)
  {
    delay(2000);
    SPS30_read_loop();
    t_count++;
  }

  for (int j = SPS30_NUMB_OFFSET; j < SPS30_NUMB_READINGS; j++)
  {
    sps30_MassPM1 = sps30_MassPM1 + sps30_MassPM1_array[j];
    sps30_MassPM2 = sps30_MassPM2 + sps30_MassPM2_array[j];
    sps30_MassPM4 = sps30_MassPM4 + sps30_MassPM4_array[j];
    sps30_MassPM10 = sps30_MassPM10 + sps30_MassPM10_array[j];
  }

  sps30_MassPM1 = sps30_MassPM1 / (SPS30_NUMB_READINGS - SPS30_NUMB_OFFSET);
  sps30_MassPM2 = sps30_MassPM2 / (SPS30_NUMB_READINGS - SPS30_NUMB_OFFSET);
  sps30_MassPM4 = sps30_MassPM4 / (SPS30_NUMB_READINGS - SPS30_NUMB_OFFSET);
  sps30_MassPM10 = sps30_MassPM10 / (SPS30_NUMB_READINGS - SPS30_NUMB_OFFSET);

// Print Sensor Values Result
#ifdef DEBUG_SERIAL
  Serial.println("mean value of PMx measurement");
  Serial.print(sps30_MassPM1);
  Serial.print(F("\t"));
  Serial.print(sps30_MassPM2);
  Serial.print(F("\t"));
  Serial.print(sps30_MassPM4);
  Serial.print(F("\t"));
  Serial.print(sps30_MassPM10);
  Serial.print(F("\n"));
#endif

#ifdef DEBUG_SERIAL
  Serial.println("Entering sleep-mode SPS30");
#endif

  // put the SPS30 to sleep
  ret = sps30.sleep();

  if (ret != ERR_OK)
  {
#ifdef DEBUG_SERIAL
    Serial.println("Could not set SPS30 to sleep");
#endif
  }
  else
  {
#ifdef DEBUG_SERIAL
    Serial.println("SPS30 goes to sleep");
#endif
  }
}

void SPS30_read_loop()
{

  uint8_t ret, error_cnt = 0;
  struct sps_values val;

  // loop to get data
  do
  {

    ret = sps30.GetValues(&val);

    // data might not have been ready
    if (ret == ERR_DATALENGTH)
    {

      if (error_cnt++ > 3)
      {
#ifdef DEBUG_SERIAL
        Serial.println("Error during reading values SPS30");
#endif
      }
      delay(1000);
    }
    // if other error
    else if (ret != ERR_OK)
    {
#ifdef DEBUG_SERIAL
      Serial.println("Error during reading values SPS30");
#endif
    }

  } while (ret != ERR_OK);

  // Read Sensor Values and Store in Variable
  sps30_MassPM1_array[t_count] = val.MassPM1;
  sps30_MassPM2_array[t_count] = val.MassPM2;
  sps30_MassPM4_array[t_count] = val.MassPM4;
  sps30_MassPM10_array[t_count] = val.MassPM10;

// Print Sensor Values on Serial
#ifdef DEBUG_SERIAL
  Serial.print(sps30_MassPM1_array[t_count]);
  Serial.print(F("\t"));
  Serial.print(sps30_MassPM2_array[t_count]);
  Serial.print(F("\t"));
  Serial.print(sps30_MassPM4_array[t_count]);
  Serial.print(F("\t"));
  Serial.print(sps30_MassPM10_array[t_count]);
  Serial.print(F("\n"));
#endif
}

void BQ27441_setup()
{
  // Initialize sensor
  if (!lipo.begin())
  {
#ifdef DEBUG_SERIAL
    Serial.println("Sensor BQ27441 not found");
#endif
  }

  delay(500);

#ifdef INIT_BQ27441
  Serial.println("Writing gague config");

  lipo.enterConfig();                 // To configure the values below, you must be in config mode
  lipo.setCapacity(BATTERY_CAPACITY); // Set the battery capacity

  /*
      Design Energy should be set to be Design Capacity × 3.7 if using the bq27441-G1A or Design
      Capacity × 3.8 if using the bq27441-G1B
    */
  lipo.setDesignEnergy(BATTERY_CAPACITY * 3.7f);

  /*
      Terminate Voltage should be set to the minimum operating voltage of your system. This is the target
      where the gauge typically reports 0% capacity
    */
  lipo.setTerminateVoltage(TERMINATE_VOLTAGE);

  /*
      Taper Rate = Design Capacity / (0.1 * Taper Current)
    */
  lipo.setTaperRate(10 * BATTERY_CAPACITY / TAPER_CURRENT);

  lipo.exitConfig(); // Exit config mode to save changes
#endif
}

void BQ27441_read()
{
  battery_soc = lipo.soc();
  battery_mV = lipo.voltage();
  battery_current = lipo.current();

// Now print out those values
#ifdef DEBUG_SERIAL
  String toPrint = String(battery_soc) + "% | ";
  toPrint += String(battery_mV) + " mV | ";
  toPrint += String(battery_current) + " mA | ";
  toPrint += String(battery_capacity) + " / ";
  toPrint += String(battery_fullCapacity) + " mAh | ";
  toPrint += String(battery_power) + " mW | ";
  toPrint += String(battery_soh) + "%";
  Serial.println(toPrint);
#endif
}
