/*
  Incorporates elements of: 
    matter_fan.ino (https://github.com/SiliconLabs/arduino/blob/main/libraries/Matter/examples/matter_fan/matter_fan.ino)
    matter_lightbulb_custom_name.ino (https://github.com/SiliconLabs/arduino/blob/main/libraries/Matter/examples/matter_lightbulb_custom_name/matter_lightbulb_custom_name.ino)
    matter_lightbulb_callback.ino (https://github.com/SiliconLabs/arduino/blob/main/libraries/Matter/examples/matter_lightbulb_callback/matter_lightbulb_callback.ino)
   By Tamas Jozsi (Silicon Labs)

  Author: Timothy Perrett
 */

// If defined, debug messages are printed to Serial
#define DEBUG 1

// If defined, Matter setup debug messages are printed to Serial
#define DEBUG_MATTER 1

// If defined, button inputs and interrupts are used
// #define BUTTONS 1

// Configuration option for active low/high relays.
#define RELAY_ACTIVE LOW
#if RELAY_ACTIVE == LOW
  #define RELAY_INACTIVE HIGH
#elif RELAY_ACTIVE == HIGH
  #define RELAY_INACTIVE LOW
#else
  #error RELAY_ACTIVE must be set to HIGH or LOW
#endif

#include <stdint.h>
#include <Matter.h>
#include <MatterTemperature.h>
#include "MatterFanCustom.h"

/// @brief Enum representing fan speed settings.
enum class FanState
{
    Off,
    Low,
    Med,
    High,
    On,
    Auto,
    Smart
};

/// @brief Pin the low speed relay is connected to.
const uint8_t kLowSpeedPin = D6; // Marked C1 on board
/// @brief Pin the medium speed relay is connected to.
const uint8_t kMediumSpeedPin = D8; // Marked D0 on board
/// @brief Pin the high speed relay is connected to.
const uint8_t kHighSpeedPin = D9; // Marked D1 on board

/// @brief Percentage to set the fan to when the low speed button is pressed.
const uint8_t kLowSpeed = 20;
/// @brief Percentage to set the fan to when the medium speed button is pressed.
const uint8_t kMediumSpeed = 50;
/// @brief Percentage to set the fan to when the high speed button is pressed.
const uint8_t kHighSpeed = 100;

#if BUTTONS
/// @brief Pin the off button is connected to. C5 on the board.
const uint8_t kOffButtonPin = A8;
/// @brief Pin the low speed button is connected to. C8 on the board.
const uint8_t kLowButtonPin = D0;
/// @brief Pin the medium speed button is connected to. C0 on the board.
const uint8_t kMediumButtonPin = D7;
/// @brief Pin the high speed button is connected to. D2 on the board.
const uint8_t kHighButtonPin = D10;
#endif

/// @brief Pin the temp sensor is connected to. A0 on the board.
const uint8_t kTempSensorPin = A5;

/// @brief Minimum interval, in milliseconds, between temp sensor readings
const unsigned long kTempSensorInterval = 10000UL;
/// @brief Reading of millis() last time the temp sensor was read
unsigned long previousTime = 0UL;

// Chosen analog reference
// AR_INTERNAL1V2 gives a -50 to 70 C range for the TMP36
#define AR_CHOICE AR_INTERNAL1V2

#if AR_CHOICE == AR_INTERNAL1V2
/// @brief Analog reference voltage, in millivolts.
/// Gives a -50 to 70 C range for the TMP36
const int16_t kARVoltage = 1200;
#elif AR_CHOICE == AR_EXTERNAL_1V25
/// @brief Analog reference voltage, in millivolts
/// Gives a -50 to 75 C range for the TMP36
const int16_t kARVoltage = 1250;
#endif


/// @brief Temp sensor calibrated temp in degrees C
const int16_t kTempSensorCalibrated = 25;
/// @brief Temp sensor scale in mV/C
const int16_t kTempSensorVScale = 10;
/// @brief Output voltage in millivolts of the temp sensor at the calibrated temperature
const int16_t kTempSensorVoltageCalibrated = 750;
/// @brief Temperature corresponding to a temp sensor reading of 0V
const int16_t kTempSensorTemp0V = kTempSensorCalibrated - kTempSensorVoltageCalibrated / kTempSensorVScale;
/// @brief Temperature corresponding to a temp sensor reading of kARVoltage
const int16_t kTempSensorTemp1V2 = kTempSensorCalibrated + (kARVoltage - kTempSensorVoltageCalibrated) / kTempSensorVScale;

/// @brief FanState the fan's hardware is set to.
volatile FanState fan_hardware_state = FanState::Off;

/// @brief Object exposed to Matter. Tracks software state of the fan.
MatterFanCustom matter_fan;
MatterTemperature matter_temp_sensor;
MatterTemperature matter_cpu_temp_sensor;

StaticSemaphore_t matter_device_event_semaphore_buf;
SemaphoreHandle_t matter_device_event_semaphore;

// Forward declarations
void updateFanState();
void setFanSpeed(FanState speed);
String fanStateToString(FanState state);
void updateTempSensor();
void updateCpuTempSensor();
void matterFanChangeCallback();

void setup()
{
  // Upon powering on, VUSB and 3V3 on the board will be hot,
  // and all of the relay controls will be LOW.
  // This means all of the relays will energise.
  // This will have two effects:
  // 1. A ~280mA current draw from VUSB.
  // 2. The fan will turn on.
  //
  // The latter issue could be avoided by routing the active 250VAC 
  // wire through the NC terminal on a relay such that all the others
  // get cut off when it energizes. If the current draw doesn't cause
  // issues, this is probably the best course of action.
  
  // Pin setup
  
  pinMode(kLowSpeedPin, OUTPUT);
  pinMode(kMediumSpeedPin, OUTPUT);
  pinMode(kHighSpeedPin, OUTPUT);

  setFanSpeed(FanState::Off);

  #if BUTTONS
  // INPUT_PULLUP so we can reuse the existing buttons directly
  pinMode(kOffButtonPin, INPUT_PULLUP);
  pinMode(kLowButtonPin, INPUT_PULLUP);
  pinMode(kMediumButtonPin, INPUT_PULLUP);
  pinMode(kHighButtonPin, INPUT_PULLUP);

  // Mechanical fan state will only change when a setting has actually changed,
  // so it's no problem for these to be triggered multiple times in a row, and 
  // we don't really need debouncing.
  attachInterrupt(digitalPinToInterrupt(kOffButtonPin), offInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(kLowButtonPin), lowSpeedInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(kMediumButtonPin), mediumSpeedInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(kHighButtonPin), highSpeedInterrupt, FALLING);

  // Disable interrupts while we set up Matter
  noInterrupts();
  #endif

  analogReference(AR_INTERNAL1V2);

  // Create a binary semaphore
  matter_device_event_semaphore = xSemaphoreCreateBinaryStatic(&matter_device_event_semaphore_buf);

  // Matter setup

  Serial.begin(115200);
  Matter.begin();
  matter_fan.begin();
  matter_temp_sensor.begin();
  matter_cpu_temp_sensor.begin();

  matter_fan.set_device_name("Matter Fan");
  matter_fan.set_vendor_name("Homemaker/Sparkfun");
  matter_fan.set_product_name("HMAWP-4097/Thing Plus Matter");
  matter_fan.set_serial_number(getDeviceUniqueIdStr().c_str());

  matter_temp_sensor.set_device_name("Room Temperature Sensor");
  matter_temp_sensor.set_vendor_name("Analog Instruments/Sparkfun");
  matter_temp_sensor.set_product_name("TMP36/Thing Plus Matter");
  matter_temp_sensor.set_serial_number(getDeviceUniqueIdStr().c_str());

  matter_cpu_temp_sensor.set_device_name("CPU Temperature Sensor");
  matter_cpu_temp_sensor.set_vendor_name("Sparkfun");
  matter_cpu_temp_sensor.set_product_name("Thing Plus Matter");
  matter_cpu_temp_sensor.set_serial_number(getDeviceUniqueIdStr().c_str());

  matter_fan.set_device_change_callback(matterFanChangeCallback);

  #if DEBUG | DEBUG_MATTER
  Serial.println("Matter fan and temperature sensor");

  if (!Matter.isDeviceCommissioned()) {
    Serial.println("Matter device is not commissioned");
    Serial.println("Commission it to your Matter hub with the manual pairing code or QR code");
    Serial.printf("Manual pairing code: %s\n", Matter.getManualPairingCode().c_str());
    Serial.printf("QR code URL: %s\n", Matter.getOnboardingQRCodeUrl().c_str());
  }
  #endif
  while (!Matter.isDeviceCommissioned()) {
    delay(200);
  }

  #if DEBUG | DEBUG_MATTER
  Serial.println("Waiting for Thread network...");
  #endif
  while (!Matter.isDeviceThreadConnected()) {
    delay(200);
  }
  #if DEBUG | DEBUG_MATTER
  Serial.println("Connected to Thread network");

  Serial.println("Waiting for Matter device discovery...");
  #endif
  while (!matter_fan.is_online() | !matter_temp_sensor.is_online() | !matter_cpu_temp_sensor.is_online()) {
    delay(200);
  }
  #if DEBUG | DEBUG_MATTER
  Serial.println("Matter device is now online");
  #endif

  #if BUTTONS
  // Re-enable interrupts now everything is set up
  interrupts();
  #endif
}

void loop()
{
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - previousTime;
  
  if (elapsedTime >= kTempSensorInterval)
  {
    // Temp sensor polling interval has already expired.
    #if DEBUG
    Serial.println("Temp sensor interval elapsed in loop");
    #endif
    
    // xBlockTime of 0, then immediately poll the sensor
    if (xSemaphoreTake(matter_device_event_semaphore, 0)  == pdTRUE)
    {
      #if DEBUG
      Serial.println("Semaphore taken. Updating fan mode.");
      #endif
      updateFanState();
    }

    updateTempSensor();
    updateCpuTempSensor();
    previousTime = millis();
  }
  else
  {
    // Temp sensor polling interval is yet to expire.

    #if DEBUG
    Serial.println("Temp sensor interval not yet elapsed");
    #endif

    // xBlockTime equal to the remaining time in the interval
    TickType_t xTempSensorTicksToWait = pdMS_TO_TICKS(kTempSensorInterval - elapsedTime);
    if (xSemaphoreTake(matter_device_event_semaphore, xTempSensorTicksToWait) == pdTRUE)
    {
      // Semaphore taken means we've resumed before the interval expired
      #if DEBUG
      Serial.println("Semaphore taken. Updating fan mode.");
      #endif
      updateFanState();
    }
    else
    {
      // Semaphore not taken means the interval has expired
      #if DEBUG
      Serial.println("Temp sensor interval elapsed while awaiting semaphore.");
      #endif
      updateTempSensor();
      updateCpuTempSensor();
      previousTime = millis();
    }
  }
}

/// @brief Synchronise the physical fan's on/off state with that of the Matter fan
void updateFanState()
{
    FanState matter_fan_mode = (FanState)matter_fan.get_mode();

    if (matter_fan_mode != fan_hardware_state) {
      setFanSpeed(matter_fan_mode);
      fan_hardware_state = matter_fan_mode;
    }
}

/// @brief Set the relays to obtain the desired fan speed.
/// @param speed The speed to set the fan to, or FanState::Off to turn it off.
void setFanSpeed(FanState speed)
{
  switch (speed)
  {
  case FanState::Low:
    #if DEBUG
    Serial.println("Relays set for low speed");
    #endif
    digitalWrite(kMediumSpeedPin, RELAY_INACTIVE);
    digitalWrite(kHighSpeedPin, RELAY_INACTIVE);
    digitalWrite(kLowSpeedPin, RELAY_ACTIVE);
    break;
  case FanState::Med:
    #if DEBUG
    Serial.println("Relays set for medium speed");
    #endif
    digitalWrite(kLowSpeedPin, RELAY_INACTIVE);
    digitalWrite(kHighSpeedPin, RELAY_INACTIVE);
    digitalWrite(kMediumSpeedPin, RELAY_ACTIVE);
    break;
  case FanState::High:
    #if DEBUG
    Serial.println("Relays set for high speed");
    #endif
    digitalWrite(kLowSpeedPin, RELAY_INACTIVE);
    digitalWrite(kMediumSpeedPin, RELAY_INACTIVE);
    digitalWrite(kHighSpeedPin, RELAY_ACTIVE);
    break;
  case FanState::Off:
  default:
    #if DEBUG
    Serial.println("Relays turned off");
    #endif
    digitalWrite(kLowSpeedPin, RELAY_INACTIVE);
    digitalWrite(kMediumSpeedPin, RELAY_INACTIVE);
    digitalWrite(kHighSpeedPin, RELAY_INACTIVE);
    break;
  }
}

String fanStateToString(FanState state)
{
  switch (state)
  {
  case FanState::Off:
    return "Off";
  case FanState::Low:
    return "Low";
  case FanState::Med:
    return "Med";
  case FanState::High:
    return "High";
  case FanState::On:
    return "On";
  case FanState::Auto:
    return "Auto";
  case FanState::Smart:
    return "Smart";
  default:
    return "Unknown state (" + (String)(uint8_t)state + ")";
  }
}

void matterFanChangeCallback()
{
  #if DEBUG
  Serial.println("Matter change callback triggered.");
  #endif
  xSemaphoreGive(matter_device_event_semaphore);
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

float readTempSensorRaw()
{
  // TMP36 has a scale factor of 10mV/C
  // and outputs 750mV at 25C.
  // The 0 to 3.3V range readable on the analog input pins
  // then maps to temperatures from -50 to 280.

  int rawInput = analogRead(kTempSensorPin);
  #if DEBUG
  Serial.print("Raw temp input: ");
  Serial.println(rawInput);
  #endif

  // Temperatures scaled by 100 so we can plug 
  // them straight into set_measured_value_raw
  return mapfloat(rawInput, 0, 4096,
    kTempSensorTemp0V * 100, kTempSensorTemp1V2 * 100);
}

void updateTempSensor()
{
  #if DEBUG
  Serial.println("Measuring temperature.");
  #endif
  matter_temp_sensor.set_measured_value_raw(readTempSensorRaw());
  #if DEBUG
  Serial.print("Temperature: ");
  Serial.print(matter_temp_sensor.get_measured_value_celsius());
  Serial.println(" C");
  #endif
}

void updateCpuTempSensor()
{
  #if DEBUG
  Serial.println("Measuring CPU temperature.");
  #endif
  matter_cpu_temp_sensor.set_measured_value_celsius(getCPUTemp());
  #if DEBUG
  Serial.print("Temperature: ");
  Serial.print(matter_cpu_temp_sensor.get_measured_value_celsius());
  Serial.println(" C");
  #endif
}

#if BUTTONS
// These interrupts all set the percent setting of
// the Matter fan. This should trip the fan change
// callback

void offInterrupt()
{
    #if DEBUG
    Serial.println("Off button interrupt triggered.");
    #endif
    matter_fan.set_percent(0);
}

void lowSpeedInterrupt()
{
    #if DEBUG
    Serial.println("Low speed button interrupt triggered.");
    #endif
    matter_fan.set_percent(kLowSpeed);
}

void mediumSpeedInterrupt()
{
    #if DEBUG
    Serial.println("Medium speed button interrupt triggered.");
    #endif
    matter_fan.set_percent(kMediumSpeed);
}

void highSpeedInterrupt()
{
    #if DEBUG
    Serial.println("High speed button interrupt triggered.");
    #endif
    matter_fan.set_percent(kHighSpeed);
}
#endif
