/*
   Adapted from matter_fan.ino (https://github.com/SiliconLabs/arduino/blob/main/libraries/Matter/examples/matter_fan/matter_fan.ino)
   Original Author: Tamas Jozsi (Silicon Labs)

   Modified By: Timothy Perrett
 */

// If defined, debug messages are printed to Serial
#define DEBUG 1

// If defined, Matter setup debug messages are printed to Serial
#define DEBUG_MATTER 1

// If defined, button inputs and interrupts are used
// #define BUTTONS 1

// If defined, the Matter fan is polled at the specified interval
// #define SMART_FAN_POLLING 1

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
#include <MatterFan.h>

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

#if SMART_FAN_POLLING
/// @brief Interval in milliseconds at which the matter fan is polled.
const unsigned long kPollingInterval = 100UL;
#endif

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

/// @brief Percentage the fan's hardware is set to.
volatile uint8_t fan_hardware_percent = 0;
/// @brief FanState the fan's hardware is set to.
volatile FanState fan_hardware_state = FanState::Off;

/// @brief Object exposed to Matter. Tracks software state of the fan.
MatterFan matter_fan;

// Forward declarations
void updateFanState();
void setHardwareState(FanState state, bool override_percent = false);
FanState percentToFanState(uint8_t percent);
void setFanSpeed(FanState speed);
String fanStateToString(FanState state);

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
  
  if (RELAY_INACTIVE == HIGH) {
    // This sets relay control pins to HIGH by default
    // to minimize how long we spend with all the relay coils active
    pinMode(kLowSpeedPin, INPUT_PULLUP);
    pinMode(kMediumSpeedPin, INPUT_PULLUP);
    pinMode(kHighSpeedPin, INPUT_PULLUP);
  }

  // Now we set them to be OUTPUTs
  pinMode(kLowSpeedPin, OUTPUT);
  pinMode(kMediumSpeedPin, OUTPUT);
  pinMode(kHighSpeedPin, OUTPUT);

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

  // Matter setup

  Serial.begin(115200);
  Matter.begin();
  matter_fan.begin();

  matter_fan.set_device_name("Matter Fan");
  matter_fan.set_vendor_name("Homemaker/Sparkfun");
  matter_fan.set_product_name("HMAWP-4097/Thing Plus Matter");
  matter_fan.set_serial_number(getDeviceUniqueIdStr().c_str());

  #if DEBUG | DEBUG_MATTER
  Serial.println("Matter fan");

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
  while (!matter_fan.is_online()) {
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
  #if SMART_FAN_POLLING
  // Enforce polling interval.
  // TODO: Will the underflow that happens every ~50 days break anything?
  static unsigned long time = 0UL;
  if (millis() >= time + kPollingInterval) {
  #endif
  
  // TODO: Any need to disable interrupts while we perform these updates?
  // If so, should that be done here or in the functions themselves?
  updateFanState();

  #if SMART_FAN_POLLING
  }
  time = millis();
  #endif
}

/// @brief Synchronise the physical fan's on/off state with that of the Matter fan
void updateFanState()
{
    bool matter_fan_state = matter_fan.get_onoff();
    uint8_t matter_fan_percent = matter_fan.get_percent();
    FanState matter_fan_speed_state;

    if (matter_fan_percent != fan_hardware_percent) {
      #if DEBUG
      Serial.print("Matter fan percent changed from ");
      Serial.print(fan_hardware_percent);
      Serial.print("% to ");
      Serial.print(matter_fan_percent);
      Serial.println("%");
      #endif
      matter_fan_speed_state = percentToFanState(matter_fan_percent);
      if (matter_fan_speed_state == fan_hardware_state)
      {
        fan_hardware_percent = matter_fan_percent;
      }
    }
    else
    {
      matter_fan_speed_state = fan_hardware_state;
    }

    
    if (matter_fan_speed_state != fan_hardware_state | (matter_fan_state != (fan_hardware_state != FanState::Off))) {
      #if DEBUG
      Serial.println("Matter fan state change!");
      
      Serial.print("Hardware fan state: ");
      Serial.println(fanStateToString(fan_hardware_state));

      Serial.print("Hardware fan percent: ");
      Serial.print(fan_hardware_percent);
      Serial.println("%");

      Serial.print("Matter fan On/Off: ");
      Serial.println((matter_fan_state) ? "On" : "Off");

      Serial.print("Matter fan percent: ");
      Serial.print(matter_fan_percent);
      Serial.print("% (");
      Serial.print(fanStateToString(matter_fan_speed_state));
      Serial.println(")");
      
      #endif
    } 
    else
    {
      // No changes.
      return;
    }
    

    // Special cases, turning off the fan when it's on, and on when it's off
    if (matter_fan_state)
    {
      if (fan_hardware_state == FanState::Off && matter_fan_speed_state == FanState::Off)
      {
        setHardwareState(FanState::On);
        return;
      }
    } else
    {
      if (fan_hardware_state != FanState::Off) {
        setHardwareState(FanState::Off);
        return;
      }
    }

    setHardwareState(matter_fan_speed_state);
    fan_hardware_percent = matter_fan.get_percent();
}

void setHardwareState(FanState state, bool override_percent)
{
  // Act only if we're changing the hardware state.
  if (state != fan_hardware_state)
  {
    switch (state)
    {
    case FanState::Off:
      matter_fan.set_percent(0);
      matter_fan.set_onoff(false);
      setFanSpeed(FanState::Off);
      break;
    case FanState::Low:
      matter_fan.set_onoff(true);
      if (override_percent) {
        matter_fan.set_percent(kLowSpeed);
      }
      setFanSpeed(FanState::Low);
      break;
    case FanState::Med:
      matter_fan.set_onoff(true);
      if (override_percent) {
        matter_fan.set_percent(kMediumSpeed);
      }
      setFanSpeed(FanState::Med);
      break;
    case FanState::High:
      matter_fan.set_onoff(true);
      if (override_percent) {
        matter_fan.set_percent(kHighSpeed);
      }
      setFanSpeed(FanState::High);
      break;
    case FanState::On:
      // Specification states to set to HIGH
      #if DEBUG
      Serial.println("Fan state is being set to On. Setting to High.");
      #endif
      setHardwareState(FanState::High, true);
      return;
    case FanState::Auto:
      // Not implementing
      #if DEBUG
      Serial.println("Setting fan state to Auto is not supported.");
      #endif
      return;
    case FanState::Smart:
      // Auto is not implemented,
      // so specification says to set to HIGH
      #if DEBUG
      Serial.println("Fan state is being set to Smart. Setting to High.");
      #endif
      setHardwareState(FanState::High, true);
      return;
    default:
      #if DEBUG
      Serial.print("Tried to set fan state to ");
      Serial.println(fanStateToString(state));
      #endif
      return;
    }
    
    #if DEBUG
    Serial.print("Fan state changed from ");
    Serial.print(fanStateToString(fan_hardware_state));
    Serial.print(" to ");
    Serial.println(fanStateToString(state));
    #endif
    fan_hardware_state = state;
  }
}

/// @brief Converts a percentage to a FanState
/// @param percent A value in the inclusive range 0-100
/// @return A FanState corresponding to percent, from FanState::Low to FanState::High
FanState percentToFanState(uint8_t percent)
{
    if (percent == 0) {
        return FanState::Off;
    } else if (percent <= 33) {
        return FanState::Low;
    } else if (percent > 66) {
        return FanState::High;
    } else {
        return FanState::Med;
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

#if BUTTONS
// These interrupts all set the state of
// the Matter fan, then call updateFanSpeed
// to propagate that state to the hardware.

void offInterrupt()
{
    #if DEBUG
    Serial.println("Off button interrupt triggered.");
    #endif
    noInterrupts();
    matter_fan.set_onoff(false);
    updateFanState();
    interrupts();
}

void lowSpeedInterrupt()
{
    #if DEBUG
    Serial.println("Off button interrupt triggered.");
    #endif
    noInterrupts();
    matter_fan.set_onoff(true);
    matter_fan.set_percent(kLowSpeed);
    updateFanSpeed();
    interrupts();
}

void mediumSpeedInterrupt()
{
    #if DEBUG
    Serial.println("Medium speed button interrupt triggered.");
    #endif
    noInterrupts();
    matter_fan.set_onoff(true);
    matter_fan.set_percent(kMediumSpeed);
    updateFanSpeed();
    interrupts();
}

void highSpeedInterrupt()
{
    #if DEBUG
    Serial.println("High speed button interrupt triggered.");
    #endif
    noInterrupts();
    matter_fan.set_onoff(true);
    matter_fan.set_percent(kHighSpeed);
    updateFanSpeed();
    interrupts();
}
#endif
