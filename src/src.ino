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
enum class FanSpeed
{
    Off,
    Low,
    Medium,
    High
};

#if SMART_FAN_POLLING
/// @brief Interval in milliseconds at which the matter fan is polled.
const unsigned long kPollingInterval = 100UL;
#endif

/// @brief Pin the low speed relay is connected to.
const uint8_t kLowSpeedPin = D6; // Marked C1 on board
/// @brief Pin the medium speed relay is connected to.
const uint8_t kMediumSpeedPin = D7; // Marked C0 on board
/// @brief Pin the high speed relay is connected to.
const uint8_t kHighSpeedPin = D8; // Marked D0 on board

#if BUTTONS
/// @brief Percentage to set the fan to when the low speed button is pressed.
const uint8_t kLowSpeed = 20;
/// @brief Percentage to set the fan to when the medium speed button is pressed.
const uint8_t kMediumSpeed = 50;
/// @brief Percentage to set the fan to when the high speed button is pressed.
const uint8_t kHighSpeed = 100;

/// @brief Pin the off button is connected to.
const uint8_t kOffButtonPin = A5;
/// @brief Pin the low speed button is connected to.
const uint8_t kLowButtonPin = A6;
/// @brief Pin the medium speed button is connected to.
const uint8_t kMediumButtonPin = A7;
/// @brief Pin the high speed button is connected to.
const uint8_t kHighButtonPin = A8;
#endif

/// @brief Percentage that matter_fan was set to last time updateFanSpeed was called.
volatile uint8_t fan_last_percent = 0;
/// @brief FanSpeed corresponding to fan_last_percent.
volatile FanSpeed fan_last_speed = FanSpeed::Low;
/// @brief On/off state of matter_fan last time updateFanState was called.
volatile bool fan_last_state = false;

/// @brief Object exposed to Matter. Tracks software state of the fan.
MatterFan matter_fan;

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
  // Re-enable interrupts now Matter is set up
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
  updateFanSpeed();
  updateFanState();

  #if SMART_FAN_POLLING
  }
  time = millis();
  #endif
}

/// @brief Synchronise the physical fan's speed setting with that of the Matter fan
void updateFanSpeed()
{
    uint8_t fan_current_percent = matter_fan.get_percent();
    // Act only if the value has changed...
    if (fan_current_percent != fan_last_percent) {
        fan_last_percent = fan_current_percent;
        #if DEBUG
        Serial.print("Fan percent changed to ");
        Serial.print(fan_current_percent);
        Serial.println("%");
        #endif
        FanSpeed fan_current_speed = percentToFanSpeed(fan_last_percent);
        // ...AND that value means a different speed...
        if (fan_current_speed != fan_last_speed) {
            #if DEBUG
            Serial.print("Fan speed changed to ");
            Serial.println((uint8_t)fan_current_speed);
            #endif
            fan_last_speed = fan_current_speed;
            // ... AND the fan is on.
            if (matter_fan.get_onoff()) {
                setFanSpeed(fan_last_speed);
            }
        }
    }
}

/// @brief Synchronise the physical fan's on/off state with that of the Matter fan
void updateFanState()
{
    bool fan_current_state = matter_fan.get_onoff();

    if (fan_current_state != fan_last_state) {
        fan_last_state = fan_current_state;
        if (fan_current_state) {
        setFanSpeed(fan_last_speed);
        #if DEBUG
        Serial.println("Fan state changed to ON");
        #endif
        } else {
        setFanSpeed(FanSpeed::Off);
        #if DEBUG
        Serial.println("Fan state changed to OFF");
        #endif
        }
    }
}

/// @brief Converts a percentage to a FanSpeed
/// @param percent A value in the inclusive range 0-100
/// @return A FanSpeed corresponding to percent, from FanSpeed::Low to FanSpeed::High
FanSpeed percentToFanSpeed(uint8_t percent)
{
    if (percent <= 33) {
        return FanSpeed::Low;
    } else if (percent > 66)
    {
        return FanSpeed::High;
    } else {
        return FanSpeed::Medium;
    }
    
}

/// @brief Set the relays to obtain the desired fan speed.
/// @param speed The speed to set the fan to, or FanSpeed::Off to turn it off.
void setFanSpeed(FanSpeed speed)
{
  switch (speed)
  {
  case FanSpeed::Low:
    #if DEBUG
    Serial.println("Relays set for low speed");
    #endif
    digitalWrite(kMediumSpeedPin, RELAY_INACTIVE);
    digitalWrite(kHighSpeedPin, RELAY_INACTIVE);
    digitalWrite(kLowSpeedPin, RELAY_ACTIVE);
    break;
  case FanSpeed::Medium:
    #if DEBUG
    Serial.println("Relays set for medium speed");
    #endif
    digitalWrite(kLowSpeedPin, RELAY_INACTIVE);
    digitalWrite(kHighSpeedPin, RELAY_INACTIVE);
    digitalWrite(kMediumSpeedPin, RELAY_ACTIVE);
    break;
  case FanSpeed::High:
    #if DEBUG
    Serial.println("Relays set for high speed");
    #endif
    digitalWrite(kLowSpeedPin, RELAY_INACTIVE);
    digitalWrite(kMediumSpeedPin, RELAY_INACTIVE);
    digitalWrite(kHighSpeedPin, RELAY_ACTIVE);
  case FanSpeed::Off:
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
