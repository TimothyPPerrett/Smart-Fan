/*
   Adapted from matter_fan.ino (https://github.com/SiliconLabs/arduino/blob/main/libraries/Matter/examples/matter_fan/matter_fan.ino)
   Original Author: Tamas Jozsi (Silicon Labs)

   Modified By: Timothy Perrett
 */
// #define DEBUG 1
#define DEBUG_MATTER 1
// #define BUTTONS 1

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

/// @brief Pin the low speed relay is connected to.
const uint8_t kLowSpeedPin = D6;
/// @brief Pin the medium speed relay is connected to.
const uint8_t kMediumSpeedPin = D7;
/// @brief Pin the high speed relay is connected to.
const uint8_t kHighSpeedPin = D8;

#if BUTTONS
/// @brief Percentage to set the fan to when the low speed button is pressed.
const uint8_t kLowSpeed = 0;
/// @brief Percentage to set the fan to when the medium speed button is pressed.
const uint8_t kMediumSpeed = 50;
/// @brief Percentage to set the fan to when the high speed button is pressed.
const uint8_t kHighSpeed = 100;

/// @brief Pin the off button is connected to.
const uint8_t kOffButtonPin = A0;
/// @brief Pin the low speed button is connected to.
const uint8_t kLowButtonPin = A1;
/// @brief Pin the medium speed button is connected to.
const uint8_t kMediumButtonPin = A2;
/// @brief Pin the high speed button is connected to.
const uint8_t kHighButtonPin = A3;
#endif

MatterFan matter_fan;

void setup()
{
  // Pin setup

  pinMode(kLowSpeedPin, OUTPUT);
  pinMode(kMediumSpeedPin, OUTPUT);
  pinMode(kHighSpeedPin, OUTPUT);

  #if BUTTONS
  pinMode(kOffButtonPin, INPUT_PULLUP);
  pinMode(kLowButtonPin, INPUT_PULLUP);
  pinMode(kMediumButtonPin, INPUT_PULLUP);
  pinMode(kHighButtonPin, INPUT_PULLUP);
  #endif

  // Matter setup

  Serial.begin(115200);
  Matter.begin();
  matter_fan.begin();

  #if DEBUG | DEBUG_MATTER
  Serial.println("Matter fan");
  #endif

  if (!Matter.isDeviceCommissioned()) {
    #if DEBUG | DEBUG_MATTER
    Serial.println("Matter device is not commissioned");
    Serial.println("Commission it to your Matter hub with the manual pairing code or QR code");
    Serial.printf("Manual pairing code: %s\n", Matter.getManualPairingCode().c_str());
    Serial.printf("QR code URL: %s\n", Matter.getOnboardingQRCodeUrl().c_str());
    #endif
  }
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
  #endif

  #if DEBUG | DEBUG_MATTER
  Serial.println("Waiting for Matter device discovery...");
  #endif
  while (!matter_fan.is_online()) {
    delay(200);
  }
  #if DEBUG | DEBUG_MATTER
  Serial.println("Matter device is now online");
  #endif
}

void loop()
{
  static uint8_t fan_last_percent = 0;
  static FanSpeed fan_last_speed = FanSpeed::Low;
  uint8_t fan_current_percent = matter_fan.get_percent();

  if (fan_current_percent != fan_last_percent) {
    fan_last_percent = fan_current_percent;
    fan_last_speed = percentToFanSpeed(fan_last_percent);
    setFanSpeed(fan_last_speed);
    #if DEBUG
    Serial.print("Fan speed: ");
    Serial.print(fan_current_percent);
    Serial.println("%");
    #endif
  }

  static bool fan_last_state = false;
  bool fan_current_state = matter_fan.get_onoff();

  if (fan_current_state != fan_last_state) {
    fan_last_state = fan_current_state;
    if (fan_current_state) {
      setFanSpeed(fan_last_speed);
      #if DEBUG
      Serial.println("Fan ON");
      #endif
    } else {
      setFanSpeed(FanSpeed::Off);
      #if DEBUG
      Serial.println("Fan OFF");
      #endif
    }
  }

  #if BUTTONS
  if (digitalRead(kOffButtonPin) == LOW) {
    if (fan_current_state)
    {
      matter_fan.set_onoff(false);
      fan_last_state = false;
      setFanSpeed(FanSpeed::Off);
      #if DEBUG
      Serial.println("Fan OFF");
      #endif
    }
  } else if (digitalRead(kLowButtonPin) == LOW) {
    // If we're changing speed OR turning on
    if (fan_last_percent != kLowSpeed || !fan_last_state)
    {
      // Set speed to Low and percent to kLowSpeed
      fan_last_speed = FanSpeed::Low;
      fan_last_percent = kLowSpeed;
      
      matter_fan.set_percent(fan_last_percent);
      if (!fan_last_state) {
        matter_fan.set_onoff(true);
        fan_last_state = true;
        #if DEBUG
        Serial.println("Fan ON");
        #endif
      }
      setFanSpeed(fan_last_speed);
      #if DEBUG
      Serial.print("Fan speed: ");
      Serial.print(fan_current_percent);
      Serial.println("%");
      #endif
    }
  } else if (digitalRead(kMediumButtonPin) == LOW) {
    // If we're changing speed OR turning on
    if (fan_last_percent != kMediumSpeed || !fan_last_state)
    {
      // Set speed to Medium and percent to kMediumSpeed
      fan_last_speed = FanSpeed::Medium;
      fan_last_percent = kMediumSpeed;
      
      matter_fan.set_percent(fan_last_percent);
      if (!fan_last_state) {
        matter_fan.set_onoff(true);
        fan_last_state = true;
        #if DEBUG
        Serial.println("Fan ON");
        #endif
      }
      setFanSpeed(fan_last_speed);
      #if DEBUG
      Serial.print("Fan speed: ");
      Serial.print(fan_current_percent);
      Serial.println("%");
      #endif
    }
  } else if (digitalRead(kHighButtonPin) == LOW) {
    // If we're changing speed OR turning on
    if (fan_last_percent != kHighSpeed || !fan_last_state)
    {
      // Set speed to High and percent to kHighSpeed
      fan_last_speed = FanSpeed::High;
      fan_last_percent = kHighSpeed;
      
      matter_fan.set_percent(fan_last_percent);
      if (!fan_last_state) {
        matter_fan.set_onoff(true);
        fan_last_state = true;
        #if DEBUG
        Serial.println("Fan ON");
        #endif
      }
      setFanSpeed(fan_last_speed);
      #if DEBUG
      Serial.print("Fan speed: ");
      Serial.print(fan_current_percent);
      Serial.println("%");
      #endif
    }
  }
  #endif
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
    digitalWrite(kMediumSpeedPin, HIGH);
    digitalWrite(kHighSpeedPin, HIGH);
    digitalWrite(kLowSpeedPin, LOW);
    break;
  case FanSpeed::Medium:
    digitalWrite(kLowSpeedPin, HIGH);
    digitalWrite(kHighSpeedPin, HIGH);
    digitalWrite(kMediumSpeedPin, LOW);
    break;
  case FanSpeed::High:
    digitalWrite(kLowSpeedPin, HIGH);
    digitalWrite(kMediumSpeedPin, HIGH);
    digitalWrite(kHighSpeedPin, LOW);
  case FanSpeed::Off:
  default:
    digitalWrite(kLowSpeedPin, HIGH);
    digitalWrite(kMediumSpeedPin, HIGH);
    digitalWrite(kHighSpeedPin, HIGH);
    break;
  }
}