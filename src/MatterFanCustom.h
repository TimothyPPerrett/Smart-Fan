/*
 * This file has been modified from MatterFan.h, 
 * a part of the Silicon Labs Arduino Core
 *
 * The MIT License (MIT)
 *
 * Copyright 2024 Silicon Laboratories Inc. www.silabs.com
 * Copyright 2024 Timothy Perrett
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef MATTER_FAN_CUSTOM_H
#define MATTER_FAN_CUSTOM_H

#include <MatterFan.h>
#include "DeviceFanCustom.h"

using namespace chip;
using namespace ::chip::DeviceLayer;

class MatterFanCustom : public ArduinoMatterAppliance {
public:
  MatterFanCustom();
  ~MatterFanCustom();
  bool begin();
  void end();
  void set_onoff(bool value);
  bool get_onoff();
  void set_percent(uint8_t percent);
  uint8_t get_percent();
  void operator=(uint8_t percent);
  static EmberAfAttributeMetadata fanControlAttrs[9];
  static EmberAfCluster fanControlEndpointClusters[3];
  uint8_t get_mode();

private:
  DeviceFanCustom* fan_device;
  EmberAfEndpointType* device_endpoint;
  DataVersion* endpoint_dataversion_storage;
  bool initialized;
};

#endif // MATTER_FAN_CUSTOM_H
