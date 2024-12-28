# Smart Fan
This repository covers my personal project to convert the Homemaker HMAWP-4097 pedestal fan I've been using into a Matter-enabled smart fan using the SparkFun Thing Plus Matter.

## Methodology
### Software
#### Classes
The fan uses a modified version of the [official Arduino Matter library](https://github.com/SiliconLabs/arduino/tree/main/libraries/Matter) in the form of the `MatterFanCustom` and `DeviceFanCustom` classes.

##### MatterFanCustom Class
The `MatterFan` class from the official library is mostly unaltered, except for its use of `DeviceFanCustom` and the addition of the `get_mode` method.

`uint8_t get_mode();` Retrieves the fan's current mode setting. Value returned corresponds to the values of the FanModeEnum type outlined in section 4.4.5.5 of the Matter Application Cluster Specification Version 1.3.0.1.

##### DeviceFanCustom Class
The `DeviceFanCustom` class is a more significant rewrite of `DeviceFan`, though it introduces no new methods. Most basically, the custom class changes the advertised fan mode sequence to be Off/Low/Med/High, rather than Off/Low/Med/High/Auto.

`SetPercentSetting` is altered to call `SetFanMode` to set the fan to the appropriate mode after setting the percentage.

`SetFanMode` has been heavily rewritten to correctly handle cases where the fan mode was set to values that weren't Low/Med/High. In addition, it has been given an additional, optional parameter, `set_percent` (which defaults to `true` for consistency with its previous behavior) to allow setting the fan mode without changing the current percentage setting.

#### Fan Control
The "backbone" of the fan control sketch ([`src.ino`](src/src.ino)) was originally the [matter_fan example from the Matter library](https://github.com/SiliconLabs/arduino/tree/b0492674316ec6505a474155a2bbc3b3e9446417/libraries/Matter/examples/matter_fan), and most of the Matter setup code has been included from there unaltered. [matter_lightbulb_custom_name](https://github.com/SiliconLabs/arduino/blob/b0492674316ec6505a474155a2bbc3b3e9446417/libraries/Matter/examples/matter_lightbulb_custom_name/matter_lightbulb_custom_name.ino) was used as reference for the custom device data settings at the beginning of the Matter setup.

Control is achieved through the use of FreeRTOS semaphores, based on the [matter_lightbulb_callback](https://github.com/SiliconLabs/arduino/blob/b0492674316ec6505a474155a2bbc3b3e9446417/libraries/Matter/examples/matter_lightbulb_callback/matter_lightbulb_callback.ino) example. This should lead to lower power consumption than polling the Matter device. 

The buttons already on the fan will be fully incorporated into the control of the smart fan. Pressing each one will trigger an interrupt that sets the Matter fan's percentage setting to the appropriate value. This in turn should trigger the callback that gives the semaphore.

### Hardware
In the unaltered fan, speed/power control is achieved by pressing one of the four buttons on the front (high, medium and low speeds, and off). These buttons latch in place and complete a connection between the mains active wire and one of three other wires that are connected somewhere in the fan motor unit (the off button is purely mechanical, and unlatches whichever button is currently pressed). To retrofit this for Matter control is simple - disconnect the button panel and instead use relays to select which wire is connected. In this project I used an HW-316 4 Relay Module with the JD-VCC jumper removed and the JD-VCC pin connected to VUSB of the Thing Plus Matter, and the relay channels controlled using the Thing Plus Matter's standard GPIO pins.

The button panel will be reused as a manual controller. Each button's connection will be wired to a GPIO pin in `INPUT_PULLUP` mode, and the terminal previously used for the active wire will be connected to ground. When these buttons are pressed, the pin will be brought low and an interrupt triggered to set the fan to the appropriate speed.

## Resources
![Thing Plus Matter Pinout](https://github.com/SiliconLabs/arduino/raw/main/doc/sparkfunthingplusmatter_pinout.png)

Source: https://github.com/SiliconLabs/arduino
