# Touch Advanced Usermod for WLED - version 0.8 (beta)

Provides esp32 touch functionality with auto calibration, pin-specific settings and extended functionality.

### Currently there are five button modes:

* Digital Button (Mode 1 - 127) - up to 127 actions per button, including dimming on long touch
* Analog Value (Mode 131) - when touched, sets an analog value depending on touch strength
* Analog Weighted (Mode 141) - when touched, sets a fixed analog value, multiple such buttons are weighted depending on touch strength, can be used for (absolute) sliders
* Analog Wheel (Mode 151) - when touched, provides an angle, multiple such buttons are combined depending their touch strength, finger movement allows to scroll a virtual wheel, can be used for virtual knobs, wheels
* Digital Switch (Mode 171) - similar to button, but has 2 stable states and toggle actions

The usermod is still under development, settings might change, expect bugs.

Bug reports, feature requests: https://github.com/bernis/WLED

## Installation

### Option A

Take the WLED folder from this repository

### Option B

* Copy the `/touch_advanced/` folder to your `/WLED/usermods/`
* In your `WLED/wled00/usermods_list.cpp`, add this to the list of usermods at the top of the file:
 
      #ifdef  USERMOD_TOUCHADVANCED
      #include  "../usermods/touch_advanced/usermod_v2_touchadvanced.h"
      #endif
      
  And this to the second list at the bottom of the file:

      #ifdef  USERMOD_TOUCHADVANCED
      usermods.add(new TouchAdvancedUsermod());
      #endif

### To enable
Either add a line with `#define USERMOD_TOUCHADVANCED` somewhere, e.g. in `WLED/wled00/my_config.h`
Or add `-D USERMOD_TOUCHADVANCED` to your `WLED/platformio_override.ini`

## Manual

### Basic principle: esp32 touch functionality

The esp32 has 10 pins capable of touch input. To read a pin, it repeatedly charges it to a upper threshold voltage and then discharges it to a lower threshold voltage. It does so for a set measurement time, and counts the number of cycles it completed. The time it takes to charge or discharge a pin depends on the capacitance of the pin, which increases if a touchpad is connected, and increase further if your finger is near the pad.  

More infos on the hardware and touch pad design: https://github.com/espressif/esp-iot-solution/blob/release/v1.0/documents/touch_pad_solution/touch_sensor_design_en.md

#### WLED touch implementation

WLED uses the Arduino touchRead() functionality to read the cycle count for a pin and compares it to a threshold value set in the config. The pin is otherwise treated like a momentary switch to allow digital touch input.

#### Touch advanced implementation

TA increases the measurement time from the default value of 0x1000 (4096) clock cycles (0.5ms), to the maximum of 0xffff (65535) clock cycles (7.8ms). The sleep time between measurements is decreased from the default value of 0x1000 (4096) slow clock cycles (33ms) to 0x0040 (64) - (0.5ms). This increases sensitivity and decreases noise.

After startup a calibration is performed, measuring the mean value and noise for each pad. Threshold values can be calculated from these, or set manually per pad. Optionally, the statistics can be kept updated during runtime.


### Mode of operation

After initialization (takes around 10 seconds), TA runs an update cycle every 10ms. For each pin, it reads the charging cycles and calculate a value:

	capDelta = measurement_time / measured_cycles - capMean

This value is the extra time that your finger causes each charging-discharging cycle to take and should be somewhat independent from the pad capacitance. From testing it seems that 4 cycles correspond to a capacitance of ~1pF. A direct touch gives a value of around 500, an indirect touch with tape inbetween around 50, touching an insulated wire around 5, and hovering a few cm above a pad give values from 1-5 depending on pad size.

If the value is above or below the threshold value for a number of update cycles, the update function will increase the digital state of the pin. Each button, depending on its type, has a maximum state (1 for analog buttons), after which it will return to state 0. E.g.:

    (0) -> touch -> (1) -> release -> (2) -> touch -> (3) -> release -> (0)

Additionally, it will calculate an analog value which can be used by analog button modes:

    analog = (capDelta - capThreshold) / (capMax - capThreshold)

Optionally, it will then update the mean and noise values for each button (if `updateMean` and `updateSigma` are set).

Finally, the button state, state transitions, or timeout may trigger button specific behavior.

Every 10th update cycle (100ms) analog values (e.g.: brightness, effect speed) are updated if necessary.

`capThreshold` can be automatically calculated from noise or set individually for each pin.
`capMean`can be disabled per pin by setting 'CapsIncludePad', in which case `capThreshold` and `capMax` must be absolute values including the capacitance of the touchpad.

#### Digital Button
  
Digital Buttons can have up to 127 actions, `Mode` defines this number. Any state above 0 can trigger an action. This can either be a preset or special values for long-touch dimming. Actions are triggered after a timeout `LongTouch` or instantly if the final state is reached.

Let's take for example a `Mode`=3 button, which is similar to the default touch buttons in WLED:

* The button will start in state (0)
* Once touched, the button will go to state (1)
* If it is kept touched for `LongTouch`, it will trigger action 1 (long press) and return to state (0) once its released
* If it is released before, it will go to state (2)
* If it is not touched again for `LongTouch`, it will trigger action 2 (short press)
* If it is touched again before, it will go to state (3), which will trigger action 3 (double press)

Or graphically:

          touched         released          touched
    (0) -----------> (1) -----------> (2) -----------> (3) -> last state, instantly trigger action 3, double press
                      |                | timeout
                      |                ` -----------> trigger action 2, short press
                      | timeout
                      ` -----------> trigger action 1, long press

All even numbers are short press, while odd numbers are long press or instant (if its the final state).

Another example, mode 6 button:
    
          t          r          t          r          t          r
    (0) -----> (1) -----> (2) -----> (3) -----> (4) -----> (5) -----> (6) -> last state, instantly trigger action 6, triple short
                |          |          |          |          |
                |          |          |          |          ` -----> trigger action 5, double short + long press
                |          |          |          ` -----> trigger action 4, double short press
                |          |          ` -----> trigger action 3, short + long press
                |          ` -----> trigger action 2, short press
                ` -----> trigger action 1, long press

Note: Special values for dimming should only be set for long press actions.
Note: If you need more than 6 states you need to recompile with a higher value in `#define TOUCHADV_NUM_ACTIONS 6`  

#### Analog value button

Has two state, released and touched. Every update cycle (10ms**), if its touched, it will use the analog value, and according to your settings, map it to e.g. a brightness range, see `Analog value button settings` below.

	value = valMin + analog * (valMax-valMin)

The calculated value is applied every analog update cycle (100ms) and additionally stored to a buffer which keeps the last 4 values. This is necessary, as once you remove your hand, it will inevitable read some values close to 0, which would render the button useless.

Once the analog update function sees no new analog values incoming, it will restore the oldest value in the buffer (300-400ms old), which hopefully is still a good value.  

#### Analog weighted button

Similar to the analog value button, but does not directly use the calculated value to set e.g. the brightness.
Instead, the analog weighted button has a fixed output value, e.g. brightness 255 or brightness 10.

The calculated value is used to give a weight to this value. Every analog update cycle, the analog update function then calculates a weighted average from all inputs i.e.:

    value = (value_btn_0 * weight_btn_0 + value_btn_1 * weight_btn_1) / (weight_btn_0 + weight_btn_1)

This is then mapped to e.g. a brightness, according to your settings, see `Analog weighted button settings` below.

Can be used for fixed value sliders, etc., see illustration:

Note that there needs to be some blending over between the pads so both buttons are pressed at once to get a mixed value.

     __________       __________
    |          |     |          |
    |  pin  0  |     |  pin  0  |
    |  val 250 |     |  val 250 |
    |\        /|     |\        /|
    |\        /|     | \      / |
    | \      / |     |  \    /  |
    | \      / |     |   \  /   |
    |  \    /  |     |    \/    |
    |  \    /  |     |  pin 1   |
    |   \  /   |     |  val 130 |
    |   \  /   |     |\        /|
    |  pin 1   |     | \      / |
    |  val 10  |     |  \    /  |
    |__________|     |   \  /   |
                     |    \/    |
                     |  pin 2   |
                     |  val 10  |
                     |__________|

#### Analog wheel button

Similar to analog weighted but the input is relative, think of scrolling a web page on your phone.

From the values and weights of all buttons, a finger position is calculated. Moving your finger rotates a virtual wheel,
which in turn modifies the analog value according to your settings, see `Analog wheel button setting` below.

The button values are given in degrees [0-360[ and warp around, i.e. the average between 240 degrees and 0 degrees is 300 degrees, and not 120. This allows for repeating pads or circular inputs.

Can be used with a 2 or 3 button slider like shown above for analog weighted buttons, or with a repeating slider as shown below (may be arranged in a circle), check the esp document above for better illustrations:

    _______________________________________________________________
    |     \      \      \      \      \      \      \      \       |
    | pin0 \ pin1 \ pin2 \ pin0 \ pin1 \ pin2 \ pin0 \ pin1 \ pin2 |
    |  0°  / 120° / 240° /  0°  / 120° / 240° /  0°  / 120° / 240° |
    |_____/______/______/______/______/______/______/______/_______|

* For sliders with repeating pins, use angles equally spaced on a circle, e.g.: 0°-120°-240° or 0°-90°-180°-270°.
* For circular inputs, a divider setting is provided to match the angle of wheel rotation to the finger movement. E.g. if you have 4 groups of pads arranged in a circle 0-1-2-0-1-2-0-1-2-0-1-2, this would give an angle of 360*4 per finger rotation, or in other words, full output after only 90° of rotation. Use a divider value of 4 to match the wheel to the finger input or higher values to increase control.
* For sliders with non-repeating pads, use any values, e.g. 0°-90°.

The wheel has some inertia and friction, can either stop at 0/360 degrees or roll over, and either fall back to min directly after max output or go min-max-min. These settings are available as build settings.

### Electrical considerations

#### Earth connection

The esp senses the capacitance between the touch pin and GND. Your body acts as a capacitance between the pin an earth.
Thus, to get a good reading you want your GND connected to earth. This is the case for many power supplies (PSU) with earth connection, including laptop PSUs (laptop usb ports have GND connected to earth if the laptop is plugged in). Power supplies with ungrounded plugs, like phone chargers lack this connection, but also some laptop-like 12V LED PSUs have a floating GND.
This can be tested with a multimeter, just measure continuity between your PSUs GND and the EARTH pin on the (disconnected) plug.
If it's not connected, you might improve sensitivity by switching to a different PSU or making an extra connection from GND to earth.

#### Capacitance estimation, TA readings

The measured value can be roughly estimated using the formula for a parallel plate capacitor, simplified:

    Finger Capacitance = (Touch Area in square mm) / (Touch Distance in mm) * 0.01pF

Example: For 15mm x 15mm finger tip hovering 2mm over the similar sized touch area -> 15\*15/2*0.01pF =~ 1.125pF.
Insulators other than air increase this value by a factor K (relative permittivity), thereby increasing sensitivity:

* Air ~1
* Wood ~2-6
* Acrylic ~3
* PCV ~4
* Glass  ~4-8
* Aluminium Oxide ~9
* Water (distilled) ~80

Example: Same finger tip resting on 2mm thick glass over the pad -> 4\*15\*15/2*0.01pF =~ 4.5pF
With the default settings, TA gives a readings which are about 4 times the capacitance in pF, i.e 4-5(Air) and 18(Glass) in
these examples.

Sensitivity can be increased by increasing the pad size, decreasing the insulator thickness and using a higher-K insulator.

### General Notes

Capcitance values for each pin can be seen in 'info' in the wled webinterface.

For testing the analog functions, it might be helpful to disable fading.

There is currently no check for wrong input parameters:

* For digital buttons, special presets should only be used for long presses (even numbered long touch fields). Setting special presets to short presses will result in strange behavior.
* Wheel Buttons belonging together should be set to the same min/max output values. If they do not have the same values, currently the highest-id button wins, but this could change.
* Only one analog control can be used at the same time, even if they control different things. There is currently no lock-out, trying to change e.g. effect speed and brightness at the same might lead to strange results.
* Setting thresholdCap=maxCap results in a division by 0



### Settings

#### Global:

* `Enabled` - GPIOs will not be initialized if the mod is disabled
* `ThresholdSigma` - The number of standard deviations the threshold will be set to for buttons with thresholdCap=0, higher value decreases sensitivity.
* `Debounce` - button state will change after this number of measurements above/below the threshold, higher value decreases responsivity.
* `LongTouch` - relevant for digital buttons only, timeout after which a button press is registered. I.e. the time before a long touch is registered as such, or before a short touch is registered when waiting for a potential double touch.
* `UpdateMean` - A mean value is measured for each pin during boot, this can be updated continuously. May cause drift if you keep your hand close to a button without touching it.
* `UpdateSigma` - Same as UpdateMean, but for the variance/standard deviation.

#### Per Pin:
  
Note: Change `#define TOUCHADV_NUM_PINS 4` to get more pins
Note: Click the `info` button in WLED to get current readings for each pin
  

* `GPIO` - The GPIO used, use '-1' to disable
* `ThresholdCap` - Digital buttons will trigger at this cap, analog buttons will register 0% touch. 
If set to 0, Threshold will be calculated from measured variance * `ThresholdSigma`, this is the most sensitive mode, but might lead to issues if the calibration values are off, which is why it's not the default.
* `FullCap` - For analog buttons, at this value, the button will register 100% touch. 
If set to 0, will be set to 30 (it's not relevant for digital buttons). Needs to be higher than `ThresholdCap`.
* `CapsIncludePad` - Do not use calibration mean value, `ThresholdCap` and `FullCap` include the pad capacitance.
* `Mode` - Pin mode: 1-128 digital, 131 analog value, 141 analog weighted, 151 analog wheel, 171 digital switch
For digital pins, this gives the maximum number of states:
-- '1' - single touch
-- '2' - long / short touch
-- '3' - long / short / double touch
-- '4' - long / short / short+long / short+short touch
-- '5' - long / short / short+long / short+short / triple touch
-- '6' - long / short / short+long / short+short / short+short+long / short+short+short touch
* `Actions` - For digital pins, these are the presets (with special presets for brightness etc.),
analog pins have special settings here, see below

#### Digital button setting:
  
Note: Change `#define TOUCHADV_NUM_ACTIONS 6` to get more actions

Note: Special presets should only be set for even numbered actions (0,2,4, etc.)

* `Actions[0]` - preset for long touch (or any touch for mode 1)
* `Actions[1]` - preset for short touch
* `Actions[2]` - preset for short+long touch (or any double touch for mode 3)
* `Actions[3]` - preset for short+short touch
* `Actions[4]` - preset for short+short+long touch (or any triple touch for mode 5)
* `Actions[5]` - preset for short+short+short touch
and so on..

#### Analog value button settings:

* `Actions[0]` - preset (special presets only)
* `Actions[1]` - output value at `ThresholdCap`
* `Actions[2]` - output value at `FullCap`

#### Analog weighted button settings:

* `Actions[0]` - preset (special presets only)
* `Actions[1]` - output value

#### Analog wheel button settings:

* `Actions[0]` - preset (special presets only)
* `Actions[1]` - wheel min output value
* `Actions[2]` - wheel max output value
* `Actions[3]` - speed divider for layouts with repeating buttons, inactive for 0
* `Actions[4]` - position on the wheel in degrees/10

For layouts with repeating buttons use equally spaced angles, i.e.: 0°-120°-240° (0,12,24) or 0°-90°-180°-270° (0,9,18,27)

* Build settings:
-- `#define TOUCHADV_WHEEL_HAS_ENDSTOP` 1:wheel stops at 0/360 0:wheel continues to turn
-- `#define TOUCHADV_WHEEL_FRICTION` speed decrease each update, default 0.02 (-2% every 10ms)
-- `#define TOUCHADV_WHEEL_VALUEUPDOWN` 0: min val at 0 degrees, max at 360 1:min val at 0/360, max at 180

#### Digital switch settings:

Note: Either use `CapsIncludePad` for this mode, or make sure all switches are in low state during calibration.

* `Actions[0]` - preset for cap >`ThresholdCap`
* `Actions[1]` - preset for cap < `ThresholdCap`
* `Actions[2]` - preset for switch tapped to cap > `ThresholdCap`
* `Actions[3]` - preset for switch tapped to cap < `ThresholdCap`
* `Actions[4]` - preset for switch double tapped to cap > `ThresholdCap`
* `Actions[5]` - preset for switch double tapped to cap < `ThresholdCap`

The reason this mode exists is because I have endstop/lever switches for each door in my bathroom cabinet, and I wanted to use them to control WLED presets by toggling them. For this reason, this usermod also works with non-touch GPIOs. It will report fake capacitances of 100 (high) and 150 (low) in this case. To use this function, set `CapsIncludePad`: Enabled, `ThresholdCap` : 125, and `FullCap` : 150.

#### Special presets
* 250 - brightness
* 249 - effect speed
* 248 - effect intensity

### Debug output

Add `#define WLED_DEBUG` in your WLED/wled00/my_config.h to enable serial debugging

Additional defines:

`#define TOUCHADV_DEBUG_VALUES` gives regular updates on measured mean value, variance, threshold, and pin state
`#define TOUCHADV_DEBUG_TRANSITION` gives updates on button transitions
`#define TOUCHADV_DEBUG_ANALOG_VALUE` gives updates if analog pins are touched
`#define TOUCHADV_DEBUG_WHEEL` gives updates when the wheel is active