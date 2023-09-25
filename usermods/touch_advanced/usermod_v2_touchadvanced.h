#pragma once

#include "wled.h"

// extra settings
#ifndef TOUCHADV_NUM_ACTIONS
  #define TOUCHADV_NUM_ACTIONS 6
#endif

#ifndef TOUCHADV_NUM_PINS
  #define TOUCHADV_NUM_PINS    4
#endif

#define TOUCHADV_WHEEL_HAS_ENDSTOP 1 // 1 or 0
#define TOUCHADV_WHEEL_FRICTION    0.02f
#define TOUCHADV_WHEEL_VALUEUPDOWN 0 // 1 or 0

//#define TOUCHADV_CALIBRATE_PWM # not implemented

// debug switches
//#define TOUCHADV_DEBUG_VALUES
//#define TOUCHADV_DEBUG_TRANSITION
//#define TOUCHADV_DEBUG_ANALOG_VALUE
//#define TOUCHADV_DEBUG_WHEEL

// modes and special presets
#define TOUCHADV_MODE_DIGITAL_MAX     127
#define TOUCHADV_MODE_ANALOG_VALUE    131
#define TOUCHADV_MODE_ANALOG_WEIGHTED 141
#define TOUCHADV_MODE_ANALOG_WHEEL    151
#define TOUCHADV_MODE_DIGITAL_META    161 // not working yet: programmed, never tested
#define TOUCHADV_MODE_DIGITAL_SWITCH  171 // somewhat working, but switches needs to be in low state during start
                                          // or use absolute threshold value

#define TOUCHADV_ACTION_META          255 // not working yet: programmed, never tested
#define TOUCHADV_ACTION_BRIGHTNESS    250
#define TOUCHADV_ACTION_SPEED         249
#define TOUCHADV_ACTION_INTENSITY     248
#define TOUCHADV_ACTION_PALETTE       247 // not working yet: should be limited to available palettes

class TouchAdvancedUsermod : public Usermod {
  private:

    // hardcoded settings
    static const uint8_t pins = TOUCHADV_NUM_PINS;
    static const uint8_t actions = TOUCHADV_NUM_ACTIONS;
    static const uint8_t analogBufferLen = 4;     // to restore 300-400ms old analog value after lift
    static const uint8_t bufferLen = 6;           // for the median filter
    const uint8_t millisUpdate = 10;              // update time in ms
    const uint16_t calibrationSamples = 100;      // 10s total calibration time
    const bool enableAdaptiveLongTouch = false;   // does not seem to improve things much
    
    const uint16_t hw_measure_cycles = 0xffff;    // ~8.5MHz fast RC clock -> ~7.7ms
    const uint16_t hw_sleep_cycles =   0x0040;    // ~150kHz slow RC clock -> ~0.5ms

    // settings
    bool      enabled               = true;
    uint8_t   guard                 = 5;          // minCap in standard deviations
    uint8_t   debounceLen           = 5;          // in update cycles
    uint16_t  millisLongTouch       = 400;        // touch timeout in ms
    bool      updateMean            = true;
    bool      updateVariance        = true;

    // settings per pin
    int8_t    pin[pins]             = {-1};       // should maybe be named gpio maybe
    uint8_t   pinModex[pins]        = {0};        // renamed to be able to use arduino pinMode()
    uint16_t  pinCapMin[pins]       = {5};        // trigger distance to mean, analog 0%
    uint16_t  pinCapMax[pins]       = {50};       // analog 100%, unit:clock cycles per charging cycle
    bool      pinNoMean[pins]       = {false};    // cap values are absolute
    uint8_t   action[pins][actions] = {{0}};
    
    // calibration
    uint8_t   calibrationStep     = 0;
    uint16_t  calibrationSample   = 0;
    float     calK[pins]          = {0};                // first calibration sample, subtracted to 
    float     capMean[pins]       = {0};                // avoid numerical issues in summation for
    float     capMeanSq[pins]     = {0};                // mean and variance calculations
    float     capVar[pins]        = {0};
    uint16_t  pinLongTouch[pins]  = {millisLongTouch};  // per pin long touch, for adaptive long touch

    // state variables: general
    unsigned long millisLast      = 0;
    uint16_t  updateCounter       = 0;
    bool      initDone            = false;
 
    // state variables per pin
    float     capLast[pins]       = {0};        // last cap, for mean reset, web interface
    uint8_t   bufferPos[pins]     = {0};
    float     buffer[pins][bufferLen] = {{0}};  // value buffer for fir filter
    uint16_t  millisTouch[pins]   = {0};
    uint8_t   state[pins]         = {0};        // even-untouched, odd-touched
    int8_t    debounce[pins]      = {0};

    // state variables: analog
    uint8_t   analogMeta              = 0;
    uint8_t   analogAction            = 0;      // 0 -> analog update stopped, else what to change
    uint32_t  analogValueTimesWeight  = 0;      // averaging numerator
    uint16_t  analogTotalWeight       = 0;      // averaging denominator
    bool      analogBufferUse         = false;  // some functions need to use the analog buffer
    uint8_t   analogBufferPos         = 0;      // next position to write
    uint8_t   analogBufferFill        = 0;      // valid values in buffer
    uint8_t   analogBuffer[analogBufferLen] = {0};

    // state variable: wheel
    uint8_t   wheelAction     = 0;       // 0 -> wheel stopped, else current analogAction
    uint8_t   wheelAnalogMin  = 0;       // min output value
    uint8_t   wheelAnalogMax  = 0;       // max output value
    uint8_t   wheelDivider    = 0;       // divides speed
    float     wheelPosition   = 0;       // angle in degrees
    float     wheelSpeed      = 0;       // angular speed in degrees/update
    float     fingerReal      = 0;       // cartesian sum variable for finger position
    float     fingerImag      = 0;       // cartesian sum variable for finger position
    bool      fingerOnWheel   = false;   // finger position is valid
    float     fingerPosition  = 0;       // finger position on wheel in degrees
    float     fingerSpeed     = 0;       // finger speed in degrees/update

    // strings to reduce flash memory usage (used more than twice)
    static const char _name[];
    static const char _enabled[];
    static const char _nsigma[];
    static const char _debounce[];
    static const char _lptime[];
    static const char _umean[];
    static const char _uvariance[];
    static const char _mode[];
    static const char _gpio[];
    static const char _actions[];
    static const char _nomean[];
    static const char _capmin[];
    static const char _capmax[];
  

    void touchInit() {
      touch_pad_init();
      // around 4 extra clocks per cycle and pF with these settings
      // (dis)charging current:     I ~= 8.5uA @ slope7   (default, value measured)
      // fast RC oscillator         f ~= 8.5MHz           (nominal, can be 8-9MHz)
      // measurement clocks:        cm = 65535
      // measurement time:          tm = cm / f = 7.71ms  (~7.3ms measured)
      // charge cycle time:         tc = tm / touchRead()
      // charge per cycle:          dQ = I * tc
      // voltage change per cycle:  dV = 4.4V (0.5V->2.7V->0.5V)
      //
      // measured cap in SI units As/V (F): 
      //   C_SI = dQ/dV = I * tc / dV = 8.5uA * 65535 / 8.5Mhz / touchRead() / 4.4V 
      //        = 14894pF / touchRead()
      // measured cap in clocks: (TA uses this)
      //   C_TA = dQ/dV 
      //        = 65535 / touchRead()
      //
      // conversion factor between SI and clocks:
      //   C_SI/C_TA = 8.5uA / 8.5Mhz / 4.4V = 227fF (per clock)
      touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_0V);
      touch_pad_set_meas_time(hw_sleep_cycles, hw_measure_cycles);
      touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
      touch_pad_filter_start(millisUpdate/3);
    }

    void touchDeInit() {
      touch_pad_filter_stop();
      touch_pad_filter_delete();
      touch_pad_deinit();
    }

    void touchPinInit(int8_t* pin) {
      if (*pin == -1) return;

      if (!pinManager.allocatePin(*pin, false, PinOwner::UM_Unspecified)) {
        *pin = -1;
        DEBUG_PRINTLN(F(" - allocation failed - manager"));
      } else {
        int8_t touchChannel = digitalPinToTouchChannel(*pin);
        if (touchChannel != -1)
          touch_pad_config((touch_pad_t)digitalPinToTouchChannel(*pin), 0x1000);
          // slope, default seems to be max anyway
          //touch_pad_set_cnt_mode((touch_pad_t)digitalPinToTouchChannel(*pin), TOUCH_PAD_SLOPE_7, TOUCH_PAD_TIE_OPT_LOW);
        else
          // does not really seem to be a touch input :P
          pinMode(*pin, INPUT_PULLUP);
      }
    }

    void touchPinDeInit(int8_t pin) {
      if (pin == -1) return;
      pinManager.deallocatePin(pin, PinOwner::UM_Unspecified);
    }

    float touchPinMeasure(uint8_t p, bool filtered) {
      int8_t touchChannel = digitalPinToTouchChannel(pin[p]);

      if (touchChannel == -1) {
        // no real touch channel, report fake values 100/150
        capLast[p] = 150-50*digitalRead(pin[p]);
      } else {
        uint16_t raw;
        float corr, val;
        touch_pad_read_raw_data((touch_pad_t)touchChannel, &raw);
        corr = raw;

        #if 0
        //testing pwm correction
        if (p == 6 && pin[p] == 32) {// right
          corr += 7.0f * (R(busses.getPixelColor(70)))/255.0f;
          //Serial.println(busses.getPixelColor(70));
        }

        if (p == 9 && pin[p] == 13) {// left
          corr += 8.0f * (R(busses.getPixelColor(71)))/255.0f;
          //Serial.println(busses.getPixelColor(71));
        }
        #endif

        // median filter, length 7 hardcoded atm
        // filters up to three bad values, causes 30ms input delay,
        // affects input behaviour, minimum 40ms touch/release required
        if (filtered) {
          float sort[bufferLen+1], swap;
          memcpy(sort, buffer[p], sizeof(buffer[p]));
          sort[bufferLen] = corr;

          for (uint8_t i=1; i<(bufferLen+1); i++) {
            for (uint8_t j=i; j>0 && sort[j-1] > sort[j]; j--) {
              swap = sort[i]; sort[i] = sort[j]; sort[j] = swap;
            }
          }
          
          //val = sort[bufferLen/2];
          
          uint8_t cnt=0;
          val = 0;
          for (uint8_t i=0; i<(bufferLen+1); i++) {
            // med +- 2
            if ( fabsf(sort[i] - sort[bufferLen/2]) <= 3.0f ) {
              val += sort[i];
              cnt++;
            }
          }
          val /= cnt;
        } else {
          val = corr;
        }

        // update buffer
        buffer[p][bufferPos[p]] = corr;
        bufferPos[p]++;
        bufferPos[p] %= bufferLen;

        // first cycle takes longer (0.0-2.7-0.5 vs 0.5-2.7-0.5 -> 1.114)
        // partial last cycle not counted (-> 0.5)
        capLast[p] = hw_measure_cycles / (val+0.114f+0.5f);
      }
      return capLast[p];
    }

    void calibration() {

      #ifdef TOUCHADV_CALIBRATE_PWM
      //busses 
      //  getNumBusses
      //each bus
      //  getType == TYPE_ANALOG_1CH, etc TYPE_ONOFF
      //
      for (uint8_t i; i<busses.getNumBusses(); i++)
      {
        switch busses.getBus(i)->getType() {
          case TYPE_ANALOG_1CH:

        }
        
      }
      #endif

      if (calibrationSample == calibrationSamples) {
        calibrationStep++;
        calibrationSample = 0;
      }

      for (int p=0; p<pins; p++) {
        if (pin[p] == -1) continue;

        // step 0: do nothing (noisy values)
        if (calibrationStep == 0) {
            capMean[p] = 0;
            capMeanSq[p] = 0;
        }

        // step 1: collect data
        if (calibrationStep == 1) {
          float cap = touchPinMeasure(p, false);
          if (!calibrationSample) 
            calK[p] = cap;
          capMean[p]   += (cap-calK[p]); // avoid floating point issues
          capMeanSq[p] += (cap-calK[p])*(cap-calK[p]);
          DEBUG_PRINT(F("TA cal1 pin: ")); DEBUG_PRINT(p);DEBUG_PRINT(F(" sample: "));DEBUG_PRINT(calibrationSample);
          DEBUG_PRINT(F(" cap: ")); DEBUG_PRINTLN(cap);
        }

        // step2: calculate moments (runs only once)
        if (calibrationStep == 2) {
          capVar[p] = (calibrationSamples*capMeanSq[p]-(capMean[p]*capMean[p]))/(calibrationSamples*calibrationSamples-1);
          //capVar[p] = max(capVar[p], 0.04f);
          capMean[p] = calK[p] + capMean[p] / (calibrationSamples-1);
          DEBUG_PRINT(F("TA cal2 pin: ")); DEBUG_PRINT(p);DEBUG_PRINT(F(" mean: ")); DEBUG_PRINT(capMean[p]);
          DEBUG_PRINT(F(" std: ")); DEBUG_PRINT(sqrtf(capVar[p]));DEBUG_PRINTLN(F(" -> TouchAdv calibrated"));
        }
      }
      calibrationSample++;
    }

    void updateDigital(uint8_t p, bool transition, bool timeout) {
      // digital button mode < 128
      // increases state every touch/release, triggers on timeout or final state
      // sweeps analog value in special long press actions > 200
      // action[i-1] = action to perform in state i

      if (!state[p])
        return;

      // timeout or final state triggers action: apply preset or move on to dim state
      if (state[p] < 128 && (timeout || (transition && (state[p] == pinModex[p]) ))) {
        if (action[p][state[p]-1] < 200) {
          if (action[p][state[p]-1] != 0 )
            applyPreset(action[p][state[p]-1], CALL_MODE_BUTTON);
          if (state[p] % 2)
            state[p] = 255;     // still touched, waiting state
          else
            state[p] = 0;       // lifted, return to 0
        } else {
          state[p] += 128;    // special dim state
        }
      }

      // special dim state
      if (state[p] > 128 && state[p] != 255) {
        int16_t value = 127 + 150 * sinf(millisTouch[p]/2000.0f);
        value = max((int16_t)1  , value);
        value = min((int16_t)255, value);

        analogAction = action[p][state[p]-1-128];
        analogValueTimesWeight = value * 255;
        analogTotalWeight = 255;
        analogBufferUse = 0;
      }

      if (state[p] == 255 && millisTouch[p] > 3000) // pad cap changed
        capMean[p] = capLast[p];                        // up mean
    }

    void updateDigitalSwitch(uint8_t p, bool transition, bool longTouch) {
      // digital switch mode 171
      // increases state every touch/release, triggers on timeout, return to state 0 or 1
      // action[i-1] = action to perform in state i

      // state 0 stable low
      // state 1 stable high

      // action state 1 high
      // action state 2 low
      // action state 3 tapped to high
      // action state 4 tapped to low
      // action state 5 double tapped to high
      // action state 6 double tapped to low 

      // state 2 is ambiguous, go to 4 if tap transition (0->1->2) not (1->2)
      if (transition && state[p] == 2 && !longTouch)
        state[p] = 4;

      // apply potential tap action on timeout
      if (!transition) {
        if (action[p][state[p]-1] != 0 )
          applyPreset(action[p][state[p]-1], CALL_MODE_BUTTON);
        state[p] %= 2;
      }
    }

    void updateAnalogValue(uint8_t p, float measurement) {
      // analog button mode 131
      // when touched, button will output a value depending on touch input

      analogAction   = action[p][0];  // what to change
      uint8_t minVal = action[p][1];  // min output value at threshold
      uint8_t maxVal = action[p][2];  // max output value at max cap.

      uint8_t value = 0;              // output value, calculated

      measurement = min(1.0f, measurement);
      measurement = max(0.0f, measurement);
      value = minVal + (maxVal-minVal) * measurement;

      analogValueTimesWeight += (uint32_t)value*255;
      analogTotalWeight += 255;
      analogBufferUse = 1;

      #ifdef TOUCHADV_DEBUG_ANALOG_VALUE
        DEBUG_PRINT(F("AnalogValue raw: "));DEBUG_PRINT(measurement);
        DEBUG_PRINT(F(" action: "));DEBUG_PRINT(analogAction);
        DEBUG_PRINT(F(" val: "));DEBUG_PRINTLN(value);
      #endif
    }

    void updateAnalogWeighted(uint8_t p, float measurement) {
      // weighted analog button mode 141
      // button will always ouput a set value, but will be assigned a weight depending on the touch value
      // if more than one weighted button is touched, their values will be mixed with their weight values

      analogAction    = action[p][0];  // what to change
      uint8_t value   = action[p][1];  // output value
      
      uint8_t weight = 0;             // output weight, calculated

      measurement = min(1.0f, measurement);
      measurement = max(0.0f, measurement);
      weight = 255 * measurement;

      analogValueTimesWeight += (uint32_t)value * weight;
      analogTotalWeight += weight;
      analogBufferUse = 1;

      #ifdef TOUCHADV_DEBUG_ANALOG_VALUE
      DEBUG_PRINT(F("AnalogWeighted raw: "));DEBUG_PRINT(measurement);
      DEBUG_PRINT(F(" action: "));DEBUG_PRINT(analogAction);
      DEBUG_PRINT(F(" value: "));DEBUG_PRINTLN(value);
      DEBUG_PRINT(F(" weight: "));DEBUG_PRINTLN(weight);
      #endif
    }

    void updateAnalogWheel(uint8_t p, float measurement) {
      // wheel mode 151
      wheelAction      = action[p][0];  // what to change
      wheelAnalogMin   = action[p][1];  // wheel min output value
      wheelAnalogMax   = action[p][2];  // wheel max output value
      wheelDivider     = action[p][3];  // speed divider
      uint8_t position = action[p][4];  // pad position in degrees / 10
      //uint8_t mass     = action[p][5];  // mass

      uint8_t weight = 0;               // output weight, calculated

      measurement = min(1.0f, measurement);
      measurement = max(0.0f, measurement);
      weight = 255 * measurement;

      fingerReal += cosf((float)position/18*PI) * weight;
      fingerImag += sinf((float)position/18*PI) * weight;

      #ifdef TOUCHADV_DEBUG_ANALOG_VALUE
      DEBUG_PRINT(F("AnalogWheel raw: "));DEBUG_PRINT(measurement);
      DEBUG_PRINT(F(" action: "));DEBUG_PRINT(wheelAction);
      DEBUG_PRINT(F(" position: "));DEBUG_PRINT(position);
      DEBUG_PRINT(F(" weight: "));DEBUG_PRINTLN(weight);
      #endif
    }

    void updateDigitalMeta(uint8_t p) {
      // incremental button mode 161
      // action[i] = presets to cycle

      // we are at the end of the list, go to first
      if (!action[p][state[p]/2])
        state[p] = 1;

      analogMeta = action[p][state[p]/2];

      #ifdef TOUCHADV_DEBUG_ANALOG_VALUE
      DEBUG_PRINT(F("DigitalMeta meta: "));DEBUG_PRINTLN(analogMeta);
      #endif
    }

    void spinWheel() {
      // for wheel mode
      // update angular speed and position, update analog value
      float fingerPositionOld = fingerPosition;
      bool fingerOnWheelOld = fingerOnWheel;
      float fingerSpeedOld = fingerSpeed;

      // init
      if (wheelAction && !analogAction) {
        uint8_t current = 0;
        uint8_t action = wheelAction;
        if (action == TOUCHADV_ACTION_META)        action = analogMeta;
        if (action == TOUCHADV_ACTION_BRIGHTNESS)  current = bri;
        if (action == TOUCHADV_ACTION_SPEED)       current = effectSpeed;
        if (action == TOUCHADV_ACTION_INTENSITY)   current = effectIntensity;
        if (action == TOUCHADV_ACTION_PALETTE)     current = effectPalette; //should be limited somehow strip.getPaletteCount(); 
        if (TOUCHADV_WHEEL_VALUEUPDOWN)
          wheelPosition = 180 * (current+0.5f) / 256;
        else 
          wheelPosition = 360 * (current+0.5f) / 256;
      }

      // update finger parameters
      float fingerAmplitude = fingerReal*fingerReal + fingerImag*fingerImag;
      if (fingerAmplitude > 1) {
        fingerOnWheel = true;
        fingerPosition = remainderf(atan2f(fingerImag,fingerReal) * 180 / PI, 360);
        if (fingerOnWheelOld) {
          fingerSpeed = remainderf(fingerPosition - fingerPositionOld, 360);
          if (wheelDivider)
            fingerSpeed /= wheelDivider;
          // might be choppy, moving average
          fingerSpeed = fingerSpeedOld * 0.9 + fingerSpeed*0.1;
        } else
          fingerSpeed = 0;
        fingerReal *= 0.5; // average a bit to keep finger on wheel if
        fingerImag *= 0.5; // there is a gap between the pads or something
      } else {
        fingerOnWheel = false;
      }

      // get wheel variables
      //   domega/dt = alpha
      wheelSpeed -= TOUCHADV_WHEEL_FRICTION * wheelSpeed; // -axis friction * wheel speed
      if (fingerOnWheelOld)
        wheelSpeed += 0.5 * (fingerSpeed-wheelSpeed);     // finger friction * relative speed (inertia = 1)
      
      //   dphi/dt = omega
      wheelPosition += wheelSpeed;

      // brake/bounce back on 0/360?
      if (wheelPosition >= 360 || wheelPosition < 0)
        wheelSpeed *= -0.0;

      // endstop on 0/360?
      if (TOUCHADV_WHEEL_HAS_ENDSTOP) {    // with stop
        wheelPosition = max(min(wheelPosition, 359.99f), 0.0f);
      } else {  // without stop
        wheelPosition = fmodf(wheelPosition + 360, 360);
      }

      // no finger, no speed -> disable wheel, set analogAction 0 to stop analog write from restoring buffer
      if (wheelAction && !fingerOnWheel && fabsf(wheelSpeed) < millisUpdate/400.0f) { //one step every 2000ms
        wheelAction = 0;
        wheelSpeed = 0;
      }

      // set analog stuff
      if (wheelAction) {
        analogAction = wheelAction;
        if (TOUCHADV_WHEEL_VALUEUPDOWN) {
          // triangle 0 -> 128 -> 254 -> 128 -> 0
          analogValueTimesWeight = 0 + (min(wheelPosition / 180 * 256, (360-wheelPosition) / 180 * 256));
        } else {
          // sawtooth 0 -> 128 -> 255 -> 0
          analogValueTimesWeight = 0 + (wheelPosition / 360 * 256);
        }
        analogTotalWeight = 1;
        analogBufferUse = 0;
      }

      // debug stuff
      #ifdef TOUCHADV_DEBUG_WHEEL
        if (wheelAction) {
          DEBUG_PRINT(F("Wheel position: "));DEBUG_PRINT(wheelPosition);DEBUG_PRINT(F(" speed: "));DEBUG_PRINT(wheelSpeed);
          DEBUG_PRINT(F(" action: "));DEBUG_PRINT(wheelAction);DEBUG_PRINT(F(" value: "));DEBUG_PRINT(analogValueTimesWeight);
          if (fingerOnWheelOld) {
            DEBUG_PRINT(F(" Finger position: "));DEBUG_PRINT(fingerPosition);DEBUG_PRINT(F(" amp: "));DEBUG_PRINT(fingerAmplitude);DEBUG_PRINT(F(" change: "));DEBUG_PRINT(fingerSpeed);
          }
          DEBUG_PRINTLN(F(""));
        }
      #endif
    }
    
    void writeAnalogValues() {
      // updates wled brightness, speed, etc. values, gets updated slower than than the normal update function
      
      uint8_t value = 0;

      if (!analogTotalWeight) {
        if (analogBufferFill && analogBufferUse) {
          // restore value a few 100ms ago, in case finger was lifted
          value = analogBuffer[analogBufferPos % analogBufferFill];
          DEBUG_PRINT(F("Writing analog "));DEBUG_PRINT(analogAction);DEBUG_PRINT(F(" buffer val: "));DEBUG_PRINTLN(value);
        }
        else
          // set function inactive
          analogAction = 0;
        analogBufferPos = 0;
        analogBufferFill = 0;
      }
      else 
      {
        value = analogValueTimesWeight / analogTotalWeight;
        // store old values, needed for the analog value button
        analogBuffer[analogBufferPos] = value;
        analogBufferPos++;
        analogBufferPos %= analogBufferLen;
        analogBufferFill = max(analogBufferPos, analogBufferFill);
      }

      // update if different
      uint8_t* current = 0;
      uint8_t action = analogAction;
      if (action == TOUCHADV_ACTION_META)       action = analogMeta;
      if (action > 200 && action <= 216)        current = &(strip.getSegment(action-200).opacity);
      if (action == TOUCHADV_ACTION_BRIGHTNESS) current = &bri;
      if (action == TOUCHADV_ACTION_SPEED)      current = &effectSpeed;
      if (action == TOUCHADV_ACTION_INTENSITY)  current = &effectIntensity;
      if (action == TOUCHADV_ACTION_PALETTE)    current = &effectPalette; //should be limited somehow strip.getPaletteCount(); 

      if (current && (*current != value)) {
        *current = value;
        colorUpdated(CALL_MODE_NO_NOTIFY);
        DEBUG_PRINT(F("Writing analog "));DEBUG_PRINT(analogAction);DEBUG_PRINT(F(" value: "));DEBUG_PRINTLN(value);
      } else {
        DEBUG_PRINT(F("Not writing analog "));DEBUG_PRINT(analogAction);DEBUG_PRINT(F(" value: "));DEBUG_PRINTLN(value);
      }

      // old Buffer value written, set function inactive
      if (!analogTotalWeight)
        analogAction = 0;

      // reset for next analog cycle
      analogValueTimesWeight = 0;
      analogTotalWeight = 0;
    }

    void update(unsigned long millisSinceLast) {
      // main update function, runs once every 10ms after init
      // reads analog values, button timing, state transitions, calls button functions

      updateCounter++;
      updateCounter%=6000;

      for (int p=0; p<pins; p++) {

        if (pin[p] == -1 || !pinModex[p]) continue;

        // read
        float cap      = touchPinMeasure(p, true);   // total capacitance in hw_cycles (equal to about 0.25pF)
        float capDelta = pinNoMean[p] ? cap          : cap-capMean[p];  // finger capacitance and noise
        float capMin   = pinCapMin[p] ? pinCapMin[p] : guard * sqrtf(capVar[p]);
        float capMax   = pinCapMax[p] ? pinCapMax[p] : 30;
        float analog   = (capDelta - capMin) / (capMax-capMin);

        // time, keep from overflowing
        bool longTouchLast = millisTouch[p] > millisLongTouch || (enableAdaptiveLongTouch && (millisTouch[p] > pinLongTouch[p]));
        if (millisTouch[p] < 60000)
          millisTouch[p] += millisSinceLast;
        bool longTouch = millisTouch[p] > millisLongTouch || (enableAdaptiveLongTouch && (millisTouch[p] > pinLongTouch[p]));
        bool timeOut = longTouch && !longTouchLast;

        // "debouncing" threshold filter
        // causes debounceLen*10ms input delay and changes input behaviour
        // touches/releases below debounceLen will not be registered
        // high values may cause double touch to be registered as single touch, etc.
        bool touchedLast = state[p] & 0x01;    // last cycle
        if (touchedLast == (capDelta > capMin)) // logical state == button state?
          debounce[p] = 0;
        else
          debounce[p]++;
        bool transition = debounce[p] > debounceLen;
        bool transition10 = transition && touchedLast;
        bool transition01 = transition && !touchedLast;
        bool touchedNow = !transition != !touchedLast; //xor
        
        // update mean, deviation, after 3s in state, for switch buttons in state 0 only
        if (debounce[p] == 0 && millisTouch[p] >= 3000 && (pinModex[p]!=TOUCHADV_MODE_DIGITAL_SWITCH || state[p] == 0)) {
            float k = state[p] == 0 ? 0.01f : 0.0001f;
          if (updateMean)//) && fabsf(capDelta) < 3*sqrtf(capVar[p]))
            capMean[p] = (capMean[p] * (1-k) + cap * k);
          if (updateVariance && fabsf(capDelta) < 5*sqrtf(capVar[p]) && fabsf(cap-capMean[p]) < 5*sqrtf(capVar[p])) {
            capVar[p] = capVar[p] * 0.999f + capDelta*(cap-capMean[p]) * 0.001f;
            capVar[p] = max(capVar[p], 0.01f);
          }
        }
        
        // debug stuff
        #ifdef TOUCHADV_DEBUG_VALUES
        if (updateCounter % 100 == 0) {
          DEBUG_PRINT(F("TA pin: "));DEBUG_PRINT(p);DEBUG_PRINT(F(" cap: "));DEBUG_PRINT(cap);
          DEBUG_PRINT(F(" mean: "));DEBUG_PRINT(capMean[p]);DEBUG_PRINT(F(" delta: "));DEBUG_PRINT(capDelta);
          DEBUG_PRINT(F(" std: "));DEBUG_PRINT(sqrtf(capVar[p]));DEBUG_PRINT(F(" state: "));DEBUG_PRINT(state[p]);
          DEBUG_PRINT(F(" analog: "));DEBUG_PRINT(analog);DEBUG_PRINT(F(" time: "));DEBUG_PRINTLN(millisTouch[p]);       
        } 
        #endif

        if (transition)
        { 
          state[p]++;

          // find maximum state
          uint8_t maxstate=1;           // default for analog buttons
          if (pinModex[p] <= 128)                         
            maxstate = pinModex[p];      // digital buttons
          if (pinModex[p] == TOUCHADV_MODE_DIGITAL_META)
            maxstate = actions*2 - 1;   // incremental digital
          if (pinModex[p] == TOUCHADV_MODE_DIGITAL_SWITCH)
            maxstate = actions;         // correct?
          
          if (transition10 && state[p] > maxstate)
            state[p] = 0;
          if (transition01 && state[p] > maxstate)
            state[p] = 255;             // waiting state

          // adaptive timeout
          if (enableAdaptiveLongTouch && state[p] == 1)
            pinLongTouch[p] = longTouch;
          if (enableAdaptiveLongTouch && state[p] == 2)
            pinLongTouch[p] = 4 * millisTouch[p];
          
          millisTouch[p] = 0;
          
          // debug stuff
          #ifdef TOUCHADV_DEBUG_TRANSITION
          DEBUG_PRINTLN(millis()); DEBUG_PRINT(F(" pin: ")); DEBUG_PRINT(p); DEBUG_PRINT(F(" state: "));
          DEBUG_PRINT(state[p]); DEBUG_PRINT(F(" mean: ")); DEBUG_PRINT(capMean[p]); DEBUG_PRINT(F(" var: ")); DEBUG_PRINTLN(capVar[p]);
          #endif
        }

        // digital button                     called on (timeout,) transition, long touch
        if (pinModex[p] <= TOUCHADV_MODE_DIGITAL_MAX     && (transition || longTouch))
          updateDigital(p, transition, timeOut);

        // digital switch                     called on timeout or transition
        if (pinModex[p] == TOUCHADV_MODE_DIGITAL_SWITCH  && (timeOut || transition))
          updateDigitalSwitch(p, transition, longTouch);

        // incremental meta button            called when touched
        if (pinModex[p] == TOUCHADV_MODE_DIGITAL_META    && transition01) 
          updateDigitalMeta(p);

        // analog value button                called while touched
        if (pinModex[p] == TOUCHADV_MODE_ANALOG_VALUE    && touchedNow)
          updateAnalogValue(p, analog);

        // analog weighted button             called while touched
        if (pinModex[p] == TOUCHADV_MODE_ANALOG_WEIGHTED && touchedNow)
          updateAnalogWeighted(p, analog);
        
        // analog wheel button                called while touched
        if (pinModex[p] == TOUCHADV_MODE_ANALOG_WHEEL    && touchedNow)
          updateAnalogWheel(p, analog);
                  
      }
      // analog wheel
      spinWheel();

      // analog output
      if (analogAction && updateCounter % 10 == 0)
        writeAnalogValues();
    }

  public:
    void setup() {
      DEBUG_PRINT(FPSTR(_name));        
      DEBUG_PRINTLN(F(" - setup()"));
      if (!enabled)
        return;
      
      touchInit();

      for (uint8_t p=0; p<pins; p++)
        touchPinInit(&pin[p]);

      DEBUG_PRINT(FPSTR(_name));
      DEBUG_PRINTLN(F(" - initialized"));
      initDone = true;
    }

    void loop() {
      if (!enabled || !initDone)
        return;
        
      unsigned long millisNow = millis();

      if ((millisNow - millisLast) >= millisUpdate) {
        if (calibrationStep < 2)
          calibration();
        else
          update(millisNow - millisLast);
        millisLast = millisNow;
      }
    }

    /*
     * addToJsonInfo() can be used to add custom entries to the /json/info part of the JSON API.
     * Creating an "u" object allows you to add custom key/value pairs to the Info section of the WLED web UI.
     * Below it is shown how this could be used for e.g. a light sensor
     */
    void addToJsonInfo(JsonObject& root)
    {
      DEBUG_PRINT(FPSTR(_name));
      DEBUG_PRINTLN(F(" addToJsonInfo"));
      // if "u" object does not exist yet wee need to create it
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");

      if (!enabled) {
        JsonArray touchArr = user.createNestedArray(FPSTR(_name));
        touchArr.add(F("disabled"));
        touchArr.add(F(""));
        return;
      } 
      if (calibrationStep<2) {
        JsonArray touchArr = user.createNestedArray(FPSTR(_name));
        touchArr.add(F("calibrating"));
        touchArr.add(F("..."));
        return;
      }
      if (calibrationStep>=2) {
        for (uint p=0; p<pins; p++) {
          String temp;
          temp += FPSTR(_name);
          temp += F(" Pin ");
          temp += p;
          temp += F(" GPIO");
          temp += pin[p];
          JsonArray touchArr = user.createNestedArray(temp);
          if (pin[p] < 0) {
            temp = F(" Disabled ");
          } else {
            // read - copy from update function
            float cap      = capLast[p];   // total capacitance in hw_cycles (equal to about 0.25pF)
            float capDelta = pinNoMean[p]   ? cap          : cap-capMean[p];
            float capMin   = pinCapMin[p]>0 ? pinCapMin[p] : guard * sqrtf(capVar[p]);
            float capMax   = pinCapMax[p]>0 ? pinCapMax[p] : 30;
            float analog   = (capDelta - capMin) / (capMax-capMin);

            if (updateMean)
              temp = F("EMA: ");
            else
              temp = F("Mean: ");  
            temp += capMean[p];
            temp += F("&#177;");
            temp += sqrtf(capVar[p]);

            temp += F("<br><details><summary>");
            temp += F("Cap: ");
            temp += cap;
            temp += F("</summary>");

            temp += F("<br>CapDelta: ");
            temp += capDelta;
            temp += F("<br>CapThr: ");
            temp += capMin;
            temp += F("<br>CapMax: ");
            temp += capMax;

            temp += F("<br>State");
            temp += state[p];
            temp += F("<br>Analog: ");
            temp += analog*100;
            temp += F("%");
            temp += F("</details>");
          }
          touchArr.add(temp);
        }
      }

      // if you are implementing a sensor usermod, you may publish sensor data
      //JsonObject sensor = root[F("sensor")];
      //if (sensor.isNull()) sensor = root.createNestedObject(F("sensor"));
      //temp = sensor.createNestedArray(F("light"));
      //temp.add(reading);
      //temp.add(F("lux"));
    }

    /**
     * addToConfig() (called from set.cpp) stores persistent properties to cfg.json
     */
    void addToConfig(JsonObject& root)
    {
      // general
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[FPSTR(_enabled)] = enabled; //save these vars persistently whenever settings are saved
      top[FPSTR(_nsigma)] = guard;
      top[FPSTR(_debounce)] = debounceLen;
      top[FPSTR(_lptime)] = millisLongTouch;
      top[FPSTR(_umean)] = updateMean;
      top[FPSTR(_uvariance)] = updateVariance;
      
      // per pin
      for (int p=0; p<pins; p++) {
        String pinName = "pin"; pinName += p;
        JsonObject pinConfig = top.createNestedObject(pinName);
        pinConfig[FPSTR(_gpio)] = pin[p];
        pinConfig[FPSTR(_capmin)] = pinCapMin[p];
        pinConfig[FPSTR(_capmax)] = pinCapMax[p];
        pinConfig[FPSTR(_nomean)] = pinNoMean[p];
        pinConfig[FPSTR(_mode)] = pinModex[p];
        JsonArray pinActions = pinConfig.createNestedArray(FPSTR(_actions));
        for (int a=0; a<actions; a++)
          pinActions.add(action[p][a]);
      } 
    }
  
    /**
     * readFromConfig() is called before setup() to populate properties from values stored in cfg.json
     *
     * The function should return true if configuration was successfully loaded or false if there was no configuration.
     */
    bool readFromConfig(JsonObject &root)
    {
      //return true;

      // previous pin enable states for reallocation
      bool oldEnabled = enabled;
      int8_t oldPin[pins];
      uint8_t oldPinMode[pins];
      memcpy(oldPin, pin, pins);
      memcpy(oldPinMode, pinModex, pins);

      DEBUG_PRINT(FPSTR(_name));

      // general
      JsonObject top = root[FPSTR(_name)];
      if (top.isNull()) {
        DEBUG_PRINTLN(F(": No config found. (Using defaults.)"));
        return false;
      }

      enabled         = top[FPSTR(_enabled)]    | enabled;
      guard           = top[FPSTR(_nsigma)]      | guard;
      debounceLen     = top[FPSTR(_debounce)]   | debounceLen;
      millisLongTouch = top[FPSTR(_lptime)]     | millisLongTouch;
      updateMean      = top[FPSTR(_umean)]      | updateMean;
      updateVariance  = top[FPSTR(_uvariance)]  | updateVariance;

      // per pin 
      for (int p=0; p<pins; p++) {
        String pinName = "pin"; pinName += p;
        JsonObject pinConfig = top[pinName];
        if (pinConfig.isNull()) {
          DEBUG_PRINTLN(F(": No pin config found. (Using defaults.)"));
          return false;
        }

        pin[p]        = pinConfig[FPSTR(_gpio)]   | pin[p];
        pinModex[p]    = pinConfig[FPSTR(_mode)]   | pinModex[p];
        pinCapMin[p]  = pinConfig[FPSTR(_capmin)] | pinCapMin[p];
        pinCapMax[p]  = pinConfig[FPSTR(_capmax)] | pinCapMax[p];
        pinNoMean[p]  = pinConfig[FPSTR(_nomean)] | pinNoMean[p];

        JsonArray actionArray = pinConfig[FPSTR(_actions)];
        if (actionArray.isNull()) {
          DEBUG_PRINTLN(F(": No action config found. (Using defaults.)"));
          return false;
        }

        for(int a=0; a<actions && a<actionArray.size() ; a++) {
          action[p][a] = actionArray[a] | action[p][a];
          action[p][a] = max(0,min(255,(int)action[p][a]));
        }
      }
      

      if (!initDone) {
        // reading config prior to setup()
        DEBUG_PRINTLN(F(" config loaded."));
      } 
      else
      {
        if (enabled && !oldEnabled)
          touchInit();

        for (int p=0; p<pins; p++) {
          if (oldEnabled && (!enabled || pin[p] != oldPin[p]))
            touchPinDeInit(oldPin[p]);
          if (enabled && (!oldEnabled || pin[p] != oldPin[p])) 
            touchPinInit(&pin[p]);
        }
          
        if (!enabled && oldEnabled)
          touchDeInit();
        
        calibrationStep = 0; //recalibrate
        initDone = true;

        DEBUG_PRINTLN(F(" config (re)loaded."));
      }
      // use "return !top["newestParameter"].isNull();" when updating Usermod with new features
      return !top[FPSTR(_gpio)].isNull();
    }

    /**
     * getId() allows you to optionally give your V2 usermod an unique ID (please define it in const.h!).
     * This could be used in the future for the system to determine whether your usermod is installed.
     */
    /*uint16_t getId()
    {
      return USERMOD_ID_TOUCHADV;
    }*/
};


// strings to reduce flash memory usage (used more than twice)
const char TouchAdvancedUsermod::_name[]        PROGMEM = "TouchAdvanced";
const char TouchAdvancedUsermod::_enabled[]     PROGMEM = "Enabled";
const char TouchAdvancedUsermod::_nsigma[]      PROGMEM = "ThresholdSigma";
const char TouchAdvancedUsermod::_debounce[]    PROGMEM = "Debounce";
const char TouchAdvancedUsermod::_lptime[]      PROGMEM = "LongTouch";
const char TouchAdvancedUsermod::_umean[]       PROGMEM = "UpdateMean";
const char TouchAdvancedUsermod::_uvariance[]   PROGMEM = "UpdateSigma";
const char TouchAdvancedUsermod::_nomean[]      PROGMEM = "CapsIncludePad";
const char TouchAdvancedUsermod::_capmin[]      PROGMEM = "ThresholdCap";
const char TouchAdvancedUsermod::_capmax[]      PROGMEM = "FullCap";
const char TouchAdvancedUsermod::_gpio[]        PROGMEM = "GPIO";
const char TouchAdvancedUsermod::_mode[]        PROGMEM = "Mode";
const char TouchAdvancedUsermod::_actions[]     PROGMEM = "Actions";

// ideas:
// fix adaptive time, add setting
// analog input active switch, lockout, activate related
// statistics