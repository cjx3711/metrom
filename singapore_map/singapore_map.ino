// Uncomment this line to set the pins to arduino uno
// #define ARDUINO_DEBUG_MODE

#ifdef ARDUINO_DEBUG_MODE
  // Arduino Uno Pins
  #define SR_LATCH_PIN 5 // Pin connected to ST_CP of 74HC595
  #define SR_CLOCK_PIN 4 // Pin connected to SH_CP of 74HC595
  #define SR_DATA_PIN 6 // Pin connected to DS of 74HC595
  #define BUTTON_PIN A0 // Analogue Pin
  #define BRIGHTNESS_PIN 3 // Sink pin
  #define puts(x) Serial.print(x)
  #define putsln(x) Serial.println(x)
#else
  // ATTINY Pins
  #define SR_LATCH_PIN 3 // Pin connected to ST_CP of 74HC595
  #define SR_CLOCK_PIN 4 // Pin connected to SH_CP of 74HC595
  #define SR_DATA_PIN 1 // Pin connected to DS of 74HC595
  #define BUTTON_PIN A1 // Analogue Pin (A1)
  #define BRIGHTNESS_PIN 0 // Sink pin
  // #define puts(x) (void(0)) // Disable prints on the ATTINY
  // #define putsln(x) (void(0)) // Disable prints on the ATTINY
#endif

#define BTN_MODE 0
#define BTN_LIGHT 1

#define STATE_HOLD_OFF 0
#define STATE_TRANSITION_ON 1
#define STATE_HOLD_ON 2
#define STATE_TRANSITION_OFF 3

// 
/*
Without Bitfields or structs
Sketch uses 3362 bytes (41%) of program storage space. Maximum is 8192 bytes.
Global variables use 81 bytes of dynamic memory.

Without Bitfields with structs
Sketch uses 3442 bytes (42%) of program storage space. Maximum is 8192 bytes.
Global variables use 85 bytes of dynamic memory.

With bitfields and structs
Sketch uses 3588 bytes (43%) of program storage space. Maximum is 8192 bytes.
Global variables use 80 bytes of dynamic memory.

*/


struct GlobalState {
  int lightingState: 2;
  uint8_t stateTicksTotal: 8;
  uint8_t stateTicksLeft: 8;
  
  uint8_t currentPatternState: 8;
  // Button states for MODE, BRIGHTNESS
  bool modeStatePrev: 1;
  bool modeState: 1;
  bool lightStatePrev: 1;
  bool lightState: 1;

  int currentBrightnessLevel: 3;
  int brightness = 0;
  float maxBrightness = 1;
};



GlobalState state;

float brightnessLevels[8] = { 1.0f, 0.7f, 0.5f, 0.35f, 0.2f, 0.1f, 0.05f, 0.025f};


// The animation will work in 4 states
// STATE_HOLD_OFF
// STATE_TRANSITION_ON
// STATE_HOLD_ON
// STATE_TRANSITION_OFF

class AnimatedPattern {
  private:
  uint8_t * states; // 16
  uint8_t totalStates;
  uint8_t ticksOn; // 16
  uint8_t ticksOff; // 16
  uint8_t ticksAnimate; // 16


  public:
  AnimatedPattern * nextPattern;

  AnimatedPattern(uint8_t * _states, uint8_t _totalStates, uint8_t _ticksOn, uint8_t _ticksAnimate, uint8_t _ticksOff) {
    states = _states;
    totalStates = _totalStates;
    ticksOn = _ticksOn;
    ticksOff = _ticksOff;
    ticksAnimate = _ticksAnimate;
    nextPattern = NULL;
  }
  AnimatedPattern(uint8_t * _states, uint8_t _totalStates, uint8_t _ticksOn, uint8_t _ticksAnimate, uint8_t _ticksOff, AnimatedPattern * _nextPattern) {
    states = _states;
    totalStates = _totalStates;
    ticksOn = _ticksOn;
    ticksOff = _ticksOff;
    ticksAnimate = _ticksAnimate;
    nextPattern = _nextPattern;
  }
  void nextState() {
    state.currentPatternState++;
    if (state.currentPatternState == totalStates) state.currentPatternState = 0;
  }
  uint8_t getState() {
    return states[state.currentPatternState];
  }
  uint8_t getTicksOn() {
    return ticksOn;
  }
  uint8_t getTicksAnimate() {
    return ticksAnimate;
  }
  uint8_t getTicksOff() {
    return ticksOff;
  }
};

AnimatedPattern * animatedPatternHead;
AnimatedPattern * animatedPatternCurrent;

// Patterns are defined as integers, but only the first 6 bits from the right are used.
// 1 represents on an state, while 0 represents an off state.
// The bits from right to left control the following lines respectively
// NSL, EWL, NEL, CCL, DTL, TEL.
// For example:
// 0b00000011
// NSL and EWL are on, while the rest are not.
// Simple Cycle Pattern
uint8_t pattern1[] = {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000};
// Appear in order of build
uint8_t pattern2[] = {0b00000001, 0b00000011, 0b00000111, 0b00001111, 0b00011111, 0b00111111};
// All on
uint8_t pattern3[] = {0b00111111};


void setupAnimatedPatterns() {
  // 48 * 3 + 7 * 3 * 62 = 1446
  animatedPatternCurrent = new AnimatedPattern(pattern1, 6, 20, 10, 0);
  animatedPatternCurrent = new AnimatedPattern(pattern1, 6, 200, 20, 0, animatedPatternCurrent);
  animatedPatternCurrent = new AnimatedPattern(pattern2, 6, 5, 100, 0, animatedPatternCurrent);
  animatedPatternCurrent = new AnimatedPattern(pattern2, 6, 200, 20, 0, animatedPatternCurrent);
  animatedPatternCurrent = new AnimatedPattern(pattern3, 1, 200, 0, 0, animatedPatternCurrent);
  animatedPatternCurrent = new AnimatedPattern(pattern3, 1, 0, 200, 0, animatedPatternCurrent);

  animatedPatternHead = animatedPatternCurrent;
}

void nextPattern() {
  if (animatedPatternCurrent->nextPattern) {
    animatedPatternCurrent = animatedPatternCurrent->nextPattern;
  } else {
    animatedPatternCurrent = animatedPatternHead;
  }
  state.lightingState = STATE_HOLD_OFF;
  state.currentPatternState = 0;
  state.stateTicksLeft = state.stateTicksTotal = animatedPatternCurrent->getTicksOff();
}

void buttonStatePreLoop() {
  uint16_t input = analogRead(BUTTON_PIN);
  if (input < 10) {
    state.lightState = state.modeState = 0;
  } else {
    state.modeState = input > 750;
    state.lightState = !state.modeState;
  }
}

void buttonStatePostLoop() {
  state.modeStatePrev = state.modeState;
  state.lightStatePrev = state.lightState;
}


void setup() {
  pinMode(SR_LATCH_PIN, OUTPUT);
  pinMode(SR_CLOCK_PIN, OUTPUT);
  pinMode(SR_DATA_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(BRIGHTNESS_PIN, OUTPUT);
  #ifdef ARDUINO_DEBUG_MODE
    Serial.begin(9600);
  #endif

  state.modeState = state.modeStatePrev = state.lightState = state.lightStatePrev = false;
  state.currentBrightnessLevel = 0;

  setupAnimatedPatterns();
  nextPattern();
}

void loop() {
  // Calculate buttons
  buttonStatePreLoop();

  if (!state.lightState && state.lightStatePrev) // Light button is released
    state.currentBrightnessLevel ++;
  if (state.currentBrightnessLevel >= 8)
    state.currentBrightnessLevel = 0;

  if (!state.modeState && state.modeStatePrev) // Mode button is released
    nextPattern();

  buttonStatePostLoop();

  // Process lighting state
  // If this state is 0 ticks, it will skip the rest of the loop
  
  float percentage;
  switch(state.lightingState) {
    case STATE_HOLD_OFF:
      state.brightness = 0;
      break;
    case STATE_TRANSITION_ON:
      percentage = (float)state.stateTicksLeft / (float)state.stateTicksTotal;
      state.brightness = (1-percentage) * 255.0f;
      break;
    case STATE_HOLD_ON:
      state.brightness = 255;
      break;
    case STATE_TRANSITION_OFF:
      percentage = (float)state.stateTicksLeft / (float)state.stateTicksTotal;
      state.brightness = percentage * 255.0f;
      break;
  }
  // State changing
  bool skipLoop = false;
  if (state.stateTicksTotal == 0) skipLoop = true;
  if (state.stateTicksLeft == 0) {
    switch(state.lightingState) {
      case STATE_HOLD_OFF:
        state.lightingState = STATE_TRANSITION_ON;
        state.stateTicksTotal = animatedPatternCurrent->getTicksAnimate();
        break;
      case STATE_TRANSITION_ON:
        state.lightingState = STATE_HOLD_ON;
        state.stateTicksTotal = animatedPatternCurrent->getTicksOn();
        break;
      case STATE_HOLD_ON:
        state.lightingState = STATE_TRANSITION_OFF;
        state.stateTicksTotal = animatedPatternCurrent->getTicksAnimate();
        break;
      case STATE_TRANSITION_OFF:
        state.lightingState = STATE_HOLD_OFF;
        state.stateTicksTotal = animatedPatternCurrent->getTicksOff();
        animatedPatternCurrent->nextState();
        break;
    }
    state.stateTicksLeft = state.stateTicksTotal;
    if (skipLoop) return; // If the previous state had 0 ticks, we will not update the state.brightness or lighting.
  } else {
    state.stateTicksLeft--;
  }
  // Set pin state.brightness
  state.brightness = state.brightness * brightnessLevels[state.currentBrightnessLevel];
  analogWrite(BRIGHTNESS_PIN, 255 - state.brightness); // Invert the state.brightness value

  // Set the image state
  digitalWrite(SR_LATCH_PIN, LOW);
  shiftOut(SR_DATA_PIN, SR_CLOCK_PIN, MSBFIRST, animatedPatternCurrent->getState());
  digitalWrite(SR_LATCH_PIN, HIGH);

  // puts("Brightness "); putsln(state.brightness);
  // puts(state.currentPatternState); puts(' ');
  // puts(animatedPatternCurrent->getTicksOn()); puts(' ');
  // puts(animatedPatternCurrent->getTicksAnimate()); puts(' ');
  // puts(animatedPatternCurrent->getTicksOff()); puts(' ');
  // putsln(animatedPatternCurrent->getState());
  // animatedPatternCurrent->getCurrentState()->printState();
  // puts("State Ticks "); putsln(state.stateTicksLeft);
  // puts("Anim State "); putsln(state.lightingState);

  // For testing on ATTINY without serial
  // analogWrite(BRIGHTNESS_PIN, (millis() / 10) % 255);

  delay(10);
}