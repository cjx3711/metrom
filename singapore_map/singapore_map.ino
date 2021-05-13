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

#define STATE_HOLD_OFF 1
#define STATE_TRANSITION_ON 2
#define STATE_HOLD_ON 3
#define STATE_TRANSITION_OFF 4



uint8_t lightingState;
uint8_t stateTicksTotal;
uint8_t stateTicksLeft;

uint8_t currentPatternState;


// Button states for MODE, BRIGHTNESS
bool buttonStatesPrev [2] = {false, false};
bool buttonStates [2] = {false, false};
bool firstRun = true;

float brightnessLevels[8] = { 1.0f, 0.7f, 0.5f, 0.35f, 0.2f, 0.1f, 0.05f, 0.025f};
int currentBrightnessLevel = 0;

int brightness = 0;

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
    currentPatternState++;
    if (currentPatternState == totalStates) {
      currentPatternState = 0;
      firstRun = false;
    }
  }
  uint8_t getState() {
    return states[currentPatternState];
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
uint8_t pattern1[] = {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000};
uint8_t pattern2[] = {0b00000001, 0b00000011, 0b00000111, 0b00001111, 0b00011111, 0b00111111};
uint8_t pattern3[] = {0b00000001, 0b00000011, 0b00000111, 0b00001111, 0b00011111, 0b00111111, 0b00011111, 0b00001111, 0b00000111, 0b00000011};
uint8_t pattern4[] = {0b00111111};


void setupAnimatedPatterns() {
  animatedPatternCurrent = new AnimatedPattern(pattern1, 6, 40, 10, 0);
  animatedPatternCurrent = new AnimatedPattern(pattern2, 6, 40, 10, 0, animatedPatternCurrent);
  animatedPatternCurrent = new AnimatedPattern(pattern3, 10, 40, 10, 0, animatedPatternCurrent);
  animatedPatternCurrent = new AnimatedPattern(pattern4, 1, 200, 0, 0, animatedPatternCurrent);
  animatedPatternCurrent = new AnimatedPattern(pattern4, 1, 20, 200, 20, animatedPatternCurrent);

  animatedPatternHead = animatedPatternCurrent;
}

void nextPattern() {
  if (animatedPatternCurrent->nextPattern) {
    animatedPatternCurrent = animatedPatternCurrent->nextPattern;
  } else {
    animatedPatternCurrent = animatedPatternHead;
  }
  lightingState = STATE_HOLD_OFF;
  currentPatternState = 0;
  firstRun = true;
  stateTicksLeft = stateTicksTotal = animatedPatternCurrent->getTicksOff();
}

bool buttonRelease(uint8_t btn) {
  return !buttonStates[btn] && buttonStatesPrev[btn];
}

void buttonStatePreLoop() {
  uint16_t input = analogRead(BUTTON_PIN);
  if (input < 10) { //
    buttonStates[BTN_LIGHT] = buttonStates[BTN_MODE] = 0;
  } else {
    buttonStates[BTN_MODE] = input > 750;
    buttonStates[BTN_LIGHT] = !buttonStates[BTN_MODE];
  }
}

void buttonStatePostLoop() {
  for ( int i = 0; i < 2; i++ ) buttonStatesPrev[i] = buttonStates[i];
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

  setupAnimatedPatterns();
  nextPattern();
}

void loop() {
  // Calculate buttons
  buttonStatePreLoop();

  if (buttonRelease(BTN_LIGHT)) 
    currentBrightnessLevel ++;
  if (currentBrightnessLevel >= 8)
    currentBrightnessLevel = 0;

  if (buttonRelease(BTN_MODE))
    nextPattern();

  buttonStatePostLoop();

  // Process lighting state
  // If this state is 0 ticks, it will skip the rest of the loop
  
  float percentage;
  switch(lightingState) {
    case STATE_HOLD_OFF:
      brightness = 0;
      break;
    case STATE_TRANSITION_ON:
      percentage = (float)stateTicksLeft / (float)stateTicksTotal;
      brightness = (1-percentage) * 255.0f;
      break;
    case STATE_HOLD_ON:
      brightness = 255;
      break;
    case STATE_TRANSITION_OFF:
      percentage = (float)stateTicksLeft / (float)stateTicksTotal;
      brightness = percentage * 255.0f;
      break;
  }
  // State changing
  bool skipLoop = false;
  if (stateTicksTotal == 0) skipLoop = true;
  if (stateTicksLeft == 0) {
    switch(lightingState) {
      case STATE_HOLD_OFF:
        lightingState = STATE_TRANSITION_ON;
        stateTicksTotal = firstRun ? 3 : animatedPatternCurrent->getTicksAnimate();
        break;
      case STATE_TRANSITION_ON:
        lightingState = STATE_HOLD_ON;
        stateTicksTotal = firstRun ? 1 : animatedPatternCurrent->getTicksOn();
        break;
      case STATE_HOLD_ON:
        lightingState = STATE_TRANSITION_OFF;
        stateTicksTotal = firstRun ? 8 : animatedPatternCurrent->getTicksAnimate();
        break;
      case STATE_TRANSITION_OFF:
        lightingState = STATE_HOLD_OFF;
        stateTicksTotal = firstRun ? 1 : animatedPatternCurrent->getTicksOff();
        animatedPatternCurrent->nextState();
        break;
    }
    stateTicksLeft = stateTicksTotal;
    if (skipLoop) return; // If the previous state had 0 ticks, we will not update the brightness or lighting.
  } else {
    stateTicksLeft--;
  }
  // Set pin brightness
  brightness = brightness * brightnessLevels[currentBrightnessLevel];
  analogWrite(BRIGHTNESS_PIN, 255 - brightness); // Invert the brightness value

  // Set the image state
  digitalWrite(SR_LATCH_PIN, LOW);
  shiftOut(SR_DATA_PIN, SR_CLOCK_PIN, MSBFIRST, animatedPatternCurrent->getState());
  digitalWrite(SR_LATCH_PIN, HIGH);

  // puts("Brightness "); putsln(brightness);
  // puts(currentPatternState); puts(' ');
  // puts(animatedPatternCurrent->getTicksOn()); puts(' ');
  // puts(animatedPatternCurrent->getTicksAnimate()); puts(' ');
  // puts(animatedPatternCurrent->getTicksOff()); puts(' ');
  // putsln(animatedPatternCurrent->getState());
  // animatedPatternCurrent->getCurrentState()->printState();
  // puts("State Ticks "); putsln(stateTicksLeft);
  // puts("Anim State "); putsln(lightingState);

  // For testing on ATTINY without serial
  // analogWrite(BRIGHTNESS_PIN, (millis() / 10) % 255);

  delay(10);
}