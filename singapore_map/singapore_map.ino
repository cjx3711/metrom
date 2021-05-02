// Uncomment this line to set the pins to arduino uno
#define ARDUINO_DEBUG_MODE

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
  #define puts(x) (void(0)) // Disable prints on the ATTINY
  #define putsln(x) (void(0)) // Disable prints on the ATTINY
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

// The animation will work in 4 states
// STATE_HOLD_OFF
// STATE_TRANSITION_ON
// STATE_HOLD_ON
// STATE_TRANSITION_OFF

class AnimatedState {
  // Each state starts on the OFF state for ticksOff time,
  // then it takes ticksAnimate ticks to turn on,
  // then it holds the on state for ticksOn
  public:
  bool * state;
  uint8_t ticksOn;
  uint8_t ticksOff;
  uint8_t ticksAnimate;
  AnimatedState * next;
  AnimatedState(bool _state[6], uint8_t _ticksOn, uint8_t _ticksAnimate, uint8_t _ticksOff) {
    state = _state;
    ticksOn = _ticksOn;
    ticksAnimate = _ticksAnimate;
    ticksOff = _ticksOff;
    next = NULL;
  }

  void printState() {
    puts(state[0]); puts(' ');
    puts(state[1]); puts(' ');
    puts(state[2]); puts(' ');
    puts(state[3]); puts(' ');
    puts(state[4]); puts(' ');
    putsln(state[5]);
  }
};

class AnimatedPattern {
  private:
  AnimatedState * headState;
  AnimatedState * currentState;
  AnimatedPattern * nextPattern;

  public:
  AnimatedPattern() {
    headState = NULL;
    nextPattern = NULL;
  }
  AnimatedPattern(AnimatedPattern * _nextPattern) {
    headState = NULL;
    nextPattern = _nextPattern;
  }
  void addState(AnimatedState * state) {
    // Add to the tail by looping to the end
    if (headState == NULL) {
      headState = state;
      currentState = state;
    } else {
      AnimatedState * pointer = headState;
      while (pointer->next) pointer = pointer->next;
      pointer->next = state;
    }

  }
  void nextState() {
    if (currentState->next == NULL) {
      currentState = headState;
    } else {
      currentState = currentState->next;
    }
  }

  AnimatedState * getCurrentState() {
    return currentState;
  }

  void animate() {

  }
};

AnimatedPattern * animatedPatternHead;
AnimatedPattern * animatedPatternCurrent;

bool * arrGen(bool a, bool b, bool c, bool d, bool e, bool f) {
  bool * arr = new bool[6];
  arr[0] = a; arr[1] = b; arr[2] = c; arr[3] = d; arr[4] = e; arr[5] = f;
  return arr;
}

void setupAnimatedPatterns() {
  AnimatedState * state;
  animatedPatternCurrent = new AnimatedPattern();
  animatedPatternCurrent->addState(new AnimatedState(arrGen(1, 0, 0, 0, 0 ,0), 20, 10, 0));
  animatedPatternCurrent->addState(new AnimatedState(arrGen(0, 1, 0, 0, 0 ,0), 20, 10, 0));
  animatedPatternCurrent->addState(new AnimatedState(arrGen(0, 0, 1, 0, 0 ,0), 20, 10, 0));
  animatedPatternCurrent->addState(new AnimatedState(arrGen(0, 0, 0, 1, 0 ,0), 20, 10, 0));
  animatedPatternCurrent->addState(new AnimatedState(arrGen(0, 0, 0, 0, 1 ,0), 20, 10, 0));
  animatedPatternCurrent->addState(new AnimatedState(arrGen(0, 0, 0, 0, 0 ,1), 20, 10, 0));

  animatedPatternCurrent = new AnimatedPattern(animatedPatternCurrent);
  animatedPatternCurrent->addState(new AnimatedState(arrGen(0, 0, 0, 0, 0 ,0), 50, 5, 0));
  animatedPatternCurrent->addState(new AnimatedState(arrGen(1, 0, 0, 0, 0 ,0), 50, 5, 0));
  animatedPatternCurrent->addState(new AnimatedState(arrGen(1, 1, 0, 0, 0 ,0), 50, 5, 0));
  animatedPatternCurrent->addState(new AnimatedState(arrGen(1, 1, 1, 0, 0 ,0), 50, 5, 0));
  animatedPatternCurrent->addState(new AnimatedState(arrGen(1, 1, 1, 1, 0 ,0), 50, 5, 0));
  animatedPatternCurrent->addState(new AnimatedState(arrGen(1, 1, 1, 1, 1 ,0), 50, 5, 0));
  animatedPatternCurrent->addState(new AnimatedState(arrGen(1, 1, 1, 1, 1 ,1), 50, 5, 0));

  animatedPatternCurrent = new AnimatedPattern(animatedPatternCurrent);
  animatedPatternCurrent->addState(new AnimatedState(arrGen(0, 0, 0, 0, 0 ,0), 5, 50, 0));
  animatedPatternCurrent->addState(new AnimatedState(arrGen(1, 0, 0, 0, 0 ,0), 5, 50, 0));
  animatedPatternCurrent->addState(new AnimatedState(arrGen(1, 1, 0, 0, 0 ,0), 5, 50, 0));
  animatedPatternCurrent->addState(new AnimatedState(arrGen(1, 1, 1, 0, 0 ,0), 5, 50, 0));
  animatedPatternCurrent->addState(new AnimatedState(arrGen(1, 1, 1, 1, 0 ,0), 5, 50, 0));
  animatedPatternCurrent->addState(new AnimatedState(arrGen(1, 1, 1, 1, 1 ,0), 5, 50, 0));
  animatedPatternCurrent->addState(new AnimatedState(arrGen(1, 1, 1, 1, 1 ,1), 5, 50, 0));

  animatedPatternHead = animatedPatternCurrent;
}

// Button states for MODE, BRIGHTNESS
bool buttonStatesPrev [2] = {false, false};
bool buttonStates [2] = {false, false};

float brightnessLevels[8] = { 1.0f, 0.7f, 0.5f, 0.35f, 0.2f, 0.1f, 0.05f, 0.025f};
int currentBrightnessLevel = 0;

int brightness = 0;
float maxBrightness = 1;

bool buttonRelease(uint8_t btn) {
  return !buttonStates[btn] && buttonStatesPrev[btn];
}

bool buttonPress(uint8_t btn) {
  return buttonStates[btn] && !buttonStatesPrev[btn];
}


void buttonStatePreLoop() {
  uint16_t input = analogRead(BUTTON_PIN);
  if (input < 10) {
    buttonStates[BTN_LIGHT] = buttonStates[BTN_MODE] = 0;
  } else {
    buttonStates[BTN_MODE] = input > 750;
    buttonStates[BTN_LIGHT] = !buttonStates[BTN_MODE];
  }
}

void buttonStatePostLoop() {
  for ( int i = 0; i < 2; i++ ) buttonStatesPrev[i] = buttonStates[i];
}


void setup()
{
  pinMode(SR_LATCH_PIN, OUTPUT);
  pinMode(SR_CLOCK_PIN, OUTPUT);
  pinMode(SR_DATA_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(BRIGHTNESS_PIN, OUTPUT);
  #ifdef ARDUINO_DEBUG_MODE
    Serial.begin(9600);
  #endif

  setupAnimatedPatterns();

  lightingState = STATE_HOLD_OFF;
  stateTicksLeft = stateTicksTotal = animatedPatternCurrent->getCurrentState()->ticksOff;
}

void loop() {
  // Calculate buttons
  buttonStatePreLoop();

  if (buttonRelease(BTN_LIGHT)) 
    currentBrightnessLevel ++;
  if (currentBrightnessLevel >= 8)
    currentBrightnessLevel = 0;

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
        stateTicksTotal = animatedPatternCurrent->getCurrentState()->ticksAnimate;
        break;
      case STATE_TRANSITION_ON:
        lightingState = STATE_HOLD_ON;
        stateTicksTotal = animatedPatternCurrent->getCurrentState()->ticksOn;
        break;
      case STATE_HOLD_ON:
        lightingState = STATE_TRANSITION_OFF;
        stateTicksTotal = animatedPatternCurrent->getCurrentState()->ticksAnimate;
        break;
      case STATE_TRANSITION_OFF:
        lightingState = STATE_HOLD_OFF;
        stateTicksTotal = animatedPatternCurrent->getCurrentState()->ticksOff;
        animatedPatternCurrent->nextState();
        break;
    }
    stateTicksLeft = stateTicksTotal;
    if (skipLoop) return; // If the previous state had 0 ticks, we will not update the brightness or lighting.
  } else {
    stateTicksLeft--;
  }
  // Set pin brightness
  // puts("Brightness "); putsln(brightness);
  brightness = brightness * brightnessLevels[currentBrightnessLevel];
  analogWrite(BRIGHTNESS_PIN, 255 - brightness); // Invert the brightness value

  // Set the image state
  uint8_t a = 0;
  for (uint8_t i = 6; i > 0; i-- ) {
    a = a << 1;
    a = a | animatedPatternCurrent->getCurrentState()->state[i-1];
  }
  digitalWrite(SR_LATCH_PIN, LOW);
  shiftOut(SR_DATA_PIN, SR_CLOCK_PIN, MSBFIRST, a);
  digitalWrite(SR_LATCH_PIN, HIGH);

  animatedPatternCurrent->getCurrentState()->printState();
  // puts("State Ticks "); putsln(stateTicksLeft);
  // puts("Anim State "); putsln(lightingState);

  delay(10);
}