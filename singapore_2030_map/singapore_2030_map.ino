#include <EEPROM.h>

// These can be modified to change the program behaviour
#define MAX_LENGTH_MULTIPLIER 10
#define LONG_PRESS_TICKS 40
#define PREVIEW_OFF 2
#define PREVIEW_TRANSITION 1
#define PREVIEW_ON 9

// Uncomment this line to set the pins to arduino uno
#define DEBUG_MODE

// #define ARDUINO_UNO_MODE
#ifdef ARDUINO_UNO_MODE
  // Arduino Uno Pins
  #define SR_LATCH_PIN 5 // Pin connected to ST_CP of 74HC595
  #define SR_CLOCK_PIN 4 // Pin connected to SH_CP of 74HC595
  #define SR_DATA_PIN 6 // Pin connected to DS of 74HC595
  #define BUTTON_PIN A0 // Analogue Pin
  #define BRIGHTNESS_PIN 3 // Sink pin
  #define puts(x) Serial.print(x)
  #define putsln(x) Serial.println(x)
#else
  // Arduino Pro Micro Pins
  #define SR_LATCH_PIN 7 // Pin connected to ST_CP of 74HC595 (LATCH)
  #define SR_CLOCK_PIN 8 // Pin connected to SH_CP of 74HC595 (CLOCK)
  #define SR_DATA_PIN 6 // Pin connected to DS of 74HC595 (DATA)
  #define BUTTON1_PIN A0 // Analogue Pin
  #define BUTTON2_PIN A1 // Analogue Pin
  #define BUTTON3_PIN A2 // Analogue Pin
  #define BRIGHTNESS_PIN 9 // Brightness Control (LIGHT)
  #define puts(x) Serial.print(x)
  #define putsln(x) Serial.println(x)
#endif

/*    ATTINY85 Pins
 * RESET +---+ VCC
 * LATCH |   | BTN
 * CLOCK |   | DATA
 *   GND +---+ LIGHT
 */

/*     Arduino Pro Mini Pins
 *        TX0 +------+ RAW
 *        RX1 |      | GND
 *        RST |      | RST
 *        GND |      | VCC
 *         D2 |      | A3
 *         D3 |      | A2  BTN3
 *         D4 |      | A1  BTN2
 *         D5 |      | A0  BTN1
 * DATA    D6 |      | D13
 * LATCH   D7 |      | D12
 * CLOCK   D8 |      | D11
 * LIGHT   D9 +------+ D10
 */


// These should not be touched and are required for the program to run
#define BTN_SPEED 0
#define BTN_LIGHT 1
#define BTN_MODE 2

#define STATE_HOLD_OFF 1
#define STATE_TRANSITION_ON 2
#define STATE_HOLD_ON 3
#define STATE_TRANSITION_OFF 4

// Since there are 2 resistors on the buttons,
// we need to use the analog function of the button.
#define BUTTON_OFF_THRESHOLD 60



uint8_t lightingState;
uint16_t stateTicksTotal;
uint16_t stateTicksLeft;

uint8_t currentPatternState;


// Button states for SPEED, BRIGHTNESS, MODE
uint8_t btnTicks = 0;
bool buttonStatesPrev [3] = {false, false, false};
bool buttonStates [3] = {false, false, false};
bool firstRun = true;

float brightnessLevels[8] = { 1.0f, 0.7f, 0.5f, 0.35f, 0.2f, 0.1f, 0.05f, 0.025f};
uint8_t currentBrightnessLevel = 0;
uint8_t lengthMultiplier = 0;
uint8_t currentPatternId = 0;
uint8_t randomState = random();

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

  /*
    _states: array of the different states
    _totalStates: total states in the array
    _ticksOn: Number of ticks it stays on
    _ticksAnimate: Number of ticks it takes to fade in and out
    _ticksOff: Number of ticks it stays off
    _nextPattern: The next pattern in the linked list
    Note: Each tick is slightly more than 10ms but can be assumed to be 10ms
  */
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
    if (currentPatternState == totalStates || totalStates == 0) {
      currentPatternState = 0;
      firstRun = false;
    }
    if (totalStates == 0) {
      randomState = random();
    }
  }
  uint8_t getState() {
    if (totalStates == 0) {
      return randomState;
    }
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
uint8_t pattern1[] = {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b01000000, 0b10000000};
uint8_t pattern2[] = {0b00000001, 0b00000011, 0b00000111, 0b00001111, 0b00011111, 0b00111111, 0b01111111, 0b11111111};
uint8_t pattern3[] = {0b00000001, 0b00000011, 0b00000111, 0b00001111, 0b00011111, 0b00111111, 0b01111111, 0b11111111, 0b01111111, 0b00111111, 0b00011111, 0b00001111, 0b00000111, 0b00000011};
uint8_t pattern4[] = {0b11111111};


// Includes the random pattern
#define TOTAL_PATTERNS 6
void setupAnimatedPatterns() {
  
  animatedPatternCurrent = new AnimatedPattern(NULL, 0, 40, 10, 0, NULL); // Randomly generated
  animatedPatternCurrent = new AnimatedPattern(pattern1, 8, 40, 10, 0, animatedPatternCurrent);
  animatedPatternCurrent = new AnimatedPattern(pattern2, 8, 40, 10, 0, animatedPatternCurrent);
  animatedPatternCurrent = new AnimatedPattern(pattern3, 14, 40, 10, 0, animatedPatternCurrent);
  animatedPatternCurrent = new AnimatedPattern(pattern4, 1, 10, 100, 10, animatedPatternCurrent);
  animatedPatternCurrent = new AnimatedPattern(pattern4, 1, 100, 0, 0, animatedPatternCurrent);

  animatedPatternHead = animatedPatternCurrent;
  lightingState = STATE_HOLD_OFF;
  currentPatternId = 0;
  currentPatternState = 0;
  firstRun = true;
  stateTicksLeft = stateTicksTotal = animatedPatternCurrent->getTicksOff();
}

void nextPattern() {
  if (animatedPatternCurrent->nextPattern) {
    animatedPatternCurrent = animatedPatternCurrent->nextPattern;
    currentPatternId++;
  } else {
    animatedPatternCurrent = animatedPatternHead;
    currentPatternId = 0;
  }
  EEPROM.update(3, currentPatternId);

  lightingState = STATE_HOLD_OFF;
  currentPatternState = 0;
  firstRun = true;
  stateTicksLeft = stateTicksTotal = animatedPatternCurrent->getTicksOff();
}

bool buttonRelease(uint8_t btn) {
  return !buttonStates[btn] && buttonStatesPrev[btn];
}

void buttonStatePreLoop() {
  for ( int i = 0; i < 3; i++ ) buttonStates[i] = false;
  if (analogRead(BUTTON1_PIN) > BUTTON_OFF_THRESHOLD) buttonStates[BTN_MODE] = true;
  if (analogRead(BUTTON2_PIN) > BUTTON_OFF_THRESHOLD) buttonStates[BTN_LIGHT] = true;
  if (analogRead(BUTTON3_PIN) > BUTTON_OFF_THRESHOLD) buttonStates[BTN_SPEED] = true;
}

void buttonStatePostLoop() {
  for ( int i = 0; i < 3; i++ ) buttonStatesPrev[i] = buttonStates[i];
}

void blinkDebugLight() {
  digitalWrite(13, HIGH);
  delay(150);
  digitalWrite(13, LOW);
  delay(50);
}

void setup() {
  pinMode(13, OUTPUT);
  blinkDebugLight();
  randomSeed(analogRead(A3));
  pinMode(SR_LATCH_PIN, OUTPUT);
  pinMode(SR_CLOCK_PIN, OUTPUT);
  pinMode(SR_DATA_PIN, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(BUTTON3_PIN, INPUT);
  pinMode(BRIGHTNESS_PIN, OUTPUT);
  #ifdef DEBUG_MODE
    Serial.begin(9600);
  #endif

  uint8_t gotoPatternId = 0;
  uint8_t isSetup = EEPROM.read(0);
  if (isSetup != 128) {
    EEPROM.update(0, 128);
    EEPROM.update(1, 0);
    EEPROM.update(2, 0);
    EEPROM.update(3, 0);
    putsln("First time initialisation");
  } else {
    currentBrightnessLevel = EEPROM.read(1);
    lengthMultiplier = EEPROM.read(2);
    gotoPatternId = EEPROM.read(3);
    currentBrightnessLevel = currentBrightnessLevel % 8;
    lengthMultiplier = lengthMultiplier % MAX_LENGTH_MULTIPLIER;
    gotoPatternId = gotoPatternId % TOTAL_PATTERNS;
    putsln("Loaded from memory");
    puts("Pattern: ");
    putsln(gotoPatternId);
    puts("Brightness: ");
    putsln(currentBrightnessLevel);
    puts("Length: ");
    putsln(lengthMultiplier);
  }
  
  blinkDebugLight();
  
  setupAnimatedPatterns();
  for (int i = 0; i < gotoPatternId; i++) {
    nextPattern();
  }
  
  blinkDebugLight();
}

void loop() {
  // Calculate buttons
  buttonStatePreLoop();

  if (buttonRelease(BTN_SPEED)) {
    stateTicksLeft = 1;
    lightingState = STATE_HOLD_OFF;
    lengthMultiplier++;
    lengthMultiplier = lengthMultiplier % MAX_LENGTH_MULTIPLIER;
    EEPROM.update(2, lengthMultiplier);
  }
  if (buttonRelease(BTN_LIGHT)) {
    currentBrightnessLevel++;
    currentBrightnessLevel = currentBrightnessLevel % 8;
    EEPROM.update(1, currentBrightnessLevel);
  }
  if (buttonRelease(BTN_MODE)) {
    nextPattern();
    // Changing patterns will reset the length multiplier
    lengthMultiplier = 0;
    EEPROM.update(2, lengthMultiplier);
  }

  buttonStatePostLoop();

  // Process lighting state
  // If this state is 0 ticks, it will skip the rest of the loop
  int brightness;
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
        stateTicksTotal = firstRun ? PREVIEW_OFF : animatedPatternCurrent->getTicksAnimate() * ((lengthMultiplier*2) + 1);
        break;
      case STATE_TRANSITION_ON:
        lightingState = STATE_HOLD_ON;
        stateTicksTotal = firstRun ? PREVIEW_TRANSITION : animatedPatternCurrent->getTicksOn() * ((lengthMultiplier*2) + 1);
        break;
      case STATE_HOLD_ON:
        lightingState = STATE_TRANSITION_OFF;
        stateTicksTotal = firstRun ? PREVIEW_ON : animatedPatternCurrent->getTicksAnimate() * ((lengthMultiplier*2) + 1);
        break;
      case STATE_TRANSITION_OFF:
        lightingState = STATE_HOLD_OFF;
        stateTicksTotal = firstRun ? PREVIEW_TRANSITION : animatedPatternCurrent->getTicksOff() * ((lengthMultiplier*2) + 1);
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
  analogWrite(BRIGHTNESS_PIN, brightness); // Invert the brightness value

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
