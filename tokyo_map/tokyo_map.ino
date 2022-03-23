#include <EEPROM.h>

// These can be modified to change the program behaviour
#define MAX_LENGTH_MULTIPLIER 10
#define LONG_PRESS_THRESHOLD 1000
#define PREVIEW_OFF 2
#define PREVIEW_TRANSITION 4
#define PREVIEW_ON 6

// Uncomment this line to set the pins to arduino uno
//#define ARDUINO_UNO_MODE

// #define DEBUG_MODE

#ifdef ARDUINO_UNO_MODE
  // Arduino Uno Pins
  #define SR_LATCH_PIN 5 // Pin connected to ST_CP of 74HC595
  #define SR_CLOCK_PIN 4 // Pin connected to SH_CP of 74HC595h
  #define SR_DATA_PIN 6 // Pin connected to DS of 74HC595
  #define BUTTON1_PIN A0 // Analogue Pin
  #define BUTTON2_PIN A1 // Analogue Pin
  #define BUTTON3_PIN A2 // Analogue Pin
  #define LIGHT_SENSOR_PIN A3 // Analogue Pin
  #define BRIGHTNESS_PIN 3 // Sink pin
  #define puts(x) Serial.print(x)
  #define putsln(x) Serial.println(x)
#else
  // Arduino Pro Micro Pins
  #define SR_LATCH_PIN 7 // Pin connected to ST_CP of 74HC595 (LATCH)
  #define SR_CLOCK_PIN 8 // Pin connected to SH_CP of 74HC595 (CLOCK)
  #define SR_DATA_PIN 6 // Pin connected to DS of 74HC595 (DATA)
  #define BUTTON3_PIN A0 // Analogue Pin
  #define BUTTON2_PIN A1 // Analogue Pin
  #define BUTTON1_PIN A2 // Analogue Pin
  #define LIGHT_SENSOR_PIN A3 // Analogue Pin
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
 *         D2 |      | A3  LIGHT SENSOR
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

#define CALSTATE_OFF 0
#define CALSTATE_START 1
#define CALSTATE_LOW 2
#define CALSTATE_HIGH 3

// Since there are 2 resistors on the buttons,
// we need to use the analog function of the button.
#define BUTTON_OFF_THRESHOLD 60

uint8_t lightingState;
uint16_t stateTicksTotal;
uint16_t stateTicksLeft;

uint8_t currentPatternState;
uint8_t calibrationState = CALSTATE_OFF;


// Button states for SPEED, BRIGHTNESS, MODE
bool buttonStatesPrev [3] = {false, false, false};
bool buttonStates [3] = {false, false, false};
unsigned long longPressMills [] = {0,0,0,0};
bool longPressFired[3] = {false, false, false};

bool firstRun = true;

float brightnessLevels[8] = { 1.0f, 0.7f, 0.5f, 0.35f, 0.2f, 0.1f, 0.05f, 0.025f};
uint16_t randomState = random();

int lowBrightness = 20;
int highBrightness = 500;

int photosensorRunningAverage = 0;

unsigned long millsDelta;
unsigned long prevMills;
unsigned long currentMills;


// Savable variables
uint8_t currentBrightnessLevel = 0;
uint8_t lengthMultiplier = 1;
uint8_t currentPatternId = 0;
uint8_t autoBrightness = 1;
uint8_t minPhotosensor = 5;
uint8_t maxPhotosensor = 128;


// The animation will work in 4 states
// STATE_HOLD_OFF
// STATE_TRANSITION_ON
// STATE_HOLD_ON
// STATE_TRANSITION_OFF

class AnimatedPattern {
  private:
  uint16_t * states; // 16
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
  AnimatedPattern(uint16_t * _states, uint8_t _totalStates, uint8_t _ticksOn, uint8_t _ticksAnimate, uint8_t _ticksOff, AnimatedPattern * _nextPattern) {
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
  uint16_t getState() {
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


// Patterns are defined as integers and all 16 bits are used
// 1 represents on an state, while 0 represents an off state.
// The bits from right to left control the following lines respectively
// Saikyo, Chuo-Sobu, Chuo, Keiyo, Yokohama, Joban, Negishi / Keihin-Tokoku, Musashino,
// Nanbu, Yokosuka / Sobu, Shonan Shinjuku, Tokaido Main Line, Takasaki, Yamanote, Tokyo Monorail, Shinkansen
// For example:
// 0b0000000100000001
// Saikyo and Nanbu will light up
// 0b1100000000000000
// Tokyo Monorail and Shinkansen will light up

// One at a time from the center
uint16_t pattern1[] = {0b0010000000000000, 0b0000000000000100, 0b0000000000000010, 0b0000000000000001, 
                       0b0000000001000000, 0b0000001000000000, 0b0000010000000000, 0b0000100000000000,
                       0b0000000000001000, 0b0000000100000000, 0b0000000010000000, 0b0001000000000000,
                       0b0100000000000000, 0b0000000000100000, 0b0000000000010000, 0b1000000000000000};
// Filling from the center
uint16_t pattern2[] = {0b0010000000000000, 0b0010000000000100, 0b0010000000000110, 0b0010000000000111,
                       0b0010000001000111, 0b0010001001000111, 0b0010011001000111, 0b0010111001000111,
                       0b0010111001001111, 0b0010111101001111, 0b0010111111001111, 0b0011111111001111,
                       0b0111111111001111, 0b0111111111101111, 0b0111111111101111, 0b1111111111111111};

// Fill from time of opening
uint16_t pattern3[] = {0b0000000001000000, 0b0001000001000000, 0b0011000001000000, 0b0011001001100000,
                       0b0011001001110000, 0b0011101001110000, 0b0011101101110000, 0b0011101101110110,
                       0b1111101101110110, 0b1111101111110110, 0b1111101111111110, 0b1111101111111111,
                       0b1111111111111111};
// Staying on
uint16_t pattern4[] = {0b1111111111111111};

#define TOTAL_PATTERNS 6
void setupAnimatedPatterns() {
  
  animatedPatternCurrent = new AnimatedPattern(NULL, 0, 50, 10, 0, NULL); // Randomly generated
  animatedPatternCurrent = new AnimatedPattern(pattern1, 16, 70, 8, 0, animatedPatternCurrent);
  animatedPatternCurrent = new AnimatedPattern(pattern2, 16, 70, 8, 0, animatedPatternCurrent);
  animatedPatternCurrent = new AnimatedPattern(pattern3, 13, 70, 8, 0, animatedPatternCurrent);
  animatedPatternCurrent = new AnimatedPattern(pattern4, 1, 100, 0, 0, animatedPatternCurrent);
  animatedPatternCurrent = new AnimatedPattern(pattern4, 1, 10, 100, 0, animatedPatternCurrent);

  animatedPatternHead = animatedPatternCurrent;
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
  puts("Current Pattern ID: "); putsln(currentPatternId);
  lightingState = STATE_HOLD_OFF;
  currentPatternState = 0;
  firstRun = true;
  stateTicksLeft = stateTicksTotal = animatedPatternCurrent->getTicksOff();
}

bool buttonHold(uint8_t btn) {
  if (longPressMills[btn] > LONG_PRESS_THRESHOLD && !longPressFired[btn]) {
    longPressFired[btn] = true;
    return true;
  }
  return false;
}

bool buttonRelease(uint8_t btn) {
  if (longPressMills[btn] >= LONG_PRESS_THRESHOLD) return false;
  return !buttonStates[btn] && buttonStatesPrev[btn];
}

bool anyButtonRelease() {
  return buttonRelease(BTN_MODE) || buttonRelease(BTN_LIGHT) || buttonRelease(BTN_SPEED);
}

void timerPreLoop() {
  prevMills = currentMills;
  currentMills = millis();
  millsDelta = currentMills - prevMills;
}


void buttonStatePreLoop() {
  for ( int i = 0; i < 3; i++ ) buttonStates[i] = false;
  if (analogRead(BUTTON1_PIN) > BUTTON_OFF_THRESHOLD) buttonStates[BTN_MODE] = true;
  if (analogRead(BUTTON2_PIN) > BUTTON_OFF_THRESHOLD) buttonStates[BTN_LIGHT] = true;
  if (analogRead(BUTTON3_PIN) > BUTTON_OFF_THRESHOLD) buttonStates[BTN_SPEED] = true;
  
  for (int i = 0; i < 3; i++) {
    if (buttonStates[i]) {
      longPressMills[i] += millsDelta;
    } else {
      longPressMills[i] = 0;
      longPressFired[i] = false;
    }
  }
}

void buttonStatePostLoop() {
  for ( int i = 0; i < 3; i++ ) buttonStatesPrev[i] = buttonStates[i];
}

void setAllOn() {
    digitalWrite(SR_LATCH_PIN, LOW);
    shiftOut(SR_DATA_PIN, SR_CLOCK_PIN, MSBFIRST, 0b11111111);
    shiftOut(SR_DATA_PIN, SR_CLOCK_PIN, MSBFIRST, 0b11111111);
    digitalWrite(SR_LATCH_PIN, HIGH);
}
void feedbackOn(uint16_t time) {
    digitalWrite(13, HIGH);
    analogWrite(BRIGHTNESS_PIN, 255);
    delay(time);
}
void feedbackOff(uint16_t time) {
    digitalWrite(13, LOW);
    analogWrite(BRIGHTNESS_PIN, 0);
    delay(time);
}

void blinkDebugLight() {
  #ifdef DEBUG_MODE
    setAllOn();
    feedbackOn(150);
    feedbackOff(80);
  #endif
}

void blinkA() {
    setAllOn();
    feedbackOff(300);
    feedbackOn(250);
    feedbackOff(200);
    feedbackOn(900);
    feedbackOff(300);
}
void blinkM() {
    setAllOn();
    feedbackOff(300);
    feedbackOn(600);
    feedbackOff(200);
    feedbackOn(600);
    feedbackOff(300);
}


void setupPinModes() {
  pinMode(SR_LATCH_PIN, OUTPUT);
  pinMode(SR_CLOCK_PIN, OUTPUT);
  pinMode(SR_DATA_PIN, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(BUTTON3_PIN, INPUT);
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  pinMode(BRIGHTNESS_PIN, OUTPUT);
}
void resetFactorySettings() {
  EEPROM.update(0, 0); // First bit is reset so that the program will reset.
  putsln("Reset to factory settings");
}
void firstTimeSetup() {
  currentBrightnessLevel = 0;
  lengthMultiplier = 0;
  currentPatternId = 0;
  autoBrightness = 1;
  minPhotosensor = 5;
  maxPhotosensor = 128;
  EEPROM.update(1, currentBrightnessLevel);
  EEPROM.update(2, lengthMultiplier);
  EEPROM.update(3, currentPatternId);
  EEPROM.update(4, autoBrightness);
  EEPROM.update(5, minPhotosensor);
  EEPROM.update(6, maxPhotosensor);
  EEPROM.update(0, 128);
  putsln("First time initialisation");
}
void readFromMemory() {
    currentBrightnessLevel = EEPROM.read(1);
    lengthMultiplier = EEPROM.read(2);
    currentPatternId = EEPROM.read(3);
    autoBrightness = EEPROM.read(4);
    minPhotosensor = EEPROM.read(5);
    maxPhotosensor = EEPROM.read(6);

    currentBrightnessLevel = currentBrightnessLevel % 8;
    lengthMultiplier = lengthMultiplier % MAX_LENGTH_MULTIPLIER;
    currentPatternId = currentPatternId % TOTAL_PATTERNS;

    putsln("Loaded from memory");
    puts("Pattern: ");
    putsln(currentPatternId);
    puts("Brightness: ");
    putsln(currentBrightnessLevel);
    puts("Length: ");
    putsln(lengthMultiplier);
    puts("Auto Brightness: ");
    putsln(autoBrightness ? "True" : "False");
    puts("Photosensor calibration min: ");
    putsln(minPhotosensor);
    puts("Photosensor calibration max: ");
    putsln(maxPhotosensor);
}


void normalState() {
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

  if (buttonHold(BTN_LIGHT)) {
    if (buttonStates[BTN_MODE] && buttonStates[BTN_SPEED]) {
      putsln("Calibrate Mode");
      calibrationState = CALSTATE_START;
    } else {
      if (autoBrightness == 1) {
        autoBrightness = 0;
        putsln("Turned off auto brightness");
        blinkM();
      } else {
        autoBrightness = 1;
        putsln("Turned on auto brightness");
        blinkA();
      }
      EEPROM.update(4, autoBrightness);
    }
  }

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
  int lightAnalogValue = analogRead(LIGHT_SENSOR_PIN);

  photosensorRunningAverage = lightAnalogValue * 0.04 + photosensorRunningAverage * 0.96;
  if (autoBrightness) {
    float lightValue = photosensorRunningAverage - minPhotosensor * 4;
    float maxLightValue = maxPhotosensor * 4;
    float brightPercent = lightValue / (maxLightValue - (minPhotosensor * 4));
    if (brightPercent > 1) brightPercent = 1;
    if (brightPercent < 0) brightPercent = 0;
    // putsln(brightPercent);
    brightness = brightness * (brightPercent * 0.975 + 0.025);
  } else {
    brightness = brightness * brightnessLevels[currentBrightnessLevel];
  }
  analogWrite(BRIGHTNESS_PIN, brightness);

  // Set the image state
  digitalWrite(SR_LATCH_PIN, LOW);
  shiftOut(SR_DATA_PIN, SR_CLOCK_PIN, MSBFIRST, highByte(animatedPatternCurrent->getState()));
  shiftOut(SR_DATA_PIN, SR_CLOCK_PIN, MSBFIRST, lowByte(animatedPatternCurrent->getState()));
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
}



void setup() {
  #ifdef DEBUG_MODE
    Serial.begin(9600);
    pinMode(13, OUTPUT);
  #endif
  putsln("Start setup");

  blinkDebugLight();

  setupPinModes();
  randomSeed(analogRead(A3));
  
  buttonStatePreLoop();
  if (buttonStates[BTN_MODE] && buttonStates[BTN_LIGHT] && buttonStates[BTN_SPEED]) {
    resetFactorySettings();
  }

  putsln("Reading from eeprom");
  if (EEPROM.read(0) != 128) {
    firstTimeSetup();
  } else {
    readFromMemory();
  }

  blinkDebugLight();
  
  putsln("Setting up animated patterns");
  setupAnimatedPatterns();
  int gotoPatternId = currentPatternId;
  currentPatternId = 0;
  
  // Need to + TOTAL_PATTERNS because if ID is 0,
  // it will need to go one full cycle, this makes the code simpler
  for (int i = 0; i < gotoPatternId + TOTAL_PATTERNS; i++) {
    nextPattern();
  }
  
  blinkDebugLight();
  putsln("Done setup");
}

void loop() {
  // Calculate buttons
  timerPreLoop();
  buttonStatePreLoop();

  if (calibrationState == CALSTATE_OFF) {
    // putsln("Normal State");
    normalState();
  } else {
    // putsln("Calibration State");
    uint8_t calibrationBrightness = 0;
    photosensorRunningAverage = analogRead(LIGHT_SENSOR_PIN) * 0.3 + photosensorRunningAverage * 0.7;
    uint8_t photosensorSave = photosensorRunningAverage / 4;
    if (calibrationState == CALSTATE_START) {
      if (!buttonStates[BTN_MODE] && !buttonStates[BTN_LIGHT] && !buttonStates[BTN_SPEED])
        calibrationState = CALSTATE_LOW;
    } else if (calibrationState == CALSTATE_LOW) {
      calibrationBrightness = 50;
      if (anyButtonRelease()) {
        EEPROM.update(5, photosensorSave);
        minPhotosensor = photosensorSave;
        calibrationState = CALSTATE_HIGH;
      }
    } else if (calibrationState == CALSTATE_HIGH) {
      calibrationBrightness = 255;
      if (anyButtonRelease()) {
        EEPROM.update(6, photosensorSave);
        maxPhotosensor = photosensorSave;
        calibrationState = CALSTATE_OFF;
      }
    }
    setAllOn();
    if (millis() % 1000 < 500) {
      analogWrite(BRIGHTNESS_PIN, calibrationBrightness);
    } else {
      analogWrite(BRIGHTNESS_PIN, 0);
    }
  }

  buttonStatePostLoop();
  delay(10);
}
