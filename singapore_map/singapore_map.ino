// Uncomment this line to set the pins to arduino uno
#define ARDUINO_DEBUG_MODE

#ifdef ARDUINO_DEBUG_MODE
  // Arduino Uno Pins
  #define SR_LATCH_PIN 4 // Pin connected to ST_CP of 74HC595
  #define SR_CLOCK_PIN 5 // Pin connected to SH_CP of 74HC595
  #define SR_DATA_PIN 6 // Pin connected to DS of 74HC595
  #define BUTTON_PIN A0 // Analogue Pin
  #define BRIGHTNESS_PIN 3 // Sink pin
  #define print(x) Serial.println(x)
#else
  // ATTINY Pins
  #define SR_LATCH_PIN 3 // Pin connected to ST_CP of 74HC595
  #define SR_CLOCK_PIN 4 // Pin connected to SH_CP of 74HC595
  #define SR_DATA_PIN 1 // Pin connected to DS of 74HC595
  #define BUTTON_PIN A2 // Analogue Pin (A2)
  #define BRIGHTNESS_PIN 0 // Sink pin
  #define print(x) (void(0)) // Disable prints on the ATTINY
#endif

#define BTN_MODE 0
#define BTN_LIGHT 1

// Button states for MODE, BRIGHTNESS
bool buttonStatesPrev [2] = {false, false};
bool buttonStates [2] = {false, false};

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
  if (input == 0) {
    buttonStates[BTN_LIGHT] = buttonStates[BTN_MODE] = 0;
  } else {
    buttonStates[BTN_MODE] = input > 750;
    buttonStates[BTN_LIGHT] = !buttonStates[BTN_MODE]
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
}

void loop() {
  buttonStatePreLoop();

  if (buttonRelease(BTN_LIGHT)) {
    
  }

  brightness++;
  if (brightness > 512) {
    brightness = 0;
  }
  int actualBrightness = brightness;
  if (brightness > 256) {
    actualBrightness = 256 - (brightness - 255);
  }
  analogWrite(BRIGHTNESS_PIN, actualBrightness);
  delay(5);
  print(analogRead(BUTTON_PIN));
  // print(brightness);

  buttonStatePostLoop();
}
