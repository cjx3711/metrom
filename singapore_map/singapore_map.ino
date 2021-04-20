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
int brightness = 0;
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

void loop()
{
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
}
