#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>  // LIS3DH accelerometer.
#include <Adafruit_Sensor.h>  // Needed for LIS3DH accelerometer.
#include <FastLED.h>          // Fastled library for LED control. Better functionality vs. Adafruit's NeoPixel.

#define NUM_LEDS1    12
#define NUM_LEDS2    7
#define BRIGHTNESS1  128
#define BRIGHTNESS2  64
#define LED_TYPE1    WS2811
#define LED_TYPE2    WS2811
#define COLOR_ORDER1 GRB
#define COLOR_ORDER2 GRB
#define LED_PIN1     5
#define LED_PIN2     6
CRGB leds1[NUM_LEDS1];
CRGB leds2[NUM_LEDS2];

CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

#define UPDATES_PER_SECOND 100

#ifdef __AVR__
#include <avr/power.h>
#endif

// Various light effect modes
#define SPARKLE 0
#define BREATHE 1
#define LIGHT 2
#define UV 3
#define FLASHLIGHT 4
#define RANDOM_PULSE 5

#define MAXBREATHRBIGHTNESS 128

// Which pin the discrete UV and white LEDs are connected to
#define CREEPIN 10
#define UVPIN   11

// Acceloremeter setup
// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10

// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// Adjust this number for the sensitivity of the 'click' force
// this strongly depend on the range! for 16G, try 5-10
// for 8G, try 10-20. for 4G try 20-40. for 2G try 40-80
#define CLICKTHRESHHOLD 80

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
#define Serial SerialUSB
#endif

// Global state variables
uint8_t   mode = 0;
uint8_t   lHue = 0;
uint8_t   lSat = 0;
uint8_t   lVal = 0;
int8_t    lValDelta = 1;
int8_t    lHueDelta = 32;
uint32_t  sleeptimer;

// *******
//  SETUP
// *******
void setup(void) {
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  // UV transistor to output
  pinMode(UVPIN, OUTPUT);
  pinMode(CREEPIN, OUTPUT);

  delay( 3000 ); // power-up safety delay
  FastLED.addLeds<LED_TYPE1, LED_PIN1, COLOR_ORDER1>(leds1, NUM_LEDS1).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE2, LED_PIN2, COLOR_ORDER2>(leds2, NUM_LEDS2).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS1);

  currentPalette = RainbowColors_p;
  currentBlending = LINEARBLEND;

  Serial.begin(9600);
  Serial.println("Adafruit LIS3DH Tap Test!");

  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");

  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");

  // 0 = turn off click detection & interrupt
  // 1 = single click only interrupt output
  // 2 = double click only interrupt output, detect single click
  // Adjust threshhold, higher numbers are less sensitive
  lis.setClick(2, CLICKTHRESHHOLD);
  delay(100);
}

// ***********
//  MAIN LOOP
// ***********

void loop() {
  // Start by reading the accelerometer. This'll be used throughout the loop.
  uint8_t click = lis.getClick();
  sensors_event_t event;
  lis.getEvent(&event); // Read the current acceleration.

  //leds1[mode] = CRGB::White;
  //FastLED.show();
  //leds1[mode] = CRGB::Black;

  switch (mode) {
    case SPARKLE:
      for (byte i = 0; i < NUM_LEDS1; i++) {
        if (random(12) == 1)
          leds1[i] = CHSV(lHue, 255, 255);
      }
      for (byte i = 0; i < NUM_LEDS2; i++) {
        if (random(7) == 1)
          leds2[i] = CHSV(lHue, 255, 255);
      }
      lHue++;
      FastLED.show();
      delay(20);
      FastLED.clear();
      break;

    case BREATHE:
      if (sleeptimer > millis()) // If we're still sleeping, lets just exit.
        break;
      for (byte i = 0; i < NUM_LEDS1; i++) {
        leds1[i] = CHSV(lHue, 255, lVal);
      }
      for (byte i = 0; i < NUM_LEDS2; i++) {
        leds2[i] = CHSV(lHue, 255, lVal);
      }
      lVal += lValDelta;
      if (lVal >= MAXBREATHRBIGHTNESS) {
        lValDelta = -1;
      }
      if (lVal == 0 && lValDelta == -1) {
        lValDelta = 1;
        sleeptimer = millis() + 4000; // How long we sleep between breaths.
        // Supposedly this rolls over in 50 days, so we'll just ignore like the bad programmers we are.
      }
      FastLED.show();
      delay(10);          // This controls the "speed" of breathing
      FastLED.clear();
      break;

    case LIGHT: // Tilt the device right for varying brightnesses. Tilt the device left for red light.
      if (abs(event.acceleration.x) * 20 > 255)
        lVal = 255;
      lVal = abs(event.acceleration.x) * 20;

      for (byte i = 0; i < NUM_LEDS1; i++) {
        leds1[i] = CHSV(lHue, 255, lVal);
      }
      for (byte i = 0; i < NUM_LEDS2; i++) {
        leds2[i] = CHSV(lHue, 255, lVal);
      }
      FastLED.show();
      delay(50);
      FastLED.clear();
      break;

    case UV:
      //      if (!digitalRead(UVPIN))  // If it's not already on...
      //        digitalWrite(UVPIN, 1); // ...turn it on.
      if (abs(event.acceleration.x) * 20 > 255)
        lVal = 255;
      lVal = abs(event.acceleration.x) * 20;
      analogWrite(UVPIN, lVal);
      break;

    case FLASHLIGHT:
      if (!digitalRead(CREEPIN))  // If it's not already on...
        digitalWrite(CREEPIN, 1); // ...turn it on.
      break;

    case RANDOM_PULSE:
      if (random(3000) == 69) {
        for (byte i = 0; i < NUM_LEDS1; i++) {
          leds1[i] = CHSV(lHue, 255, 255);
        }
        for (byte i = 0; i < NUM_LEDS2; i++) {
          leds2[i] = CHSV(lHue, 255, 255);
        }
        FastLED.show();
        delay(20);
        FastLED.clear();
        FastLED.show();
      }
      break;

    default:
      break;
  }

  // Finally, let's see if we got tapped.
  // If yes, and we're in a particular orientation, change mode or adjust the submode,
  // if we're in normal orientation, we'll just ignore it again.
  if (click == 0) return;
  if (! (click & 0x30)) return;

  if ((click & 0x30)) {
    Serial.print("Click detected (0x"); Serial.print(click, HEX); Serial.print("): ");
  }

  if (click & 0x10) {
    Serial.print(" single click");
    Serial.println("\n");
  }
  if (click & 0x20) {
    Serial.print(" double click");
    Serial.println("\n");

    delay(100); // Wait a little for everything to settle then read the acceleration again.
    lis.getEvent(&event);
    Serial.print("X: "); Serial.print(event.acceleration.x);
    Serial.print(" \tY: "); Serial.print(event.acceleration.y);
    Serial.print(" \tZ: "); Serial.print(event.acceleration.z);
    Serial.println(" m/s^2 ");
    Serial.println("\n");

    // Let's pick a mode depending on orientation.
    if (event.acceleration.y > 6) {
      mode = SPARKLE;
      // Cleanup, turn everything off.
      digitalWrite(UVPIN, 0);   //  Turn discrete LEDs off.
      digitalWrite(CREEPIN, 0); //  Turn discrete LEDs off.
      FastLED.clear();
      FastLED.show();
    }

    if (event.acceleration.y < -6) {
      if (mode == BREATHE) {
        lHue = lHue - lHue % 32;
        lHue += lHueDelta;        // If we're already in this mode, let's cycle through colors instead.
      } else {
        mode = BREATHE;
        // Cleanup, turn everything off.
        digitalWrite(UVPIN, 0);   //  Turn discrete LEDs off.
        digitalWrite(CREEPIN, 0); //  Turn discrete LEDs off.
        FastLED.clear();
        FastLED.show();
      }
    }

    if (event.acceleration.x > 6) {
      if (mode == LIGHT) {
        lHue = lHue - lHue % 32;
        lHue += lHueDelta;        // If we're already in this mode, let's cycle through colors instead.
      } else {
        mode = LIGHT;
        // Cleanup, turn everything off.
        digitalWrite(UVPIN, 0);   //  Turn discrete LEDs off.
        digitalWrite(CREEPIN, 0); //  Turn discrete LEDs off.
        FastLED.clear();
        FastLED.show();
      }
    }

    if (event.acceleration.x < -6) {
      mode = UV;
      // Cleanup, turn everything off.
      digitalWrite(UVPIN, 0);   //  Turn discrete LEDs off.
      digitalWrite(CREEPIN, 0); //  Turn discrete LEDs off.
      FastLED.clear();
      FastLED.show();
    }

    if (event.acceleration.z > 6) {
      mode = FLASHLIGHT;
      // Cleanup, turn everything off.
      digitalWrite(UVPIN, 0);   //  Turn discrete LEDs off.
      digitalWrite(CREEPIN, 0); //  Turn discrete LEDs off.
      FastLED.clear();
      FastLED.show();
    }

    if (event.acceleration.z < -6) {
      if (mode == RANDOM_PULSE) {
        lHue = lHue - lHue % 32;
        lHue += lHueDelta;        // If we're already in this mode, let's cycle through colors instead.
        // Because finding out the new color would otherwise take a while, let's flash immediately.
        for (byte i = 0; i < NUM_LEDS1; i++) {
          leds1[i] = CHSV(lHue, 255, 255);
        }
        for (byte i = 0; i < NUM_LEDS2; i++) {
          leds2[i] = CHSV(lHue, 255, 255);
        }
        FastLED.show();
        delay(100);
        FastLED.clear();
        FastLED.show();
      } else {
        mode = RANDOM_PULSE;
        // Cleanup, turn everything off.
        digitalWrite(UVPIN, 0);   //  Turn discrete LEDs off.
        digitalWrite(CREEPIN, 0); //  Turn discrete LEDs off.
        FastLED.clear();
        FastLED.show();
      }
    }
  }
  return;
}
