// -*- mode: c; -*-
// Software for driving strip of LED lights.
// see: https://www.adafruit.com/products/285

#include <stdint.h>
#include "ffft.h"

#define MIC_PIN 0

#define BUTTON_PIN (2)
#define REDPIN 5
#define GREENPIN 3
#define BLUEPIN 6

// make this higher to slow down
#define FADESPEED 5

#define DEBUG 1
#define DEBUG_SPEKTRUM (1)

enum light_mode {
  LIGHT_MODE_NONE = 0,
  LIGHT_MODE_SWIRL,
  LIGHT_MODE_MUSIC
};

#define LIGHT_COLOR_RED (0)
#define LIGHT_COLOR_YELLOW (1)
#define LIGHT_COLOR_GREEN (2)
#define LIGHT_COLOR_ZOINKS (3)
#define LIGHT_COLOR_BLUE (4)
#define LIGHT_COLOR_PURPLE (5)
#define LIGHT_COLOR_NMAX (6)

static int color = LIGHT_COLOR_RED;
static enum light_mode mode = LIGHT_MODE_MUSIC;

static int buttonState = 0;
static bool buttonChanged = false;

#define FFT_OUT_LEN (FFT_N/2)
#define THRESHOLD_MIN (15)
#define THRESHOLD_FADE_RATE (3)
#define THRESHOLD_INDEX (2)

int16_t capture[FFT_N];
complex_t bfly_buff[FFT_N];
uint16_t spektrum[FFT_OUT_LEN];
uint16_t threshold[FFT_OUT_LEN];
bool above_threshold = 0;
static int threshold_min = 0;

volatile  byte  position = 0;

void setup() {
    memset(threshold, 0, sizeof(threshold));

    // force device to use reference external analog interface or 3V for our microphone.
    analogReference(EXTERNAL);

    // set up color pins
    pinMode(REDPIN, OUTPUT);
    pinMode(GREENPIN, OUTPUT);
    pinMode(BLUEPIN, OUTPUT);

    pinMode(BUTTON_PIN, INPUT);

#ifdef DEBUG
    // set up serial for debugging
    Serial.begin(57600);
    Serial.println("Starting.");
#endif

    for (int i = 0; i < FFT_OUT_LEN; i++) {
        threshold[i] = THRESHOLD_MIN;
    }
}

void swirl() {
    int r, g, b;

#ifdef DEBUG
    Serial.println("Starting swirl().");
#endif

    // fade from blue to violet
    for (r = 0; r < 256; r++) {
        analogWrite(REDPIN, r);
        delay(FADESPEED);
        buttonState = digitalRead(BUTTON_PIN);
        change_mode(buttonState);
    }

    // fade from violet to red
    for (b = 255; b > 0; b--) {
        analogWrite(BLUEPIN, b);
        delay(FADESPEED);
        buttonState = digitalRead(BUTTON_PIN);
        change_mode(buttonState);
    }

    // fade from red to yellow
    for (g = 0; g < 256; g++) {
        analogWrite(GREENPIN, g);
        delay(FADESPEED);
        buttonState = digitalRead(BUTTON_PIN);
        change_mode(buttonState);
    }

    // fade from yellow to green
    for (r = 255; r > 0; r--) {
        analogWrite(REDPIN, r);
        delay(FADESPEED);
        buttonState = digitalRead(BUTTON_PIN);
        change_mode(buttonState);
    }

    // fade from green to teal
    for (b = 0; b < 256; b++) {
        analogWrite(BLUEPIN, b);
        delay(FADESPEED);
        buttonState = digitalRead(BUTTON_PIN);
        change_mode(buttonState);
    }

    // fade from teal to blue
    for (g = 255; g > 0; g--) {
        analogWrite(GREENPIN, g);
        delay(FADESPEED);
        buttonState = digitalRead(BUTTON_PIN);
        change_mode(buttonState);
    }
}

void change_mode(int buttonState)
{
    if (buttonChanged == true && buttonState == LOW) {
        buttonChanged = false;
        return;
    } else if (buttonChanged == true && buttonState == HIGH) {
        return;
    }

    if (buttonState == HIGH) {
        buttonChanged = true;
    }

    if (buttonChanged) {
        if (mode == LIGHT_MODE_SWIRL) {
#ifdef DEBUG
            Serial.println("changed mode to music");
#endif
            mode = LIGHT_MODE_MUSIC;
        } else {
#ifdef DEBUG
            Serial.println("changed mode to swirl");
#endif
            mode = LIGHT_MODE_SWIRL;
        }
    }
}

void change_color(int color)
{
    switch (color) {
    case LIGHT_COLOR_RED:
        analogWrite(REDPIN, 255);
        analogWrite(GREENPIN, 0);
        analogWrite(BLUEPIN, 0);
        break;
    case LIGHT_COLOR_YELLOW:
        analogWrite(REDPIN, 255);
        analogWrite(GREENPIN, 255);
        analogWrite(BLUEPIN, 0);
        break;
    case LIGHT_COLOR_GREEN:
        analogWrite(REDPIN, 0);
        analogWrite(GREENPIN, 255);
        analogWrite(BLUEPIN, 0);
        break;
    case LIGHT_COLOR_ZOINKS:
        analogWrite(REDPIN, 0);
        analogWrite(GREENPIN, 255);
        analogWrite(BLUEPIN, 255);
        break;
    case LIGHT_COLOR_BLUE:
        analogWrite(REDPIN, 0);
        analogWrite(GREENPIN, 0);
        analogWrite(BLUEPIN, 255);
        break;
    case LIGHT_COLOR_PURPLE:
        analogWrite(REDPIN, 255);
        analogWrite(GREENPIN, 0);
        analogWrite(BLUEPIN, 255);
        break;
    }
}

static int x = 0;

void music()
{
    // Fade down the thresholding each time through loop
    if ((threshold[THRESHOLD_INDEX] - THRESHOLD_FADE_RATE) > THRESHOLD_MIN) {
        threshold[THRESHOLD_INDEX] -= THRESHOLD_FADE_RATE;
    } else {
        threshold[THRESHOLD_INDEX] = THRESHOLD_MIN;
    }

#ifdef DEBUG
    if (threshold[THRESHOLD_INDEX] < 10) {
        Serial.print("  ");
    } else if (threshold[THRESHOLD_INDEX] < 100) {
        Serial.print(" ");
    }
    Serial.print(threshold[THRESHOLD_INDEX]);
    Serial.print(" ");
#endif

    while (position < FFT_N) {
        capture[position] = analogRead(MIC_PIN);
        ++position;
    }

    if (position == FFT_N) {
        fft_input(capture, bfly_buff);
        fft_execute(bfly_buff);
        fft_output(bfly_buff, spektrum);
    }

    int crossed_threshold = false;
    // Update thresholding

    if ((spektrum[THRESHOLD_INDEX] * 0.7) >= threshold[THRESHOLD_INDEX]) {
      crossed_threshold = true;
      if (!above_threshold) {
        above_threshold = true;
#ifdef DEBUG
        Serial.println("BLINK");
#endif
        ++color;
        if (color >= LIGHT_COLOR_NMAX) {
          color = LIGHT_COLOR_RED;
        }
        change_color(color);
      }
    }
    if (!crossed_threshold) {
      above_threshold = false;
    }

    if (threshold[THRESHOLD_INDEX] < spektrum[THRESHOLD_INDEX]) {
      threshold[THRESHOLD_INDEX] = spektrum[THRESHOLD_INDEX];
    }

#ifdef DEBUG
#ifdef DEBUG_SPEKTRUM
    for (byte i = 0; i < 40; i++){
      if (spektrum[i] < 10) {
        Serial.print("  ");
      } else if (spektrum[i] < 100) {
        Serial.print(" ");
      }
      Serial.print(spektrum[i]);
      Serial.print(" ");
    }
    Serial.println("");
#endif
#endif
    position = 0;
}


void loop()
{
    buttonState = digitalRead(BUTTON_PIN);
    change_mode(buttonState);

    switch (mode) {
    case LIGHT_MODE_NONE:
        break;
    case LIGHT_MODE_SWIRL:
        swirl();
        break;
    case LIGHT_MODE_MUSIC:
        music();
        break;
    }
}
