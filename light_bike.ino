/* -*- mode: c; -*- */
/**
 * @file light_bike.ino
 * @verbatim
 *========================================================================
 * Copyright (C) 2012 Brandon Edens All Rights Reserved
 *========================================================================
 *
 *
 * light_bike is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * light_bike is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with light_bike. If not, see <http://www.gnu.org/licenses/>.
 *
 * Project: light_bike
 * Date: 2012-07-16
 * Author: Brandon Edens <brandonedens@gmail.com>
 *
 * Description:
 * Software for driving strip of analog LED lights based upon microphone input.
 * see: https://www.adafruit.com/products/285
 *
 * @endverbatim
 */


/******************************************************************************
 * includes
 */

#include <stdbool.h>
#include <stdint.h>
#include "ffft.h"


/******************************************************************************
 * local defines
 */

/** Define the pin used for microphone input. */
#define MIC_PIN 0

/** Button that defines a pin that can be used for input. */
#define BUTTON_PIN (2)

/** PWM pin for RED */
#define REDPIN 5

/** PWM pin for GREEN */
#define GREENPIN 3

/** PWM pin for BLUE */
#define BLUEPIN 6

// make this higher to slow down
#define FADESPEED 5

/** Define this to print debug output over serial. */
#define DEBUG

/** Define this to print capture buffer. */
#define xDEBUG_CAPTURE

/** Define this to print debug output of the spectrum data. */
#define DEBUG_SPEKTRUM

/**
 * The proportion of threshold decrease per a sampling. aka this value
 * multiplied by the current threshold level is how much will be subtracted off
 * the threshold.
 */
#define THRESHOLD_DECREASE_RATE (0.05)

// Definitions of various light colors we might use.
#define LIGHT_COLOR_RED (0)
#define LIGHT_COLOR_YELLOW (1)
#define LIGHT_COLOR_GREEN (2)
#define LIGHT_COLOR_CYAN (3)
#define LIGHT_COLOR_BLUE (4)
#define LIGHT_COLOR_PURPLE (5)
#define LIGHT_COLOR_NMAX (6)

/** Macro to increment a variable by a given rate limited to the given bounds (limit). */
#define INC_BOUNDS(x, rate, limit) if ((((int) x) + rate) < limit) {x += rate;} else {x = limit;}

/** Macro to decrement a variable by a given rate limited to the given bounds (limit). */
#define DEC_BOUNDS(x, rate, limit) if ((((int) x) - rate) > limit) {--x;} else {x = limit;}


/******************************************************************************
 * local variables
 */

/** The current color of the lights. */
static int color = LIGHT_COLOR_RED;

/** A target next color for the lights. */
static int next_color = LIGHT_COLOR_YELLOW;

/** The actual RGB color. */
static uint8_t colors[3];

/** Length of the resulting array post FFT. */
#define FFT_OUT_LEN (FFT_N/2)

/** Minimum threshold levl. */
#define THRESHOLD_MIN (100)

/** The index into the array of FFT data that we will monitor. */
#define THRESHOLD_INDEX (2)

/** The rate by which we automatically change from one color to another. */
#define SWIRL_RATE (15)

/** An array of captured data. */
int16_t capture[FFT_N];

/** Internally used by the FFT library. */
complex_t bfly_buff[FFT_N];

/** The array of spectrum data after performing the FFT. */
uint16_t spektrum[FFT_OUT_LEN];

/** An array of threshold values. Currently we only inspect a single threshold
 * value so this should not be a complete array. However, in the ideal world
 * we'd detect thresholding of bass values and treble values distinctly which
 * would mean we'd light up one color for bass and another color for highs.
 */
uint16_t threshold[FFT_OUT_LEN];

/** The rate at which we're decrementing the threshold. */
uint16_t threshold_fade_rate;

/** Flag that indicates we're now above the threshold. */
bool above_threshold = 0;

/** Current position into the capture buffer. */
volatile byte position = 0;


/******************************************************************************
 * local functions
 */

static void colors_to_goal(int target_color, uint8_t colors_ptr[3]);
static int calc_next_color(int color);
static bool colors_reached_goal(int target_color, uint8_t colors[3]);


/*****************************************************************************/

void setup() {
    // By default force the fade rate to 1.
    threshold_fade_rate = 1;
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

void music()
{
    // Fade down the thresholding each time through loop
    if ((threshold[THRESHOLD_INDEX] - threshold_fade_rate) >= THRESHOLD_MIN) {
        threshold[THRESHOLD_INDEX] -= threshold_fade_rate;
    } else if (threshold[THRESHOLD_INDEX] > 55000) {
        // XXX this is a hacked bugfix - fix it by switching to signed numbers
        // the basic problem is that right now its possible for us to
        // accidentally subtract from the threshold to a point where we wrap
        // around to a super high number.
        threshold[THRESHOLD_INDEX] = THRESHOLD_MIN;
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
        // read in microphone input into the capture buffer.
        capture[position] = analogRead(MIC_PIN);
        ++position;
    }

    if (position == FFT_N) {
        // Run the fft algorithm.
        fft_input(capture, bfly_buff);
        fft_execute(bfly_buff);
        fft_output(bfly_buff, spektrum);
    }


    // Update thresholding
    // by default we haven't crossed the threshold.
    int crossed_threshold = false;

    // WARNING - hard coded value. We consider having crossed threshold if
    // we're 70% of what the previous threshold was.
    if ((spektrum[THRESHOLD_INDEX] * 0.7) >= threshold[THRESHOLD_INDEX]) {
        // now crossed threshold
        crossed_threshold = true;
        if (!above_threshold) {
            // if we weren't above threshold then we are now.
            above_threshold = true;
#ifdef DEBUG
            Serial.println("BLINK");
#endif

            // the color is now the next color of what the current color
            // is. This could probably be set to some random color to equal
            // effect (less boring).
            color = calc_next_color(color);
            // the next color is the color we'll fade into.
            next_color = calc_next_color(color);

            // Given the color; set the array of colors that we'll feed to the
            // analog strip to their respective values.
            switch (color) {
            case LIGHT_COLOR_RED:
                colors[0] = 255;
                colors[1] = 0;
                colors[2] = 0;
                break;
            case LIGHT_COLOR_YELLOW:
                colors[0] = 255;
                colors[1] = 255;
                colors[2] = 0;
                break;
            case LIGHT_COLOR_GREEN:
                colors[0] = 0;
                colors[1] = 255;
                colors[2] = 0;
                break;
            case LIGHT_COLOR_CYAN:
                colors[0] = 0;
                colors[1] = 255;
                colors[2] = 255;
                break;
            case LIGHT_COLOR_BLUE:
                colors[0] = 0;
                colors[1] = 0;
                colors[2] = 255;
                break;
            case LIGHT_COLOR_PURPLE:
                colors[0] = 255;
                colors[1] = 0;
                colors[2] = 255;
                break;
            }
        }
    }

    analogWrite(REDPIN, colors[0]);
    analogWrite(GREENPIN, colors[1]);
    analogWrite(BLUEPIN, colors[2]);

    // If we've slowly shifted our current color to the next color then we
    // update our current color to be the next color and then move our colors
    // further to the goal again.
    if (colors_reached_goal(next_color, colors)) {
        color = next_color;
        next_color = calc_next_color(color);
    }
    colors_to_goal(next_color, colors);

    // If we did not cross the threshold this time through the loop then we set
    // our above threshold to false (resetting the ability to blink)
    if (!crossed_threshold) {
        above_threshold = false;
    }

    // update the threshold to the current spectrum value if this is a new high
    // water mark.
    if (threshold[THRESHOLD_INDEX] < spektrum[THRESHOLD_INDEX]) {
        threshold[THRESHOLD_INDEX] = spektrum[THRESHOLD_INDEX];
    }

    // update the threshold fade rate which is the rate at which the threshold
    // is decremented each time through the loop.
    int new_threshold_fade_rate =
        (int) threshold[THRESHOLD_INDEX] * THRESHOLD_DECREASE_RATE;
    if (new_threshold_fade_rate > threshold_fade_rate) {
        threshold_fade_rate = new_threshold_fade_rate;
    } else if (threshold_fade_rate > 1) {
        --threshold_fade_rate;
    } else {
        threshold_fade_rate = 1;
    }

#ifdef DEBUG
    if (threshold_fade_rate < 10) {
        Serial.print("  ");
    } else if (threshold_fade_rate < 100) {
        Serial.print(" ");
    }
    Serial.print(threshold_fade_rate);
    Serial.print(" ");
#endif

#ifdef DEBUG
#ifdef DEBUG_CAPTURE
    for (byte i = 0; i < 30; i++) {
        if (capture[i] < 10) {
            Serial.print("   ");
        } else if (capture[i] < 100) {
            Serial.print("  ");
        } else if (capture[i] < 1000) {
            Serial.print(" ");
        }
        Serial.print(capture[i]);
        Serial.print(" ");
    }
    Serial.println("");
#endif
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
    music();
}

/**
 * Update colors to the given goal.
 */
static void
colors_to_goal(int target_color, uint8_t colors[3])
{
    switch (target_color) {
    case LIGHT_COLOR_RED:
        INC_BOUNDS(colors[0], SWIRL_RATE, 255);
        DEC_BOUNDS(colors[1], SWIRL_RATE, 0);
        DEC_BOUNDS(colors[2], SWIRL_RATE, 0)
            break;
    case LIGHT_COLOR_YELLOW:
        INC_BOUNDS(colors[0], SWIRL_RATE, 255);
        INC_BOUNDS(colors[1], SWIRL_RATE, 255);
        DEC_BOUNDS(colors[2], SWIRL_RATE, 0);
        break;
    case LIGHT_COLOR_GREEN:
        DEC_BOUNDS(colors[0], SWIRL_RATE, 0);
        INC_BOUNDS(colors[1], SWIRL_RATE, 255);
        DEC_BOUNDS(colors[2], SWIRL_RATE, 0);
        break;
    case LIGHT_COLOR_CYAN:
        DEC_BOUNDS(colors[0], SWIRL_RATE, 0);
        INC_BOUNDS(colors[1], SWIRL_RATE, 255);
        INC_BOUNDS(colors[2], SWIRL_RATE, 255);
        break;
    case LIGHT_COLOR_BLUE:
        DEC_BOUNDS(colors[0], SWIRL_RATE, 0);
        DEC_BOUNDS(colors[1], SWIRL_RATE, 0);
        INC_BOUNDS(colors[2], SWIRL_RATE, 255);
        break;
    case LIGHT_COLOR_PURPLE:
        INC_BOUNDS(colors[0], SWIRL_RATE, 255);
        DEC_BOUNDS(colors[1], SWIRL_RATE, 0);
        INC_BOUNDS(colors[2], SWIRL_RATE, 255);
    }
}

/**
 * Returns true if the given colors have reached goal; false otherwise.
 */
static bool
colors_reached_goal(int target_color, uint8_t colors[3])
{
    switch (target_color) {
    case LIGHT_COLOR_RED:
        if (colors[0] == 255 && colors[1] == 0 && colors[2] == 0) {
            return true;
        }
        break;
    case LIGHT_COLOR_YELLOW:
        if (colors[0] == 255 && colors[1] == 255 && colors[2] == 0) {
            return true;
        }
        break;
    case LIGHT_COLOR_GREEN:
        if (colors[0] == 0 && colors[1] == 255 && colors[2] == 0) {
            return true;
        }
        break;
    case LIGHT_COLOR_CYAN:
        if (colors[0] == 0 && colors[1] == 255 && colors[2] == 255) {
            return true;
        }
        break;
    case LIGHT_COLOR_BLUE:
        if (colors[0] == 0 && colors[1] == 0 && colors[2] == 255) {
            return true;
        }
        break;
    case LIGHT_COLOR_PURPLE:
        if (colors[0] == 255 && colors[1] == 0 && colors[2] == 255) {
            return true;
        }
        break;
    }
    return false;
}

/**
 * Given a color; return what the next color would be in its sequence.
 */
static int
calc_next_color(int color)
{
    int next_color = color;
    ++next_color;
    if (next_color >= LIGHT_COLOR_NMAX) {
        next_color = LIGHT_COLOR_RED;
    }
    return next_color;
}
