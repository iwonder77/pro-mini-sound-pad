/*
************************************************
* File: pro-mini-touch-pad.ino
* Project: testing the CapacitiveSensor library with Sparkfun's Arduino Pro Mini
*
* Description:
*
* Author: Isai Sanchez 
* (Based on CapitiveSense Library Demo Sketch by Paul Badger 2008)
* Created: 2025-11-7 
*
* Libraries: 
*   - CapacitiveSensor.h: https://github.com/PaulStoffregen/CapacitiveSensor/tree/master
*
* Notes: 
*   - Capacitive touch pad setup uses a high value resistor (1M) between send pin (11) and receive pin (10)
*   - Resistor effects sensitivity, experiment with values, 50K - 50M. Larger resistor values yield larger sensor values.
*   - Receive pin is the sensor pin - try different amounts of foil/metal on this pin
************************************************
*/

#include <CapacitiveSensor.h>
#include "RingWindow.h"

#define SEND_CS_PIN 11
#define RECEIVE_CS_PIN 10
#define BAUD_RATE 9600
#define READ_INTERVAL_MS 40               // sampling period (ms)
#define BASELINE_UPDATE_INTERVAL_MS 1000  // how often to update baseline (ms)
#define CS_SAMPLES 30                     // num samples for CapacitiveSensor library
#define BASELINE_READING_WINDOW_SIZE 50   // num samples for RingWindow library
#define SEND_OUT_PIN 9

// === OBJECTS ===
CapacitiveSensor cs = CapacitiveSensor(SEND_CS_PIN, RECEIVE_CS_PIN);
RingWindow<float> baselineWindow(BASELINE_READING_WINDOW_SIZE);

// === TOUCH PAD STATE ===
enum TouchPadState { IDLE,
                     TOUCHED,
                     RELEASED };
TouchPadState state = IDLE;

// === THRESHOLD VARIABLES ===
const float TOUCH_OFFSET = 10;
const float RELEASE_OFFSET = 5;
const float BASELINE_UPDATE_TOLERANCE = 0.15f;  // only update baseline if delta is this % of baseline
float baseline = 0;
float touchThreshold = 0;
float releaseThreshold = 0;

// === TIMING VARIABLES ===
uint32_t lastReadTime = 0;
uint32_t lastBaselineUpdate = 0;

// === DEBOUNCE ===
const uint8_t DEBOUNCE_COUNT = 3;  // consecutive readings required
uint8_t touchCounter = 0;          // counts consecutive "above threshold" readings
uint8_t releaseCounter = 0;        // counts consecutive "below threshold" readings

// === EMA variables ===
float emaFilteredTouch = 0;
const float ALPHA = 0.4f;

// === PULSE CONTROL ===
bool pulseActive = false;
uint32_t pulseStartTime = 0;
const uint16_t PULSE_DURATION_MS = 100;

void managePulse() {
  if (pulseActive) {
    if (millis() - pulseStartTime >= PULSE_DURATION_MS) {
      digitalWrite(SEND_OUT_PIN, LOW);
      pulseActive = false;
    }
  }
}

void updateStateMachine() {
  switch (state) {
    case IDLE:
      // check if touch reading passed threshold
      if (emaFilteredTouch > touchThreshold) {
        // update touch counter
        touchCounter++;
        if (touchCounter >= DEBOUNCE_COUNT) {
          // if sufficient touch events counted then we can safely change the state to TOUCHED
          state = TOUCHED;
          touchCounter = 0;
          // begin non-blocking pulse
          digitalWrite(SEND_OUT_PIN, HIGH);
          pulseActive = true;
          pulseStartTime = millis();
        }
      } else {
        // important to reset touch counter here
        touchCounter = 0;
      }
      break;
    case TOUCHED:
      // currently touched, waiting for release which is when touch readings fall below threshold
      if (emaFilteredTouch < releaseThreshold) {
        // update release counter
        releaseCounter++;
        if (releaseCounter >= DEBOUNCE_COUNT) {
          // if sufficient release events deteced then we can safely change the state back to IDLE
          state = IDLE;
          releaseCounter = 0;
          // release action triggered here
        }
      } else {
        // reset release counter here
        releaseCounter = 0;
      }
      break;
    case RELEASED:
      // optional state
      // can use for multi-touch logic or something like that
      break;
  }
}

void setup() {
  cs.set_CS_AutocaL_Millis(0xFFFFFFFF);  // turn off autocalibrate on channel 1 - just as an example
  Serial.begin(BAUD_RATE);
  delay(2000);

  pinMode(SEND_OUT_PIN, OUTPUT);
  digitalWrite(SEND_OUT_PIN, LOW);

  Serial.println("--- Cap Touch Button w/ Pro Mini ---");
  Serial.println("Taking Baseline Readings - DO NOT TOUCH!");
  for (int i = 0; i < BASELINE_READING_WINDOW_SIZE; i++) {
    long sample = cs.capacitiveSensor(CS_SAMPLES);
    if (sample < 0) {
      i--;
      continue;
    }
    baselineWindow.add((float)sample);
    delay(10);
  }

  // initialize baseline with window AVERAGE
  baseline = baselineWindow.average();
  touchThreshold = baseline + TOUCH_OFFSET;
  releaseThreshold = baseline + RELEASE_OFFSET;
  emaFilteredTouch = baseline;

  Serial.print("Baseline value: ");
  Serial.println(baseline);
  Serial.println("Raw\tEMA\tBase\tDelta\tState");
  Serial.println("---\t---\t----\t-----\t-----");
}

void loop() {
  uint32_t now = millis();
  if (now - lastReadTime < READ_INTERVAL_MS) return;
  lastReadTime = now;

  long rawResult = cs.capacitiveSensor(CS_SAMPLES);
  if (rawResult < 0) return;

  emaFilteredTouch = ALPHA * rawResult + (1 - ALPHA) * emaFilteredTouch;

  // calculate difference between touch readings and baseline
  long delta = emaFilteredTouch - baseline;

  // --- periodic baseline update ---
  // only update baseline when it is NOT being touched (delta is small)
  if (now - lastBaselineUpdate >= BASELINE_UPDATE_INTERVAL_MS) {
    if (abs(delta) < baseline * BASELINE_UPDATE_TOLERANCE) {
      baselineWindow.add(emaFilteredTouch);
      baseline = baselineWindow.median();
      touchThreshold = baseline + TOUCH_OFFSET;
      releaseThreshold = baseline + RELEASE_OFFSET;
    }
    lastBaselineUpdate = now;
  }

  managePulse();
  updateStateMachine();

  // --- output to Serial Plotter ---
  Serial.print(rawResult);
  Serial.print("\t");
  Serial.print(emaFilteredTouch, 1);  // 1 decimal place
  Serial.print("\t");
  Serial.print(baseline);
  Serial.print("\t");
  Serial.print(delta);
  Serial.print("\t");

  switch (state) {
    case IDLE: Serial.print("IDLE"); break;
    case TOUCHED: Serial.print("TOUCHED"); break;
    case RELEASED: Serial.print("RELEASED"); break;
  }

  Serial.println();
}
