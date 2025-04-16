#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "SimpleMIDI.h"
#include "SoftwareSerial.h"
#include "ModeHandler.h"
#include "table.h"

#define GATE_PIN PIN_PA7
#define LOGGER_PIN_TX PIN_PB4
#define PIN_CV1 PIN_PA1
#define PIN_CV2 PIN_PA2
#define TOGGLE_PIN PIN_PA4
#define TOGGLE_LED PIN_PA5

#define TIMER_FREQ 10000000 // 10MHz (20MHz / 2)
#define MAX_FREQ 1000 // Maximum frequency in Hz
#define MIN_FREQ 20  // Minimum frequency in Hz
#define FOLD_MIN 128 // The max value to fold - half the full range

// CV control parameters
#define CV_THRESHOLD 5 // Threshold for CV value changes (0-1023)

// Mode button debounce time
#define DEBOUNCE_TIME 300

SimpleMIDI MIDI;
// TODO: Change this to real serial port. (We need to update the schematic for that)
SoftwareSerial logger(-1, LOGGER_PIN_TX);
ModeHandler modes = ModeHandler(TOGGLE_PIN, 2, 200);

volatile uint16_t step = 0;
volatile uint16_t step_size = 1;

// Variables for main sawtooth wave generation (TCB0)
volatile uint16_t currentFrequency = 440;
volatile uint16_t pendingFrequency = 0; // New frequency to be applied at the next cycle
volatile bool frequencyChangeRequested = false; // Flag for frequency change

volatile uint8_t fold_level = 255;
uint8_t *table = saw_table;
// Variables for CV control
uint16_t lastCVValue = 0;
uint16_t lastCV2Value = 0;

// Mode control variables
bool midControlMode = false; // true = MIDI control, false = CV1 control

// Function to update the main wave frequency
void setFrequency(uint16_t frequency) {
    // Ensure frequency is within reasonable bounds (MIN_FREQ to MAX_FREQ Hz)
    if (frequency < MIN_FREQ) frequency = MIN_FREQ;
    if (frequency > MAX_FREQ) frequency = MAX_FREQ;

    // Store the current frequency
    currentFrequency = frequency;

    const uint16_t min_timer = 32; // Determined experimentally, going lower than this
    // has no effect. Presumably because the CPU has to do other things than just
    // triggering this interrupt every cycle.
    
    // Simple direct calculation for timer period
    // One complete cycle needs 256 steps (just the ramp up)
    // At frequency F, we need F * 256 steps per second
    // With timer running at TIMER_FREQ, each step needs TIMER_FREQ / (F * 256) ticks
    uint16_t f = frequency;
    uint16_t timerPeriod = 0; //TIMER_FREQ / (f * 1024UL);
    step_size = 0;
    while (timerPeriod < 64) {
      timerPeriod = (step_size * TIMER_FREQ) / (1024UL * frequency);
      step_size += 1;
    }
    // We'll just update the CCMP value without disabling/enabling the timer
    // This reduces potential timing glitches
    // 234Hz signal with CCMP = 1
    //step_size = 2;
    TCB0.CCMP = (uint16_t)timerPeriod;
}

// Update frequency based on CV1 input
void updateFrequencyFromCV() {
    // Only update if in CV control mode
    if (midControlMode) return;
    
    // Read CV1 (0-1023)
    uint16_t rawCV = analogRead(PIN_CV1);
    
    // Only update if the CV value has changed beyond the threshold
    if (abs((int)rawCV - (int)lastCVValue) > CV_THRESHOLD) {
        // Save the current CV value
        lastCVValue = rawCV;
        
        // Map the CV value (0-1023) to frequency range (MIN_FREQ to MAX_FREQ Hz)
        uint16_t newFrequency = map(rawCV, 0, 1023, MIN_FREQ, MAX_FREQ);
        
        // We're in the main loop, so use atomic operations to update volatile variables
        // This prevents race conditions with the ISR
        noInterrupts();
        
        // Set the pending frequency to be applied at the next cycle
        pendingFrequency = newFrequency;
        frequencyChangeRequested = true;
        
        interrupts();
    }
}

// Update volume of octave-down sawtooth based on CV2 input
void updateFoldFromCV2() {    
    // Read CV2 (0-1023)
    uint16_t rawCV2 = analogRead(PIN_CV2);
    
    // Only update if the CV2 value has changed beyond the threshold
    if (abs((int)rawCV2 - (int)lastCV2Value) > CV_THRESHOLD) {
        // Save the current CV2 value
        lastCV2Value = rawCV2;
        fold_level = map(rawCV2, 0, 1023, FOLD_MIN, 255);
    }
}

void setupTimers() {
    // Configure TCB0 for main sawtooth wave generation
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc; // Clock div by 2 (10MHz)
    TCB0.CTRLB = TCB_CNTMODE_INT_gc;    // Timer interrupt mode
    TCB0.INTCTRL = TCB_CAPT_bm;         // Enable capture interrupt

    setFrequency(MIN_FREQ);
    step = 0;
    
    // Enable the timers
    TCB0.CTRLA |= TCB_ENABLE_bm;
    
    sei(); // Enable global interrupts
}

// TCB0 Interrupt Service Routine (Main sawtooth wave)
ISR(TCB0_INT_vect) {
    // First, clear the interrupt flag immediately to reduce jitter
    TCB0.INTFLAGS = TCB_CAPT_bm;

    bool step_x_zero = step > ((step + step_size) % 1024);
    // At the reset point is the best time to change frequency
    // This happens when we're at step 0
    if (step_x_zero == 0 && frequencyChangeRequested) {
    //if(frequencyChangeRequested) {
        if (pendingFrequency > 0) {
            // Apply the new frequency
            setFrequency(pendingFrequency);
            // Reset the pending frequency and flag
            pendingFrequency = 0;
            frequencyChangeRequested = false;
        }
    }
    step += step_size;
    step = step % 1024;
    
    // 1Hz wave needs 1024Hz sampling rate with 1024 samples
    // 1024 samples per second = 1 sample every 1ms
    // 1000Hz wave = 1 sample ever 1us
    // frequency * nsamples = samples / sec
    // time = frequency * 1/nsamples
    // Update DAC directly from the ISR for consistent timing
    if (table[step] > fold_level)
      DAC0.DATA = fold_level - (table[step] - fold_level);// + sin_table[step/2]) / 2; //combinedValue;
    else
      DAC0.DATA = table[step];
    //DAC0.DATA = (uint8_t)(((step * lastCVValue) / 256) % 256); //sin_table[step]; //combinedValue;
    //DAC0.DATA = sin_table[((step * lastCVValue) / 256) % 1024]; //sin_table[step]; //combinedValue;
}

// Callback functions for MIDI events
void onNoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {
  digitalWrite(GATE_PIN, HIGH);
  
  // Only update frequency if in MIDI control mode
  if (midControlMode) {
    // Set frequency based on MIDI note using SimpleMIDI's midiToFrequency function
    float rawFrequency = MIDI.midiToFrequency(note);
    uint16_t frequency = (uint16_t)rawFrequency;
    
    // If the frequency is out of range, don't update the frequency
    if (frequency > MAX_FREQ || frequency < MIN_FREQ) {
      return;
    }
    
    // Set the pending frequency to be applied at the next cycle
    noInterrupts();
    pendingFrequency = frequency;
    frequencyChangeRequested = true;
    interrupts();
  }
}

void onNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) {
  digitalWrite(GATE_PIN, LOW);
  // We don't reset frequency or volume on note off to allow for legato playing
}

void onControlChange(uint8_t channel, uint8_t control, uint8_t value) {  

}

void activateMode() {
  midControlMode = modes.getMode() == 1;
  digitalWrite(TOGGLE_LED, midControlMode ? HIGH : LOW);
}


void setup() {
  // define the gate pin
  pinMode(GATE_PIN, OUTPUT);
  digitalWrite(GATE_PIN, LOW);

  // Setup Logger
  pinMode(LOGGER_PIN_TX, OUTPUT);
  logger.begin(9600);
  
  // Set the analog reference for ADC with supply voltage.
  analogReference(VDD);
  pinMode(PIN_CV1, INPUT);
  pinMode(PIN_CV2, INPUT);
 
  // DAC0 setup for sending velocity via the PA6 pin
  VREF.CTRLA |= VREF_DAC0REFSEL_4V34_gc; //this will force it to use VDD as the VREF
  VREF.CTRLB |= VREF_DAC0REFEN_bm;
  DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm;
  DAC0.DATA = 0;

  // Setup MIDI
  MIDI.begin(31250);

  // Timer related - this will start both sawtooth wave generators
  setupTimers();
  
  // Setup mode toggle button and LED
  pinMode(TOGGLE_PIN, INPUT_PULLUP);
  pinMode(TOGGLE_LED, OUTPUT);

  modes.begin();
  activateMode();

  // Register MIDI callbacks
  MIDI.setNoteOnCallback(onNoteOn);
  MIDI.setNoteOffCallback(onNoteOff);
  MIDI.setControlChangeCallback(onControlChange);

}

void loop() {
  // Process MIDI messages
  MIDI.update();
  
  // Check and update mode toggle
  if (modes.update()) {
    activateMode();
  }
  
  // Update frequency based on CV1 (only if in CV control mode)
  updateFrequencyFromCV();
  
  // Update volume based on CV2 (always active regardless of mode)
  updateFoldFromCV2();
}

