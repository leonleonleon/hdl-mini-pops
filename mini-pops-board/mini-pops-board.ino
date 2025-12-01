// O2 Minipops rhythm box (c) DSP Synthesizers 2016
// Free for non commercial use

// http://janostman.wordpress.com

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "TheWeeO3_data.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Standard Arduino Pins
#define digitalPinToPortReg(P) \
  (((P) >= 0 && (P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define digitalPinToDDRReg(P) \
  (((P) >= 0 && (P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : &DDRC))
#define digitalPinToPINReg(P) \
  (((P) >= 0 && (P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : &PINC))
#define digitalPinToBit(P) \
  (((P) >= 0 && (P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P)-8 : (P)-14))

#define digitalReadFast(P) bitRead(*digitalPinToPINReg(P), digitalPinToBit(P))

#define digitalWriteFast(P, V) bitWrite(*digitalPinToPortReg(P), digitalPinToBit(P), (V))

const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

//--------- Ringbuf parameters ----------
uint8_t Ringbuffer[256];
uint8_t RingWrite = 0;
uint8_t RingRead = 0;
volatile uint8_t RingCount = 0;
volatile uint16_t SFREQ;

volatile uint16_t tempo = 3500; // Global declaration for tempo 
volatile uint8_t stepcnt = 0; 
volatile uint8_t patlength = 0;
//-----------------------------------------

volatile uint16_t samplecntBD = 0;
volatile uint16_t samplecntBG2 = 0;
volatile uint16_t samplecntCL = 0;
volatile uint16_t samplecntCW = 0;
volatile uint16_t samplecntCY = 0;
volatile uint16_t samplecntGU = 0;
volatile uint16_t samplecntMA = 0;
volatile uint16_t samplecntQU = 0;

volatile uint16_t samplepntBD = 0;
volatile uint16_t samplepntBG2 = 0;
volatile uint16_t samplepntCL = 0;
volatile uint16_t samplepntCW = 0;
volatile uint16_t samplepntCY = 0;
volatile uint16_t samplepntGU = 0;
volatile uint16_t samplepntMA = 0;
volatile uint16_t samplepntQU = 0;

volatile uint8_t playing = 1;
volatile uint8_t stepPulse = 0;
volatile uint8_t patselect = 0;

ISR(TIMER1_COMPA_vect) {

  //-------------------  Ringbuffer handler -------------------------

  if (RingCount) {                     //If entry in FIFO..
    OCR2A = Ringbuffer[(RingRead++)];  //Output LSB of 16-bit DAC
    RingCount--;
  }

  //-----------------------------------------------------------------
}
int clockOutPin = 13;

ISR(TIMER0_COMPA_vect) {
    static uint16_t tempocnt = 100; // Initialize with safe default

    if (playing) {
      digitalWriteFast(15, HIGH);  //play state out Hi
      if (!(tempocnt--)) {
        tempocnt = tempo; // Reset tempocnt based on the adjustable tempo value
        
        digitalWriteFast(clockOutPin, stepPulse );  //Clock out Hi
        stepPulse = stepPulse == 1 ? 0 : 1;

        uint8_t trig = pgm_read_byte_near(pattern + (patselect << 4) + stepcnt++);
        PORTC = stepcnt;
        uint8_t mask = (PIND >> 2) | ((PINB & 3) << 6);
        trig &= mask;
        
        if (stepcnt > patlength) stepcnt = 0;
        
        if (trig & 1) {
          samplepntQU = 0;
          samplecntQU = 7712;
        }
        if (trig & 2) {
          samplepntCY = 0;
          samplecntCY = 9434;
        }
        if (trig & 4) {
          samplepntMA = 0;
          samplecntMA = 568;
        }
        if (trig & 8) {
          samplepntCW = 0;
          samplecntCW = 830;
        }
        if (trig & 16) {
          samplepntCL = 0;
          samplecntCL = 752;
        }
        if (trig & 32) {
          samplepntBD = 0;
          samplecntBD = 1076;
        }
        if (trig & 64) {
          samplepntBG2 = 0;
          samplecntBG2 = 1136;
        }
        if (trig & 128) {
          samplepntGU = 0;
          samplecntGU = 2816;
        }
      }
    } else {
      digitalWriteFast(15, LOW);  //play state out Lo
      digitalWriteFast(clockOutPin, 0);  //Clock out Lo
    }
}


void setup() {

  OSCCAL = 0xFF;

  //Drum mute inputs
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);

  pinMode(10, INPUT_PULLUP);  //RUN - Stop input
  pinMode(12, INPUT);     //FS input
  // pinMode(13, OUTPUT);        //Clock output
  // pinMode(13, INPUT_PULLUP);       
  digitalWrite(12, HIGH);  //FS input

  pinMode(clockOutPin, OUTPUT);  // clock output
  pinMode(15, OUTPUT); // play state output

  // pinMode(14, OUTPUT);
  // pinMode(16, OUTPUT); 
  // pinMode(17, OUTPUT); 

  //8-bit PWM DAC pin
  pinMode(11, OUTPUT);

  // Set up Timer 1 to send a sample every interrupt.
  cli();
  // Set CTC mode
  // Have to set OCR1A *after*, otherwise it gets reset to 0!
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));
  // No prescaler
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
  // Set the compare register (OCR1A).
  // OCR1A is a 16-bit register, so we have to do this with
  // interrupts disabled to be safe.
  //OCR1A = F_CPU / SAMPLE_RATE;
  // Enable interrupt when TCNT1 == OCR1A
  TIMSK1 |= _BV(OCIE1A);
  sei();
  OCR1A = 800;  //40KHz Samplefreq

  // Set up Timer 2 to do pulse width modulation on D11

  // Use internal clock (datasheet p.160)
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));

  // Set fast PWM mode  (p.157)
  TCCR2A |= _BV(WGM21) | _BV(WGM20);
  TCCR2B &= ~_BV(WGM22);

  // Do non-inverting PWM on pin OC2A (p.155)
  // On the Arduino this is pin 11.
  TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
  TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
  // No prescaler (p.158)
  TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

  // Set initial pulse width to the first sample.
  OCR2A = 128;

  // Set up Timer 0 for tempo control (1kHz / 1ms)
  TCCR0A = 0; // Set entire TCCR0A register to 0
  TCCR0B = 0; // Same for TCCR0B
  TCNT0 = 0;  // Initialize counter value to 0

  // Set compare match register for 1kHz increments
  OCR0A = 249; // (16*10^6) / (1000*64) - 1 (must be <256)
  TCCR0A |= (1 << WGM01); // Turn on CTC mode
  TCCR0B |= (1 << CS01) | (1 << CS00); // 64 prescaler

  TIMSK0 |= (1 << OCIE0A); // Enable Timer0 compare interrupt

  // set up the ADC
  SFREQ = analogRead(0);
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library
  // Choose prescaler PS_128.
  ADCSRA |= PS_128;
  ADMUX = 64;
  sbi(ADCSRA, ADSC);

  Serial.begin(9600);

  // Serial.println("Modified Jan Ostman O2 Minipops!");
  // Serial.println("[j] for next pattern, [k] for previous");
  // Serial.println("[f] to speed up, [d] to slow down.  [space] to pause.");
}

void loop() {

  int16_t total;
  
  uint8_t MUX = 4;

  patlength = pgm_read_byte_near(patlen + patselect);

  // leon add start stop
  int startButtonState;            // the current reading from the input pin
  int lastStartButtonState = HIGH;  // the previous reading from the input pin
  unsigned long lastStartButtonDebounceTime = 0;  // the last time the output pin was toggled

  int fsButtonState;            // the current reading from the input pin
  int lastFsButtonState = LOW;  // the previous reading from the input pin
  unsigned long lastFsButtonDebounceTime = 0;  // the last time the output pin was toggled

  unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

  unsigned long debounceCount = 0;
  unsigned long debounceMax = 1000;


  while (1) {

    //------ Add current sample word to ringbuffer FIFO --------------------

    if (RingCount < 255) {  //if space in ringbuffer
      total = 0;
      if (samplecntBD) {
        total += (pgm_read_byte_near(BD + samplepntBD++) - 128);
        samplecntBD--;
      }
      if (samplecntBG2) {
        total += (pgm_read_byte_near(BG2 + samplepntBG2++) - 128);
        samplecntBG2--;
      }
      if (samplecntCL) {
        total += (pgm_read_byte_near(CL + samplepntCL++) - 128);
        samplecntCL--;
      }
      if (samplecntCW) {
        total += (pgm_read_byte_near(CW + samplepntCW++) - 128);
        samplecntCW--;
      }
      if (samplecntCY) {
        total += (pgm_read_byte_near(CY + samplepntCY++) - 128);
        samplecntCY--;
      }
      if (samplecntGU) {
        total += (pgm_read_byte_near(GU + samplepntGU++) - 128);
        samplecntGU--;
      }
      if (samplecntMA) {
        total += (pgm_read_byte_near(MA + samplepntMA++) - 128);
        samplecntMA--;
      }
      if (samplecntQU) {
        total += (pgm_read_byte_near(QU + samplepntQU++) - 128);
        samplecntQU--;
      }
      if (total < -127) total = -127;
      if (total > 127) total = 127;
      cli();
      Ringbuffer[RingWrite] = total + 128;
      RingWrite++;
      RingCount++;
      sei();
    

     //----------------------------------------------------------------------------
      // Start Stop
      // read the state of the switch into a local variable:
      int reading = digitalReadFast(10);
      // If the switch changed, due to noise or pressing:
      if (reading != lastStartButtonState) {
        // reset the debouncing timer
        lastStartButtonDebounceTime = debounceCount;
      }
      if ((debounceCount - lastStartButtonDebounceTime) < debounceDelay) {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state:
        // if the button state has changed:
        if (reading != startButtonState) {
          startButtonState = reading;

          if ( !startButtonState) {
            stepcnt = 0;
            playing = !playing;
            // debounceCount = 0;
          }
        }
      }
      // save the reading. Next time through the loop, it'll be the lastButtonState:
      lastStartButtonState = reading;

      // FS
      int readingFS = digitalReadFast(12);
      // If the switch changed, due to noise or pressing:
      if (readingFS != lastFsButtonState) {
        // reset the debouncing timer
        lastFsButtonDebounceTime = debounceCount;
      }
      if ((debounceCount - lastFsButtonDebounceTime) < debounceDelay) {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state:
        // if the button state has changed:
        if (readingFS != fsButtonState) {
          fsButtonState = readingFS;

          if ( fsButtonState) {
            stepcnt = 0;
            playing = !playing;
            // debounceCount = 0;
          }
        }
      }
      // save the reading. Next time through the loop, it'll be the lastButtonState:
      lastFsButtonState = readingFS;

      debounceCount = debounceCount + 1 < debounceMax ? debounceCount + 1 : 0;
  // Serial.println(digitalReadFast(10));
    

  //--------- sequencer block REMOVED ----------------------------------------------
  
  //--------------- ADC block -------------------------------------
  if (!(ADCSRA & 64)) {
    uint16_t value = ((ADCL + (ADCH << 8)) >> 3) + 1;
    if (MUX == 6) tempo = (value * 2) + 40;  // Scale to ms (approx 40-300ms)

    // // if (MUX == 6) tempo = map(value, 1, 128,1250, 17633);
    // if (MUX == 6) tempo = map(value, 1, 128,1250, 9633) + (OCR1A * 3);
    if (MUX == 5) patselect = (value - 1) >> 3;
    if (MUX == 5) patlength = pgm_read_byte_near(patlen + patselect);
    // if (MUX == 4) OCR1A = (value << 2) + 256;
    if (MUX == 4) OCR1A = map(value, 1, 128, 600, 1000);

    MUX++;
    if (MUX == 8) MUX = 4;
    ADMUX = 64 | MUX; //Select MUX
    sbi(ADCSRA, ADSC); //start next conversation
  }
  //---------------------------------------------------------------
  }
}}
