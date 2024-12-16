//*********************User Display Code for Perculator Arduino Drum Machine**********************/
// OLED display and LED interface code by Jupertronic aka Janis Wilson Hughes aka J Dub aka vitalWho aka Evolution Stoneware (youtube)
// OLED shows what pattern you're playing with pattern number and pattern name as selected by the pattern potentiometer.
// Pins for OLED are dependent on type of Arduino used. This is set up for Nano. If you're using Pro Micro SDA = D2, SCL = D3. Check your SDA & SCL and adjust accordingly.
// This is designed for Mark Dammer's Wee O3 version of Jan Ostman's O2 Mini Pops project revived by Bloghoskins. Sounds emulate Korg Mini Pops drum machine.
// See my Perculator code for extensive changes to the pattern beats corresponding to the pattern names here and minor additions for LEDs.
// LEDs are run off this board to remove potential sound interference on main Perculator board since we have this board available to handle the OLED screen.
// You will need to install the ss_oled library to run the display.
// Change the pattern names as desired in the code. These are promarily 4 on the floor 80s dance beats for Perculator. Boot n' pants is beat box slang. 

#include <ss_oled.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

// Standard Arduino Pins

#define digitalPinToPortReg(P) \
  (((P) >= 0 && (P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define digitalPinToDDRReg(P) \
  (((P) >= 0 && (P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : &DDRC))
#define digitalPinToPINReg(P) \
  (((P) >= 0 && (P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : &PINC))
#define digitalPinToBit(P) \
  (((P) >= 0 && (P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P) - 8 : (P) - 14))


#define digitalReadFast(P) bitRead(*digitalPinToPINReg(P), digitalPinToBit(P))

#define digitalWriteFast(P, V) bitWrite(*digitalPinToPortReg(P), digitalPinToBit(P), (V))

// if your system doesn't have enough RAM for a back buffer, comment out
// this line (e.g. ATtiny85)
#define USE_BACKBUFFER

#ifdef USE_BACKBUFFER
static uint8_t ucBackBuffer[1024];
#else
static uint8_t *ucBackBuffer = NULL;
#endif

// Use -1 for the Wire library default pins
// or specify the pin numbers to use with the Wire library or bit banging on any GPIO pins
// These are the pin numbers for the M5Stack Atom default I2C
#define SDA_PIN 18
#define SCL_PIN 19
// Set this to -1 to disable or the GPIO pin number connected to the reset
// line of your display if it requires an external reset
#define RESET_PIN -1
// let ss_oled figure out the display address
#define OLED_ADDR -1
// don't rotate the display
#define FLIP180 0
// don't invert the display
#define INVERT 0
// Bit-Bang the I2C bus
#define USE_HW_I2C 0

// Change these if you're using a different OLED display
#define MY_OLED OLED_128x64
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
//#define MY_OLED OLED_64x32
//#define OLED_WIDTH 64
//#define OLED_HEIGHT 32

SSOLED ssoled;

int count = 0;


//Input Pins
#define patternPot A0

//new

#define playState 2
#define stepPulse 3


void setup() {
  //  Serial.begin(9600);
 
  pinMode(playState, INPUT);
  pinMode(stepPulse, INPUT_PULLUP);
  pinMode(patternPot, INPUT);

  //Initialize OLED display
  int rc;
  // The I2C SDA/SCL pins set to -1 means to use the default Wire library
  // If pins were specified, they would be bit-banged in software
  // This isn't inferior to hw I2C and in fact allows you to go faster on certain CPUs
  // The reset pin is optional and I've only seen it needed on larger OLEDs (2.4")
  //    that can be configured as either SPI or I2C
  //
  // oledInit(SSOLED *, type, oled_addr, rotate180, invert, bWire, SDA_PIN, SCL_PIN, RESET_PIN, speed)

  rc = oledInit(&ssoled, MY_OLED, OLED_ADDR, FLIP180, INVERT, USE_HW_I2C, SDA_PIN, SCL_PIN, RESET_PIN, 400000L); // use standard I2C bus at 400Khz


  oledFill(&ssoled, 1, 1);
  oledFill(&ssoled, 0, 1);
  //  oledDumpBuffer(&ssoled, NULL);

  oledWriteString(&ssoled, 0, 0, 1, (char *)" hdl minipops       ", FONT_NORMAL, 1, 1);
  oledWriteString(&ssoled, 0, 0, 3, (char *)"#0#0#0#0#0#0#0", FONT_STRETCHED, 0, 1);
  oledWriteString(&ssoled, 0, 0, 6, (char *)"popopopop", FONT_STRETCHED, 0, 1);

  delay(1000);

  oledFill(&ssoled, 0, 1);
  oledDumpBuffer(&ssoled, NULL);

  oledWriteString(&ssoled, 0, 0, 0, (char *)" hdl mini pops  ", FONT_NORMAL, 1, 1);
  oledWriteString(&ssoled, 0, 5, 7, (char *)"       #0        ", FONT_NORMAL, 0, 1);

  Serial.begin(9600);
}

uint8_t lastStepPulse;
unsigned long lastMillis;

// unsigned long bpm;
const uint8_t maxcount = 60;
float bpm;
float lastBpm[maxcount];
uint8_t bpmcounter = 0;


void loop() {
  update_OLED();
}

void update_OLED () {
// Read pattern pot, depending on the value update the display with the current pattern selection
  int value = analogRead(patternPot);
  int pattern = map(value, 0, 1023, 1, 16);

  // int stepAdvance = digitalReadFast(stepPulse);
  char* playing = digitalReadFast(playState) ? ">" : "=";

  // Serial.print(" stepPulse: ");
  // int stp = digitalReadFast(stepPulse);
  // Serial.print(stp);
  // Serial.print(" playState: ");
  // Serial.println(digitalReadFast(playState));
  // Serial.println(millis());

  uint8_t currentStepPulse = digitalReadFast(stepPulse);

  if ( currentStepPulse != lastStepPulse ) {
    lastStepPulse = currentStepPulse;
    // Serial.println(currentStepPulse);
    if ( currentStepPulse ) {
      bpm = (60000 / (millis() - lastMillis) / 2);
      lastBpm[bpmcounter] = bpm;
      bpmcounter = bpmcounter + 1 < maxcount ? bpmcounter + 1 : 0;

      // Serial.println(average);
      lastMillis = millis();
    }
  }

  float average = 0;

  for( int i = 0; i < maxcount; i++ ) {
    average += lastBpm[i];
  }

  bpm = average / maxcount;


  /*
    16 steps
    ------------
    1 Hard rock
    2 Disco
    3 Reggae
    4 Rock
    5 Samba
    6 Rumba
    7 Cha-Cha
    8 Swing
    9 Bossa Nova
    10 Beguine
    11 Synthpop

    12-steps
    ---------
    12 Boogie
    13 Waltz
    14 Jazz rock
    15 Slow rock
    16 Oxygen

  */
  char* patternName   = "                  ";
  char* patternLength = "                  ";

  switch (pattern)
  {
    case 1:
      patternName = "   Hard rock   ";
      patternLength = "16";
      break;
    case 2:
      patternName = "     Disco   ";
      patternLength = "16";
      break;
    case 3:
      patternName = "    Reggae   ";
      patternLength = "16";
      break;
    case 4:
      patternName = "     Rock   ";
      patternLength = "16";
      break;
    case 5:
      patternName = "     Samba   ";
      patternLength = "16";
      break;
    case 6:
      patternName = "     Rumba   ";
      patternLength = "16";
      break;
    case 7:
      patternName = "     Cha-Cha   ";
      patternLength = "16";
      break;
    case 8:
      patternName = "     Swing   ";
      patternLength = "16";
      break;
    case 9:
      patternName = "   Bossa Nova   ";
      patternLength = "16";
      break;
    case 10:
      patternName = "     Beguine   ";
      patternLength = "16";
      break;
    case 11:
      patternName = "   Synthpop   ";
      patternLength = "16";  
      break;
    case 12:
      patternName = "     Boogie    ";
      patternLength = "12";
      break;
    case 13:
      patternName = "      Waltz     ";
      patternLength = "12";  
      break;
    case 14:
      patternName = "    Jazz rock   ";
      patternLength = "12";  
      break;
    case 15:
      patternName = "    Slow rock   ";
      patternLength = "12"; 
     break;
    case 16:
      patternName = "    Oxygen   ";
      patternLength = "12";
      break;
  }

  char buffer [32];
  sprintf(buffer, " %02d ", pattern);
  char buffer2 [32];
  sprintf(buffer2, " %03dBPM", int(bpm));
  oledWriteString(&ssoled, 0, 30, 2,(char *) buffer , FONT_STRETCHED, 0, 1);
  oledWriteString(&ssoled, 0, 0, 4, (char *) patternName, FONT_NORMAL, 0, 1);
  oledWriteString(&ssoled, 0, 52, 5, (char *) patternLength, FONT_NORMAL, 0, 1);

  oledWriteString(&ssoled, 0, 5, 7, playing, FONT_NORMAL, 0, 1);  
  oledWriteString(&ssoled, 0, 52, 7, buffer2, FONT_NORMAL, 0, 1);  
}