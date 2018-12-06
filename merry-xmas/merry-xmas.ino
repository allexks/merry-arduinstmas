#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 5
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  32

uint8_t icons[NUMFLAKES][3];

void initializeIcon(uint8_t*);
void drawIcons();
void eraseAndMoveIcons();

static const unsigned char PROGMEM LOGO16_GLCD_BMP[] =
{ B00100000, B00001000, B00100000, B00001000,
  B10100000, B10000010, B10000010, B00001010,
  B00001000, B00100010, B10001000, B00100000,
  B00000010, B00000010, B10001000, B10000000,
  B00100000, B10000010, B10000010, B00001000,
  B00001000, B00100010, B10001000, B00100000,
  B10000000, B00001010, B10100000, B00000010,
  B00101010, B10101000, B10101010, B10101000,
  B00101010, B10101010, B00101010, B10101000,
  B10000000, B00001010, B10100000, B00000010,
  B00001000, B00100010, B10001000, B00100000,
  B00100000, B10000010, B10000010, B00001000,
  B00000010, B00000010, B10001000, B10000000,
  B00001000, B00100010, B10001000, B00100000,
  B10100000, B10000010, B10000010, B00001010,
  B00100000, B00001000, B00100000, B00001000 };

#define WHITE_PIN 12
#define GREEN_PIN 10
#define ORANGE_PIN 6
#define YELLOW_PIN 2
#define NUM_PINS 5

const int PINS[NUM_PINS] = {
    WHITE_PIN,
    GREEN_PIN,
    LED_BUILTIN,
    YELLOW_PIN,
    ORANGE_PIN
};


enum program_types {
  ALL_ON,
  ALL_BLINK_CONSECUTIVELY,
  ALL_BLINK_TOGETHER,
};

#define PROGRAMS_COUNT 3

const program_types PROGRAM[PROGRAMS_COUNT] = {
    ALL_BLINK_CONSECUTIVELY,
    ALL_BLINK_TOGETHER,
    ALL_ON
};

struct ProgramManager {
    uint8_t programIndex;
    program_types currProgram;

    long blinkTogetherIntv;
    long prevBlinkTogetherMillis;

    long blinkConsecIntv;
    long prevBlinkConsecMillis;
    int currIndexBlink;
    int prevIndexBlink;

    ProgramManager();
    void nextProgram();
    void handleProgram();
} programManager;


#define MODE_UPDATE_INTV 8888   // ms
#define DISPLAY_UPDATE_INTV 200 // ms

long prevMillisMode = 0;
long prevMillisDisplay = 0;
long currMillis = 0;


void setup() {
  Serial.begin(9600);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)

  // splash screen
  for (int i=0; i < NUM_PINS; i++) {
    pinMode(PINS[i], OUTPUT);
    digitalWrite(PINS[i], HIGH);
  }
  display.display();
  delay(2000);
  display.clearDisplay();

  // initialize all icons
  for (uint8_t f=0; f < NUMFLAKES; f++) {
    initializeIcon(icons[f]);

    Serial.print("x: ");
    Serial.print(icons[f][XPOS], DEC);
    Serial.print(" y: ");
    Serial.print(icons[f][YPOS], DEC);
    Serial.print(" dy: ");
    Serial.println(icons[f][DELTAY], DEC);
  }

  // initial draw
  drawIcons();

  prevMillisMode = millis();
  prevMillisDisplay = millis();
  currMillis = millis();
}


void loop() {
    currMillis = millis();

    if (currMillis - DISPLAY_UPDATE_INTV >= prevMillisDisplay) {
        eraseAndMoveIcons();
        drawIcons();

        prevMillisDisplay = millis();
    }

    programManager.handleProgram();

    if (currMillis - MODE_UPDATE_INTV >= prevMillisMode) {
        programManager.nextProgram();
        prevMillisMode = millis();
    }
}


void initializeIcon(uint8_t* icon) {
    icon[XPOS] = random(display.width());
    icon[YPOS] = 0;
    icon[DELTAY] = random(5) + 1;
}

void drawIcons() {
    for (uint8_t f=0; f < NUMFLAKES; f++) {
      display.drawBitmap(icons[f][XPOS], icons[f][YPOS], LOGO16_GLCD_BMP, LOGO16_GLCD_WIDTH, LOGO16_GLCD_HEIGHT, WHITE);
    }
    display.display();
}

void eraseAndMoveIcons() {
    for (uint8_t f=0; f < NUMFLAKES; f++) {
      display.drawBitmap(icons[f][XPOS], icons[f][YPOS],  LOGO16_GLCD_BMP, LOGO16_GLCD_WIDTH, LOGO16_GLCD_HEIGHT, BLACK);

      // move it
      icons[f][YPOS] += icons[f][DELTAY];

      // if its gone, reinit
      if (icons[f][YPOS] > display.height()) {
        initializeIcon(icons[f]);
      }
    }
}

ProgramManager::ProgramManager() :  programIndex(0),
                                    blinkTogetherIntv(500),
                                    prevBlinkTogetherMillis(0),
                                    blinkConsecIntv(500),
                                    prevBlinkConsecMillis(0),
                                    currIndexBlink(0),
                                    prevIndexBlink(0)
{
    currProgram = PROGRAM[programIndex];
}

void ProgramManager::nextProgram() {
    programIndex++;
    if (programIndex == PROGRAMS_COUNT)
        programIndex = 0;
    currProgram  = PROGRAM[programIndex];

    switch (currProgram) {
    case ALL_ON:
    case ALL_BLINK_TOGETHER:
        for (int i=0; i < NUM_PINS; i++) {
          digitalWrite(PINS[i], HIGH);
        }
        break;
    case ALL_BLINK_CONSECUTIVELY:
        currIndexBlink = 0;
        prevIndexBlink = 0;
        break;
    }

    Serial.println((int&)currProgram);
}

void ProgramManager::handleProgram() {
    long currMillis = millis();

    switch (currProgram) {
    case ALL_ON:
        break;
    case ALL_BLINK_TOGETHER:
        if (currMillis - blinkTogetherIntv >= prevBlinkTogetherMillis) {
            boolean state = digitalRead(PINS[0]);
            state = !state;
            for (int i=0; i < NUM_PINS; i++) {
              digitalWrite(PINS[i], state);
            }
            prevBlinkTogetherMillis = millis();
        }

        break;
    case ALL_BLINK_CONSECUTIVELY:
        if (currMillis - blinkConsecIntv >= prevBlinkConsecMillis) {
            prevIndexBlink = currIndexBlink;
            if (prevIndexBlink == NUM_PINS - 1) {
                currIndexBlink = 0;
            } else {
                currIndexBlink++;
            }

            digitalWrite(PINS[prevIndexBlink], LOW);
            digitalWrite(PINS[currIndexBlink], HIGH);

            prevBlinkConsecMillis = millis();
        }
        break;
    }
}
