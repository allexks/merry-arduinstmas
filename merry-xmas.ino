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
#define NUM_PINS 4

const int PINS[NUM_PINS] = {WHITE_PIN, GREEN_PIN, ORANGE_PIN, YELLOW_PIN};

enum modes {
  ALL_ON,
  ALL_BLINK_CONSECUTIVELY,
  ALL_BLINK_TOGETHER,
  __COUNT__ // do not remove; always put at end
};

const modes PROGRAM[] = {
    ALL_BLINK_CONSECUTIVELY,
    ALL_BLINK_TOGETHER,
    ALL_ON
};

uint8_t programIndex = 0;

#define MODE_UPDATE_INTV 8000   // ms
#define DISPLAY_UPDATE_INTV 200 // ms

long prevMillisMode = 0;
long prevMillisDisplay = 0;
long currMillisMode = 0;
long currMillisDisplay = 0;


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
  currMillisMode = millis();
  prevMillisDisplay = millis();
  currMillisDisplay = millis();
}


void loop() {
    drawIcons();
    delay(200);
    eraseAndMoveIcons();
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
