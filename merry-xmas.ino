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


void setup() {
  Serial.begin(9600);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)

  for (int i=0; i < NUM_PINS; i++) {
    // turn on each LED
    pinMode(PINS[i], OUTPUT);
    digitalWrite(PINS[i], HIGH);
  }

  display.display();
  delay(2000);
  display.clearDisplay();

  for (uint8_t f=0; f < NUMFLAKES; f++) {
    initializeIcon(icons[f]);

    Serial.print("x: ");
    Serial.print(icons[f][XPOS], DEC);
    Serial.print(" y: ");
    Serial.print(icons[f][YPOS], DEC);
    Serial.print(" dy: ");
    Serial.println(icons[f][DELTAY], DEC);
  }


}

void loop() {
    int w = LOGO16_GLCD_WIDTH;
    int h = LOGO16_GLCD_HEIGHT;

    // draw each icon
    for (uint8_t f=0; f < NUMFLAKES; f++) {
      display.drawBitmap(icons[f][XPOS], icons[f][YPOS], LOGO16_GLCD_BMP, w, h, WHITE);
    }
    display.display();
    delay(200);

    // then erase it + move it
    for (uint8_t f=0; f < NUMFLAKES; f++) {
      display.drawBitmap(icons[f][XPOS], icons[f][YPOS],  LOGO16_GLCD_BMP, w, h, BLACK);

      // move it
      icons[f][YPOS] += icons[f][DELTAY];

      // if its gone, reinit
      if (icons[f][YPOS] > display.height()) {
        initializeIcon(icons[f]);
      }
    }
}

void initializeIcon(uint8_t* icon) {
    icon[XPOS] = random(display.width());
    icon[YPOS] = 0;
    icon[DELTAY] = random(5) + 1;
}
