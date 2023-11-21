#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include "RectangularGauge.h"
#include "balance.h"

#define TFT_CS     3
#define TFT_RST    41
#define TFT_DC     34

unsigned long lastTime = 0;
//only update the display every minMillis milliseconds
unsigned long minMillis = 20;

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Parameters for the gauges
const int gaugeWidth = 30;
const int gaugeHeight = 200;
const int gaugeSpacing = 8;

// Initialize six gauges
RectangularGauge gaugeKp(tft, 0, 20, gaugeWidth, gaugeHeight, ST77XX_BLUE, ST77XX_CYAN, MIN_KP, MAX_KP, ST77XX_BLACK);
RectangularGauge gaugeKi(tft, gaugeWidth + gaugeSpacing, 20, gaugeWidth, gaugeHeight, ST77XX_BLUE, ST77XX_YELLOW, MIN_KI, MAX_KI, ST77XX_BLACK);
RectangularGauge gaugeKd(tft, 2*(gaugeWidth + gaugeSpacing), 20, gaugeWidth, gaugeHeight, ST77XX_BLUE, ST77XX_WHITE, MIN_KD, MAX_KD, ST77XX_BLACK);
RectangularGauge gaugeTrim(tft, 3*(gaugeWidth + gaugeSpacing), 20, gaugeWidth, gaugeHeight, ST77XX_BLUE, ST77XX_RED, MIN_TRIM, MAX_TRIM, ST77XX_BLACK);


void displaySetup() {
  tft.init(240, 320);           // Init TFT with width and height
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK); // Fill the screen with black

  // Initialize gauges
  gaugeKp.initialize();
  gaugeKi.initialize();
  gaugeKd.initialize();
  gaugeTrim.initialize();
}

void updateDisplay(double Kp, double Ki, double Kd, double trim){
  if (millis() - lastTime < minMillis) {
    return;
  }
  gaugeKp.updateValue(Kp);
  gaugeKi.updateValue(Ki);
  gaugeKd.updateValue(Kd);
  gaugeTrim.updateValue(trim);
  lastTime = millis();
}
