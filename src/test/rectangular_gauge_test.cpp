#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include "RectangularGauge.h"

#define TFT_CS     3
#define TFT_RST    41
#define TFT_DC     34


Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Parameters for the gauges
const int gaugeWidth = 30;
const int gaugeHeight = 200;
const int gaugeSpacing = 8;

// Initialize six gauges
RectangularGauge gauge1(tft, 0, 20, gaugeWidth, gaugeHeight, ST77XX_BLUE, ST77XX_GREEN, 0, 100, ST77XX_BLACK);
RectangularGauge gauge2(tft, gaugeWidth + gaugeSpacing, 20, gaugeWidth, gaugeHeight, ST77XX_BLUE, ST77XX_RED, 0, 100, ST77XX_BLACK);
RectangularGauge gauge3(tft, 2*(gaugeWidth + gaugeSpacing), 20, gaugeWidth, gaugeHeight, ST77XX_BLUE, ST77XX_YELLOW, 0, 100, ST77XX_BLACK);
RectangularGauge gauge4(tft, 3*(gaugeWidth + gaugeSpacing), 20, gaugeWidth, gaugeHeight, ST77XX_BLUE, ST77XX_CYAN, 0, 100, ST77XX_BLACK);
RectangularGauge gauge5(tft, 4*(gaugeWidth + gaugeSpacing), 20, gaugeWidth, gaugeHeight, ST77XX_BLUE, ST77XX_MAGENTA, 0, 100, ST77XX_BLACK);
RectangularGauge gauge6(tft, 5*(gaugeWidth + gaugeSpacing), 20, gaugeWidth, gaugeHeight, ST77XX_BLUE, ST77XX_WHITE, 0, 100, ST77XX_BLACK);

void setup() {
  tft.init(240, 320);           // Init TFT with width and height
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK); // Fill the screen with black

  // Initialize gauges
  gauge1.initialize();
  gauge2.initialize();
  gauge3.initialize();
  gauge4.initialize();
  gauge5.initialize();
  gauge6.initialize();
}

void loop() {
  // Simulating changes in values for the gauges.
  // Here we're using a simple sine function for variation but you can replace it with actual values from sensors or other sources.
  float t = millis() * 0.002; // A time factor for the sine function

  gauge1.updateValue(50 + 50 * sin(t));
  gauge2.updateValue(50 + 50 * sin(t + 1));
  gauge3.updateValue(50 + 50 * sin(t + 2));
  gauge4.updateValue(50 + 50 * sin(t + 3));
  gauge5.updateValue(50 + 50 * sin(t + 4));
  gauge6.updateValue(50 + 50 * sin(t + 5));

  delay(50);  // Add a short delay to slow down the loop and reduce screen flickering
}
