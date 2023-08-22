#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// Define pins for the TFT display
#define TFT_CS   3
#define TFT_RST   41
#define TFT_DC  34

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);  // 240x240 version

// Function to draw the radial gauge
void drawGauge(float value, float min_val, float max_val, int x_pos, int y_pos, int radius) {
  int angle_range[2] = {-145, 145};  // Gauge angle range
  
  // Clear previous gauge
  tft.fillCircle(x_pos, y_pos, radius, ST77XX_BLACK);
  
  // Calculate angle for value
  float mapped_value = map(value, min_val, max_val, angle_range[0], angle_range[1]);
  float value_rad = (3.14 / 180.0) * mapped_value;
  
  // Draw the gauge circle
  tft.drawCircle(x_pos, y_pos, radius, ST77XX_WHITE);
  
  // Draw the needle
  int end_x = x_pos + int(radius * sin(value_rad));
  int end_y = y_pos - int(radius * cos(value_rad));
  tft.drawLine(x_pos, y_pos, end_x, end_y, ST77XX_RED);
}

void setup() {
  tft.init(240, 320);  // Initialize TFT with screen size 240x240
  tft.setRotation(1);  // Adjust rotation as needed
  tft.fillScreen(ST77XX_BLACK);
}

void loop() {
  // This is just a test animation
  for(float val = 0; val <= 100; val += 1) {
    drawGauge(val, 0, 100, tft.width()/2, tft.height()/2, 60);  // Updated to center gauge on 240x240 screen
    delay(50);
  }
}
