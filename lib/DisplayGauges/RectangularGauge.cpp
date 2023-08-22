#include "RectangularGauge.h"

RectangularGauge::RectangularGauge(Adafruit_ST7789 &display, int16_t posX, int16_t posY, int16_t w, int16_t h, uint16_t oColor, uint16_t iColor, float minVal, float maxVal, uint16_t backgroundColor)
: tft(display), x(posX), y(posY), width(w), height(h), outerColor(oColor), innerColor(iColor), minValue(minVal), maxValue(maxVal), currentValue(minVal), bgColor(backgroundColor), visible(true) {}

void RectangularGauge::initialize() {
    tft.fillRect(x, y, width, height, outerColor);
}

void RectangularGauge::updateValue(float value) {
    if(!visible) return;

    if (value < minValue) value = minValue;
    if (value > maxValue) value = maxValue;

    // Calculate fill height based on value
    int16_t newFillHeight = ((value - minValue) / (maxValue - minValue)) * height;
    int16_t currentFillHeight = ((currentValue - minValue) / (maxValue - minValue)) * height;

    // If new value is greater than the current, fill with inner color
    if (newFillHeight > currentFillHeight) {
        tft.fillRect(x, y + height - newFillHeight, width, newFillHeight - currentFillHeight, innerColor);
    }
    // If new value is less than the current, fill with outer color
    else if (newFillHeight < currentFillHeight) {
        tft.fillRect(x, y + height - currentFillHeight, width, currentFillHeight - newFillHeight, outerColor);
    }

    // Update the current value
    currentValue = value;
}

void RectangularGauge::setLimits(float minVal, float maxVal) {
    clear();
    minValue = minVal;
    maxValue = maxVal;
}

void RectangularGauge::setPosition(int16_t newX, int16_t newY) {
    clear();
    x = newX;
    y = newY;
}

void RectangularGauge::toggleVisibility() {
    if(visible) {
        clear();
        visible = false;
    } else {
        visible = true;
        initialize();
        updateValue(currentValue); // Redraw the current value
    }
}

void RectangularGauge::clear() {
    tft.fillRect(x, y, width, height, bgColor);
}
