#ifndef RECTANGULAR_GAUGE_H
#define RECTANGULAR_GAUGE_H

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

class RectangularGauge {
private:
    Adafruit_ST7789 &tft;
    int16_t x, y, width, height;
    uint16_t outerColor, innerColor, bgColor;
    float minValue, maxValue, currentValue;
    bool visible;

    /**
     * Helper function to clear the display area of the gauge
     */
    void clear();

public:
    /**
     * Constructor to initialize the gauge.
     * 
     * @param display       Reference to the Adafruit ST7789 TFT display object.
     * @param posX          X-coordinate for the top-left corner of the gauge.
     * @param posY          Y-coordinate for the top-left corner of the gauge.
     * @param w             Width of the gauge.
     * @param h             Height of the gauge.
     * @param oColor        Color of the outer rectangle (border) of the gauge.
     * @param iColor        Color of the inner rectangle representing the value.
     * @param minVal        Minimum value the gauge can represent.
     * @param maxVal        Maximum value the gauge can represent.
     * @param backgroundColor Color used to clear/erase the gauge.
     */
    RectangularGauge(Adafruit_ST7789 &display, int16_t posX, int16_t posY, int16_t w, int16_t h, uint16_t oColor, uint16_t iColor, float minVal, float maxVal, uint16_t backgroundColor);

    /**
     * Initialize and draw the gauge on the display.
     */
    void initialize();

    /**
     * Update and display the given value on the gauge.
     * 
     * @param value    Value to be displayed on the gauge.
     */
    void updateValue(float value);

    /**
     * Set new limits for the values the gauge can represent.
     * 
     * @param minVal   New minimum value for the gauge.
     * @param maxVal   New maximum value for the gauge.
     */
    void setLimits(float minVal, float maxVal);

    /**
     * Update the position of the gauge on the screen.
     * 
     * @param newX     New X-coordinate for the top-left corner of the gauge.
     * @param newY     New Y-coordinate for the top-left corner of the gauge.
     */
    void setPosition(int16_t newX, int16_t newY);

    /**
     * Toggle the visibility of the gauge.
     * If it's currently displayed, it will be hidden.
     * If it's hidden, it will be displayed.
     */
    void toggleVisibility();
};

#endif // RECTANGULAR_GAUGE_H
