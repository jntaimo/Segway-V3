#ifndef FIRFILTER_H
#define FIRFILTER_H

class FIRFilter {
public:
    // Constructor: Initializes the FIR filter with a specified size
    FIRFilter(int size);

    // Destructor: Cleans up dynamically allocated memory
    ~FIRFilter();

    // Updates the filter with a new reading and returns the filtered value
    float update(float newReading);

    // Sets a new size for the filter and reinitializes it
    void setSize(int newSize);

private:
    // Helper function to initialize the buffer with zeros
    void initializeBuffer();

    float *gyroBuffer;  // Dynamic array to hold the gyro readings
    int bufferSize;     // Size of the buffer
    int bufferIndex;    // Current index in the buffer for the next reading
    float runningSum;   // Running sum of the last 'n' readings
    int count;          // Count of readings added, used during initial fill
};

#endif // FIRFILTER_H
