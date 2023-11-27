#include "FIRFilter.h"
#include <cstring> // For std::fill_n
#include <algorithm> //for std::fill_n

FIRFilter::FIRFilter(int size) {
    bufferSize = size;
    gyroBuffer = new float[bufferSize];
    bufferIndex = 0;
    runningSum = 0.0;
    count = 0;
    initializeBuffer();
}

FIRFilter::~FIRFilter() {
    delete[] gyroBuffer;
}

void FIRFilter::initializeBuffer() {
    std::fill_n(gyroBuffer, bufferSize, 0.0f); // Fill the buffer with zeros
}

float FIRFilter::update(float newReading) {
    // Update running sum: subtract the oldest value and add the new one
    runningSum -= gyroBuffer[bufferIndex];
    runningSum += newReading;

    // Store the new reading in the buffer
    gyroBuffer[bufferIndex] = newReading;
    bufferIndex = (bufferIndex + 1) % bufferSize; // Circular buffer index

    // Calculate average: if buffer isn't full, use count; otherwise, use bufferSize
    if (count < bufferSize) {
        count++;
        return runningSum / count;
    }

    return runningSum / bufferSize;
}

void FIRFilter::setSize(int newSize) {
    // Only proceed if the new size is different from the current size
    if (newSize != bufferSize) {
        delete[] gyroBuffer; // Free the existing buffer
        bufferSize = newSize; // Update the size
        gyroBuffer = new float[bufferSize]; // Allocate a new buffer
        bufferIndex = 0; // Reset index
        runningSum = 0.0; // Reset running sum
        count = 0; // Reset count
        initializeBuffer(); // Initialize the new buffer
    }
}
