#ifndef PREPROCESSING_H
#define PREPROCESSING_H

#include <cmath>
#include <algorithm>
#undef abs  // Undefine the abs macro to avoid conflicts with the C++ standard library
#include <complex.h>

// Constants
const int samples = 128;
#define PI 3.141592653589793238462643383279

// Define the complex type
typedef std::complex<float> Complex;

// FFT function that returns a new array containing the FFT values
Complex* fft(const Complex* x, int N);

// Function to compute Power Spectral Density (PSD)
void psd(const Complex* x, int N, float sample_rate, float* psd_values, float* frequencies);

// Function to compute mean
float computeMean(float* data, int n);

// Function to compute standard deviation
float computeStdDev(float* data, int n);

// Function to compute median
float computeMedian(float* data, int n);

// Function to compute percentile
float computePercentile(float* data, int n, float percentile);

// Function to compute max
float computeMax(float* data, int n);

// Function to compute min
float computeMin(float* data, int n);

// Function to compute peak frequency and power
void computePeak(float* psd, float* freqs, int psdSize, float& peak_freq, float& peak_power);

// Function to compute the range of a segement
float computeRange(float* data, int n);

#endif // PREPROCESSING_H
