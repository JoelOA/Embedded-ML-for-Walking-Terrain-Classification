#include "preprocessing.h"

// Function to reverse bits
unsigned int reverseBits(unsigned int num, unsigned int log2n) {
    unsigned int result = 0;
    for (unsigned int i = 0; i < log2n; i++) {
        if (num & (1 << i)) {
            result |= 1 << (log2n - 1 - i);
        }
    }
    return result;
}

// FFT function that returns a new array containing the FFT values
Complex* fft(const Complex* x, int N) {
    int logN = log2(N);

    // Bit-reversal permutation
    Complex *y = (Complex *)malloc(N * sizeof(Complex));
    for (int i = 0; i < N; i++) {
        int j = reverseBits(i, logN);
        y[j] = x[i];
    }

    // Cooley-Tukey iterative FFT
    int m = 2;
    while (m <= N) {
        Complex wm = Complex(cos(-2 * PI / m), sin(-2 * PI / m));

        for (int k = 0; k < N; k += m) {
            Complex w = Complex(1, 0);

            for (int j = 0; j < m / 2; j++) {
                Complex t = w * y[k + j + m / 2];
                Complex u = y[k + j];

                y[k + j] = u + t;
                y[k + j + m / 2] = u - t;

                w = w * wm;
            }
        }
        m *= 2;
    }

    return y;
}

void psd(const Complex* x, int N, float sample_rate, float* psd_values, float* frequencies) {

    // Compute FFT
    Complex* fft_x = fft(x, N);

    // Compute PSD and frequencies
    for (int k = 0; k < N / 2; k++) {
        psd_values[k] = (std::abs(fft_x[k]) * std::abs(fft_x[k])) / N;
        frequencies[k] = sample_rate * k / N;
    }

    delete[] fft_x;
}

// Function to convert complex array to float array containing only the real parts
void convertComplexToRealFloat(const Complex* complexArray, float* floatArray, int size) {
    for (int i = 0; i < size; i++) {
        floatArray[i] = complexArray[i].real();
    }
}

// Function to compute mean
float computeMean(float* data, int n) {
    float sum = 0.0;
    for (int i = 0; i < n; i++) {
        sum += data[i];
    }
    return sum / n;
}

// Function to compute standard deviation
float computeStdDev(float* data, int n) {
    float mean = computeMean(data, n);
    float variance_sum = 0.0;
    for (int i = 0; i < n; i++) {
        variance_sum += pow(data[i] - mean, 2);
    }
    return sqrt(variance_sum / n);
}

// Function to compute median
float computeMedian(float* data, int n) {
    std::sort(data, data + n);
    if (n % 2 == 0) {
        return (data[n / 2 - 1] + data[n / 2]) / 2;
    } else {
        return data[n / 2];
    }
}

// Function to compute percentile
float computePercentile(float* data, int n, float percentile) {
    std::sort(data, data + n);
    int index = (int)(percentile * n);
    return data[index];
}

// Function to compute max
float computeMax(float* data, int n) {
    float max_value = data[0];
    for (int i = 1; i < n; i++) {
        if (data[i] > max_value) {
            max_value = data[i];
        }
    }
    return max_value;
}

// Function to compute min
float computeMin(float* data, int n) {
    float min_value = data[0];
    for (int i = 1; i < n; i++) {
        if (data[i] < min_value) {
            min_value = data[i];
        }
    }
    return min_value;
}

// Function to compute peak frequency and power
void computePeak(float* psd, float* freqs, int psdSize, float& peak_freq, float& peak_power) {
    float max_psd = 0;
    int peak_index = 0;

    for (int i = 0; i < psdSize; i++) {
        if (psd[i] > max_psd) {
            max_psd = psd[i];
            peak_index = i;
        }
    }

    peak_freq = freqs[peak_index];
    peak_power = max_psd;
}

float computeRange(float* data, int n) {
  // Compute the range as the difference between the most recent reading and the first reading
  float range = data[n-1] - data[0];
  return range;
}
