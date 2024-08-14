#include <TensorFlowLite.h>

#include <Arduino_LSM9DS1.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "walking_terrain_model_data.h"
#include "preprocessing.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

// Constants
constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
//tflite::MicroErrorReporter micro_error_reporter;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

// Accelerometer data buffer
const int kWindowSize = 128; // Adjust the window size as needed
float acc_buffer_x[kWindowSize];
float acc_buffer_y[kWindowSize];
float acc_buffer_z[kWindowSize];
float alt_buffer[kWindowSize];
int buffer_index = 0;

// FFT settings
//const int samples = kWindowSize;  // This should match the window size

const double samplingFrequency = 20;

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

// Mean and standard deviation values from the training scaler
const float means[] = {0.77690491, 0.18235898, 0.34426206};
const float scales[] = {0.37360713, 0.20918597, 0.30304547};

void setup() {
  delay(7000);
  Serial.begin(9600);
  //while (!Serial) { delay(10); }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  unsigned status;
  status = bme.begin(0x77);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }


  const tflite::Model* model = tflite::GetModel(g_walking_terrain_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model version does not match Schema version!");
    while (1);
  }

  static tflite::MicroMutableOpResolver<10> resolver;
  //resolver.AddLstm();
  resolver.AddFullyConnected(); // Add other operators if needed
  resolver.AddSoftmax(); // Add if you need a classification layer


  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;
  interpreter->AllocateTensors();

  input = interpreter->input(0);
  output = interpreter->output(0);
}

void loop() {
  float x, y, z, altitude;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

    // Normalize the accelerometer readings
    x = (x - means[0]) / scales[0];
    y = (y - means[1]) / scales[1];
    z = (z - means[2]) / scales[2];
    //altitude = (altitude - means[3]) / scales[3];

    acc_buffer_x[buffer_index] = x;
    acc_buffer_y[buffer_index] = y;
    acc_buffer_z[buffer_index] = z;
    alt_buffer[buffer_index] = altitude;
    buffer_index++;

    if (buffer_index == kWindowSize) {
      // Reset buffer index
      buffer_index = 0;

      // Compute features for x, y, z axes
      float features[47];
      int feature_index = 0;
      float fs = 20.0;  // Sampling frequency, adjust as needed
      for (int i = 0; i < 3; ++i) {
        float* buff;
        if (i == 0) buff = acc_buffer_x;
        else if (i == 1) buff = acc_buffer_y;
        else buff = acc_buffer_z;

        // Time domain features
        //Serial.println("In Mean");
        float mean = computeMean(buff, kWindowSize);

        //Serial.println("In Std Dev");
        float stddev = computeStdDev(buff, kWindowSize);

        //Serial.println("In Min");
        float minimum = computeMin(buff, kWindowSize);

        //Serial.println("In Max");
        float maximum = computeMax(buff, kWindowSize);

        //Serial.println("In Median");
        float median = computeMedian(buff, kWindowSize);

        //Serial.println("In Percentile");
        float p25 = computePercentile(buff, kWindowSize, 0.25);

        //Serial.println("In Percentile");
        float p75 = computePercentile(buff, kWindowSize, 0.75);

        features[feature_index++] = mean;
        features[feature_index++] = stddev;
        features[feature_index++] = minimum;
        features[feature_index++] = maximum;
        features[feature_index++] = median;
        features[feature_index++] = p25;
        features[feature_index++] = p75;

        // Frequency domain features

        // creating array to hold input for fft computation
        Complex fft_input[samples];
        float fft_magnitudes[samples];

        for (int i = 0; i < samples; i++) {
          fft_input[i] = Complex(buff[i], 0.0f);
        }

        Complex* fft_values = fft(fft_input, samples);

        for (int i = 0; i < samples; i++) {
          fft_magnitudes[i] = std::abs(fft_values[i]);
        }

        // Arrays to hold PSD values and frequencies
        float psd_values[samples / 2];
        float frequencies[samples / 2];

        // compute PSD
        //Serial.println("In psd");
        psd(fft_input, samples, fs, psd_values, frequencies);

        float peak_freq, peak_power;
        const int psdSize = samples / 2;

        //Serial.println("In Peak");
        computePeak(psd_values, frequencies, psdSize, peak_freq, peak_power);


        features[feature_index++] = computeMean(fft_magnitudes, samples);
        features[feature_index++] = computeStdDev(fft_magnitudes, samples);
        features[feature_index++] = peak_freq;
        features[feature_index++] = peak_power;
        features[feature_index++] = computeMean(psd_values, psdSize);
        features[feature_index++] = computeStdDev(psd_values, psdSize);
        features[feature_index++] = computeMin(psd_values, psdSize);
        features[feature_index++] = computeMedian(psd_values, psdSize);
      }

      // adding features for altitude
      features[feature_index++] = computeStdDev(alt_buffer, kWindowSize);
      features[feature_index++] = computeRange(alt_buffer, kWindowSize);

      Serial.println("Features for data");
      Serial.print("[");
      for (int i = 0; i < 47; ++i) {
        Serial.print(features[i], 6);
        Serial.print(", ");
      }
      Serial.println("]");

      // Pass features to model
      for (int i = 0; i < 47; ++i) {
        input->data.f[i] = features[i];
      }

      TfLiteStatus invoke_status = interpreter->Invoke();
      if (invoke_status != kTfLiteOk) {
        Serial.println("Invoke failed!");
        return;
      }

      // Get classification probabilities and print them
      const char* labels[] = {"downstairs", "flat ground", "none", "upstairs"};
      for (int i = 0; i < output->dims->data[1]; i++) {
        Serial.print(labels[i]);
        Serial.print(": ");
        Serial.println(output->data.f[i], 4); // Print with 6 decimal places for better precision
      }

      // Get class with highest prediction
      float max_value = output->data.f[0];
      int max_index = 0;
      for (int i = 1; i < output->dims->data[1]; i++) {
        if (output->data.f[i] > max_value) {
          max_value = output->data.f[i];
          max_index = i;
        }
      }

      Serial.print("Predicted Terrain: ");
      Serial.println(labels[max_index]);
    }
  }

  delay(1000 / 20); // Adjust delay as needed
}
