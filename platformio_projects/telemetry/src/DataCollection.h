
#include <cmath>
#include <vector>
#include "INA190.hpp"


const int SAMPLES = 100000;
INA190 currSense(22);

float calculateMean(const std::vector<float>& samples) {
  int sampleSize = samples.size();
  
  // Calculate the mean
  float mean = 0.0f;
  for (float sample : samples) {
    mean += sample;
  }
  mean /= sampleSize;
  return mean;
}

float calculateStandardDeviation(const std::vector<float>& samples, float mean) {
  int sampleSize = samples.size();

  // Calculate the sum of squared differences
  float SSD = 0.0f;
  for (float sample : samples) {
    float difference = sample - mean;
    SSD += difference * difference;
  }

  // Calculate the average of squared differences and find the square root
  float var = SSD / (sampleSize - 1);
  double stdev = std::sqrt(var);

  return stdev;
}

void collectData() {
    Serial.println("Starting data collection in 5 seconds...");
    delay(5*1000);
    Serial.println("Beginning data collection..");
    // put your setup code here, to run once:
    std::vector<float> samples(SAMPLES);
    for (int i = 0; i < SAMPLES; i++) {
      samples[i] = currSense.getCurrent();
    }
    
    float mean = calculateMean(samples);
    Serial.print("Mean: ");Serial.println(mean);

    float standardDeviation = calculateStandardDeviation(samples, mean);

    double marginOfError = 0.01;// * /* actual value */;
    double zScore = 1.96;
    int requiredSampleSize = static_cast<int>(ceil((zScore * zScore * standardDeviation * standardDeviation) / (marginOfError * marginOfError)));
  
    Serial.print("Samples collected: ");Serial.println(SAMPLES);
    Serial.print("Standard deviation collected: ");Serial.println(standardDeviation);
    Serial.print("Required sample size for 1% margin of error: ");Serial.println(requiredSampleSize);
}
