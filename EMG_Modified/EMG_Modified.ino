#include "dual_model.h"
#include "test_emg_data.h"

// Scaling constants from the Colab Scaler cell
float scaler_mean[] = {0.05661, 0.09641, 0.07488, 16.1463, 24.2009, 19.9889, 0.07299, 0.12500, 0.09832, 97.9235, 72.5543, 101.1743, 174.7765, 165.2671, 180.1056};
float scaler_std[] = {0.06671, 0.09636, 0.10294, 15.3356, 23.9686, 22.1113, 0.08422, 0.12751, 0.13431, 27.8236, 26.6055, 26.5265, 28.4202, 32.0256, 32.7791};

const char* GESTURES[] = {"Fist", "Thumb", "Flex", "Ext", "Rest"};

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("--- Offline Inference Test ---");

  int correct = 0;

  for (int w = 0; w < NUM_TEST_WINDOWS; w++) {
    float features[15];
    
    // 1. Feature Extraction (from the static test data)
    for (int c = 0; c < 3; c++) {
        float mav = 0, wl = 0, rms = 0;
        int zc = 0, ssc = 0;
        for (int i = 0; i < 375; i++) {
            float val = test_emg_data[w][c][i];
            mav += abs(val);
            rms += val * val;
            if (i > 0) {
                float prev = test_emg_data[w][c][i-1];
                wl += abs(val - prev);
                if (prev * val < 0 && abs(val-prev) > 0.01) zc++;
                if (i > 1) {
                  float prev2 = test_emg_data[w][c][i-2];
                  if ((prev-prev2)*(val-prev) < 0 && abs(val-prev) > 0.01) ssc++;
                }
            }
        }
        features[c] = mav / 375.0;
        features[c+3] = wl;
        features[c+6] = sqrt(rms / 375.0);
        features[c+9] = (float)zc;
        features[c+12] = (float)ssc;
    }

    // 2. Scaling
    float scaled[15];
    for(int i=0; i<15; i++) scaled[i] = (features[i] - scaler_mean[i]) / scaler_std[i];

    // 3. Inference
    int pred = predict_gesture(scaled);
    float intensity = predict_intensity(scaled);

    if (pred == test_labels[w]) correct++;

    Serial.print("Window "); Serial.print(w);
    Serial.print(" | Target: "); Serial.print(GESTURES[test_labels[w]]);
    Serial.print(" | Pred: "); Serial.print(GESTURES[pred]);
    Serial.print(" | Intensity: "); Serial.print(intensity * 100);
    Serial.println("%");
  }

  Serial.print("\nFinal Offline Accuracy: ");
  Serial.print(((float)correct / NUM_TEST_WINDOWS) * 100);
  Serial.println("%");
}

void loop() {}