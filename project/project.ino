#include <Adafruit_NeoPixel.h>

#define NUM_LEDS_PER_STRIP 7
// Define 4 independent NeoPixel control objects
Adafruit_NeoPixel strips[4] = {
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 0, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 1, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 2, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 3, NEO_GRB + NEO_KHZ800)
};

struct FSRSensor {
  int pin;
  int offset;
  int maxAdcInWindow;
};

FSRSensor sensors[4] = {
  {A1, 0, 0}, {A2, 0, 0}, {A3, 0, 0}, {A4, 0, 0}
};

const int ADC_MAX_VAL = 630;
unsigned long lastWindowTime = 0;
const unsigned long WINDOW_MS = 700; // Modified to 0.7s window

void setup() {
  Serial.begin(9600);
  
  // Initialize strips and perform zero-point calibration
  for(int i = 0; i < 4; i++) {
    strips[i].begin();
    strips[i].setBrightness(50);
    
    long sum = 0;
    for(int j = 0; j < 30; j++) sum += analogRead(sensors[i].pin);
    sensors[i].offset = sum / 30;
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. Sampling Phase: Constantly capture the peak value for each sensor within the window
  for(int i = 0; i < 4; i++) {
    int val = analogRead(sensors[i].pin);
    if (val > sensors[i].maxAdcInWindow) {
      sensors[i].maxAdcInWindow = val;
    }
  }

  // 2. Settlement Phase: Process data every 0.7 seconds
  if (currentMillis - lastWindowTime >= WINDOW_MS) {
    
    // Serial print and update LED strips
    for(int i = 0; i < 4; i++) {
      int peak = constrain(sensors[i].maxAdcInWindow, sensors[i].offset, ADC_MAX_VAL);
      float weight = map(peak, sensors[i].offset, ADC_MAX_VAL, 0, 10000) / 1000.0;
      
      if (weight < 3.5) {
        weight = 0.00;
        strips[i].clear();
      } else {
        // Display color only if greater than threshold
        float constrainedWeight = constrain(weight, 3.0, 10.0);
        uint16_t hue = map(constrainedWeight * 100, 300, 1000, 21845, 0);
        strips[i].fill(strips[i].ColorHSV(hue, 255, 255));
      }
      strips[i].show();

      // Serial output
      Serial.print("S"); Serial.print(i + 1);
      Serial.print(": "); Serial.print(weight, 2);
      Serial.print("kg | ");

      // Reset sampling data
      sensors[i].maxAdcInWindow = 0; 
    }
    Serial.println(); // New line

    lastWindowTime = currentMillis;
  }
}