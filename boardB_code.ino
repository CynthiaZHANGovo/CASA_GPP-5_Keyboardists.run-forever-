#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <FreeRTOS_SAMD21.h>

// ================= 1. CORE HARDWARE CONFIGURATION =================
bool IS_BOARD_A = false; // Set to true for Board A, false for Board B

uint32_t getLedType(int index) {
  // Automatically adapt to RGB or RGBW strips based on board type/index
  if (index == 1) return NEO_GRBW + NEO_KHZ800; 
  return NEO_GRB + NEO_KHZ800;
}

// ================= 2. NETWORK CONFIGURATION =================
const char* ssid     = "CE-Hub-Student";
const char* password = "casa-ce-gagarin-public-service";
const char* mqtt_server = "mqtt.cetools.org";
const int   mqtt_port   = 1884;
const char* mqtt_user   = "student";
const char* mqtt_pass   = "ce2021-mqtt-forget-whale";

String myID     = IS_BOARD_A ? "Board_A" : "Board_B";
String remoteID = IS_BOARD_A ? "Board_B" : "Board_A";

// ================= 3. HARDWARE STATE & STRUCTURES =================
#define NUM_LEDS_PER_STRIP 7
Adafruit_NeoPixel strips[4] = {
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 0),
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 1),
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 2),
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 3)
};

// Sensor structure: added filteredVal for EMA filtering
struct FSRSensor { 
  int pin; 
  int offset; 
  int maxAdcInWindow; 
  float filteredVal; // [NEW] Stores the smoothed value after EMA filtering
};

FSRSensor sensors[4] = {{A1, 0, 0, 0}, {A2, 0, 0, 0}, {A3, 0, 0, 0}, {A4, 0, 0, 0}};

float activeWeight[4]      = {0, 0, 0, 0}; 
bool  weStartedThis[4]     = {false, false, false, false}; 
unsigned long startTime[4] = {0, 0, 0, 0};      
unsigned long lastClearTime[4] = {0, 0, 0, 0};  
const unsigned long TIMEOUT_MS  = 7000;         
const unsigned long COOLDOWN_MS = 750;          

const int ADC_MAX_VAL = 630;
const float HIT_THRESHOLD = 4.5; 

WiFiClient mkrClient;
PubSubClient client(mkrClient);

struct MqttMessage { int id; float val; };
QueueHandle_t mqttQueue;
SemaphoreHandle_t ledMutex;

// ================= FUNCTIONS =================

void triggerLED(int ledIdx, float weight, bool forceUpdate = false) {
  static float lastUpdateWeight[4] = {-1, -1, -1, -1};
  if (!forceUpdate && abs(weight - lastUpdateWeight[ledIdx]) < 0.1) return;
  lastUpdateWeight[ledIdx] = weight;

  if (xSemaphoreTake(ledMutex, portMAX_DELAY) == pdTRUE) {
    if (weight < 0.1) strips[ledIdx].clear();
    else {
      float constrainedWeight = constrain(weight, 4.5, 10.0);
      uint16_t hue = map(constrainedWeight * 100, 450, 1000, 21845, 0); 
      strips[ledIdx].fill(strips[ledIdx].ColorHSV(hue, 255, 255));
    }
    // Critical section for NeoPixel timing
    vTaskSuspendAll();
    noInterrupts();
    strips[ledIdx].show();
    interrupts();
    xTaskResumeAll();
    xSemaphoreGive(ledMutex);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  int receivedID = String(topic).substring(String(topic).lastIndexOf('/') + 1).toInt();
  float receivedWeight = msg.toFloat();

  int targetLED = receivedID;
  if (receivedID == 0) targetLED = 2;      
  else if (receivedID == 2) targetLED = 0; 

  if (receivedWeight > 0) {
    activeWeight[targetLED] = receivedWeight;
    weStartedThis[targetLED] = false; 
    startTime[targetLED] = millis(); 
    triggerLED(targetLED, receivedWeight, true);
  } else {
    activeWeight[targetLED] = 0;
    lastClearTime[targetLED] = millis(); 
    triggerLED(targetLED, 0, true);
  }
}

// ================= TASKS =================

// Task 1: Sampling & Core DSP Logic (EMA + Peak Detection)
void TaskSensing(void *pvParameters) {
  unsigned long lastWindowTime = millis();
  const float alpha = 0.25; // EMA coefficient: balances smoothness vs latency

  for (;;) {
    unsigned long currentMillis = millis();
    
    // --- Step 1: Real-time sampling and EMA low-pass filtering ---
    for(int i = 0; i < 4; i++) {
      int rawVal = analogRead(sensors[i].pin);
      
      // EMA Implementation: S_t = alpha * X_t + (1 - alpha) * S_{t-1}
      sensors[i].filteredVal = (alpha * (float)rawVal) + ((1.0 - alpha) * sensors[i].filteredVal);
      
      // --- Step 2: Peak detection within the current window ---
      // Use smoothed data to update peak, preventing noise-induced false triggers
      if ((int)sensors[i].filteredVal > sensors[i].maxAdcInWindow) {
        sensors[i].maxAdcInWindow = (int)sensors[i].filteredVal;
      }
    }

    // --- Step 3: Business logic detection (Timeout & state sync) ---
    for(int i = 0; i < 4; i++) {
      if (activeWeight[i] > 0 && (currentMillis - startTime[i] >= TIMEOUT_MS)) {
        activeWeight[i] = 0; lastClearTime[i] = currentMillis; 
        triggerLED(i, 0, true); 
        if (weStartedThis[i]) {
          MqttMessage m = {i, 0.0}; xQueueSend(mqttQueue, &m, 0);
        }
        weStartedThis[i] = false;
      }
    }

    // --- Step 4: 700ms window resolution (Decide hit events) ---
    if (currentMillis - lastWindowTime >= 700) {
      bool isAnyActive = false;
      for(int j=0; j<4; j++) { if(activeWeight[j] > 0) isAnyActive = true; }
      
      for(int i = 0; i < 4; i++) {
        // Map peak ADC value to physical weight (kg)
        int peak = constrain(sensors[i].maxAdcInWindow, sensors[i].offset, ADC_MAX_VAL);
        float weight = map(peak, sensors[i].offset, ADC_MAX_VAL, 0, 10000) / 1000.0;
        
        if (weight >= HIT_THRESHOLD) {
          if (activeWeight[i] == 0) {
            // Validated as a new local hit event
            if (!isAnyActive && (currentMillis - lastClearTime[i] >= COOLDOWN_MS)) {
              activeWeight[i] = weight; weStartedThis[i] = true;
              startTime[i] = currentMillis; triggerLED(i, weight, true);
              MqttMessage m = {i, weight}; xQueueSend(mqttQueue, &m, 0);
            }
          } else if (!weStartedThis[i]) {
            // Respond to clearing instruction from remote board
            if (weight >= activeWeight[i]) {
              activeWeight[i] = 0; weStartedThis[i] = false;
              lastClearTime[i] = currentMillis; triggerLED(i, 0, true);
              MqttMessage m = {i, 0.0}; xQueueSend(mqttQueue, &m, 0);
            }
          }
        }
        sensors[i].maxAdcInWindow = 0; // Reset window, prepare for next detection
      }
      lastWindowTime = currentMillis;
    }
    vTaskDelay(pdMS_TO_TICKS(40)); // Maintain ~25Hz frequency for stable filter performance
  }
}

void TaskNetwork(void *pvParameters) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!client.connected()) {
        if (client.connect(("Box-" + myID).c_str(), mqtt_user, mqtt_pass)) 
          client.subscribe(("student/boxing/" + remoteID + "/#").c_str());
      }
      client.loop();
    }
    MqttMessage outMsg;
    if (xQueueReceive(mqttQueue, &outMsg, 0) == pdPASS) {
      client.publish(("student/boxing/" + myID + "/sensor/" + String(outMsg.id)).c_str(), String(outMsg.val, 2).c_str());
    }
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}

// ================= SETUP =================

void setup() {
  Serial.begin(115200);

  // WiFi Connection
  WiFi.begin(ssid, password);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry++ < 40) {
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    for(int i = 0; i < 4; i++) {
      strips[i].updateType(getLedType(i)); 
      strips[i].begin();
      strips[i].setBrightness(50);
      
      // Power-on self-test blink
      strips[i].fill(strips[i].Color(120, 120, 120));
      strips[i].show();
    }
    delay(1000);
    for(int i = 0; i < 4; i++) { strips[i].clear(); strips[i].show(); }
  }

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Sensor baseline calibration and EMA initial value setup
  for(int i = 0; i < 4; i++) {
    long sum = 0; for(int j = 0; j < 30; j++) sum += analogRead(sensors[i].pin);
    sensors[i].offset = sum / 30;
    sensors[i].filteredVal = (float)sensors[i].offset; // Initial filter state
  }

  mqttQueue = xQueueCreate(10, sizeof(MqttMessage));
  ledMutex  = xSemaphoreCreateMutex();

  xTaskCreate(TaskSensing, "Sens", 512, NULL, 2, NULL);
  xTaskCreate(TaskNetwork, "Net", 1024, NULL, 1, NULL);

  vTaskStartScheduler();
}

void loop() {
  // FreeRTOS scheduler handles tasks; loop stays empty
}
