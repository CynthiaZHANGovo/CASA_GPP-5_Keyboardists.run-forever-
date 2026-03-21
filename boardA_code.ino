#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <FreeRTOS_SAMD21.h>

// ================= CONFIGURATION =================
const char* ssid     = "CE-Hub-Student";
const char* password = "casa-ce-gagarin-public-service";
const char* mqtt_server = "mqtt.cetools.org";
const int   mqtt_port   = 1884;
const char* mqtt_user   = "student";
const char* mqtt_pass   = "ce2021-mqtt-forget-whale";

bool IS_BOARD_A = true; 

String myID     = IS_BOARD_A ? "Board_A" : "Board_B";
String remoteID = IS_BOARD_A ? "Board_B" : "Board_A";

// ================= GAME STATE =================
float activeWeight[4]      = {0, 0, 0, 0}; 
bool  weStartedThis[4]     = {false, false, false, false}; 
unsigned long startTime[4] = {0, 0, 0, 0};      
unsigned long lastClearTime[4] = {0, 0, 0, 0};  
const unsigned long TIMEOUT_MS  = 7000;         // Max time a target stays active
const unsigned long COOLDOWN_MS = 750;          // Cooldown after a hit is cleared

int fsrToLocalLED[4] = {0, 1, 2, 3}; 

#define NUM_LEDS_PER_STRIP 7
Adafruit_NeoPixel strips[4] = {
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 0, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 1, NEO_GRB + NEO_KHZ800), 
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 2, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 3, NEO_GRB + NEO_KHZ800)
};

// [Enhanced DSP Structure]
struct FSRSensor {
  int pin;
  int offset;
  int maxAdcInWindow; // Used to store the peak value within the sliding window
  float filteredVal;  // Used to store the real-time value after EMA filtering
};
FSRSensor sensors[4] = {{A1, 0, 0, 0.0f}, {A2, 0, 0, 0.0f}, {A3, 0, 0, 0.0f}, {A4, 0, 0, 0.0f}};

const int ADC_MAX_VAL = 630;
const float HIT_THRESHOLD = 4.5; 
const float EMA_ALPHA = 0.3; // [DSP Parameter] Filtering coefficient (0-1), lower is smoother

WiFiClient mkrClient;
PubSubClient client(mkrClient);

// ================= RTOS RESOURCES =================
struct MqttMessage {
  int id;
  float val;
};
QueueHandle_t mqttQueue;
SemaphoreHandle_t ledMutex; 

// ================= FUNCTIONS =================

void triggerLED(int ledIdx, float weight) {
  if (ledIdx < 0 || ledIdx > 3) return;
  
  if (xSemaphoreTake(ledMutex, portMAX_DELAY) == pdTRUE) {
    if (weight < 0.1) {
      strips[ledIdx].clear();
    } else {
      // Map weight to LED color (Blue to Red)
      float constrainedWeight = constrain(weight, 4.5, 10.0);
      uint16_t hue = map(constrainedWeight * 100, 450, 1000, 21845, 0); 
      strips[ledIdx].fill(strips[ledIdx].ColorHSV(hue, 255, 255));
    }
    strips[ledIdx].show();
    xSemaphoreGive(ledMutex);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  
  // Extract ID from the end of the topic
  int receivedID = String(topic).substring(String(topic).lastIndexOf('/') + 1).toInt();
  float receivedWeight = msg.toFloat();

  // Mapping logic for specific hardware orientation
  int targetLED = receivedID;
  if (receivedID == 0) targetLED = 2;      
  else if (receivedID == 2) targetLED = 0;  

  if (receivedWeight > 0) {
    activeWeight[targetLED] = receivedWeight;
    weStartedThis[targetLED] = false; // Triggered by remote board
    startTime[targetLED] = millis(); 
    triggerLED(targetLED, receivedWeight);
  } else {
    activeWeight[targetLED] = 0;
    lastClearTime[targetLED] = millis(); 
    triggerLED(targetLED, 0);
  }
}

// ================= RTOS TASK 1: SENSING & BUSINESS LOGIC =================
void TaskSensing(void *pvParameters) {
  unsigned long lastWindowTime = millis();
  
  for (;;) {
    unsigned long currentMillis = millis();

    // [DSP Optimization: EMA Filtering + Peak Capture]
    for(int i = 0; i < 4; i++) {
      int rawVal = analogRead(sensors[i].pin);
      
      // EMA Formula: St = α * Xt + (1 - α) * St-1
      sensors[i].filteredVal = (EMA_ALPHA * (float)rawVal) + ((1.0f - EMA_ALPHA) * sensors[i].filteredVal);
      
      // Update the maximum value found in the current sliding window
      if (sensors[i].filteredVal > (float)sensors[i].maxAdcInWindow) {
        sensors[i].maxAdcInWindow = (int)sensors[i].filteredVal;
      }
    }

    // 1. Timeout Detection (Logic remains unchanged)
    for(int i = 0; i < 4; i++) {
      if (activeWeight[i] > 0 && (currentMillis - startTime[i] >= TIMEOUT_MS)) {
        activeWeight[i] = 0;
        lastClearTime[i] = currentMillis; 
        triggerLED(i, 0); 
        if (weStartedThis[i]) {
          MqttMessage msg = {i, 0.0};
          xQueueSend(mqttQueue, &msg, 0);
        }
        weStartedThis[i] = false;
      }
    }

    // 2. Sliding Window Resolution Logic (Determines a hit every 700ms)
    if (currentMillis - lastWindowTime >= 700) {
      bool isAnyActive = false;
      for(int j=0; j<4; j++) { if(activeWeight[j] > 0) isAnyActive = true; }

      for(int i = 0; i < 4; i++) {
        // Use the filtered peak value for mapping
        int peak = constrain(sensors[i].maxAdcInWindow, sensors[i].offset, ADC_MAX_VAL);
        float weight = map(peak, sensors[i].offset, ADC_MAX_VAL, 0, 10000) / 1000.0f;
        int ledIdx = fsrToLocalLED[i];

        if (weight >= HIT_THRESHOLD) {
          if (activeWeight[ledIdx] == 0) {
            // Logic to start a hit (send to the other board)
            if (!isAnyActive && (currentMillis - lastClearTime[ledIdx] >= COOLDOWN_MS)) {
              activeWeight[ledIdx] = weight;
              weStartedThis[ledIdx] = true;
              startTime[ledIdx] = currentMillis; 
              triggerLED(ledIdx, weight);
              MqttMessage msg = {ledIdx, weight};
              xQueueSend(mqttQueue, &msg, 0);
            }
          } 
          else if (!weStartedThis[ledIdx]) {
            // Logic to clear a hit received from the other board
            if (weight >= activeWeight[ledIdx]) {
              activeWeight[ledIdx] = 0;
              weStartedThis[ledIdx] = false;
              lastClearTime[ledIdx] = currentMillis; 
              triggerLED(ledIdx, 0);
              MqttMessage msg = {ledIdx, 0.0};
              xQueueSend(mqttQueue, &msg, 0);
            }
          }
        }
        // Reset window peak for the next sliding window cycle
        sensors[i].maxAdcInWindow = 0; 
      }
      lastWindowTime = currentMillis;
    }
    
    vTaskDelay(pdMS_TO_TICKS(20)); // Increase sampling frequency to 50Hz for EMA requirements
  }
}

// ================= RTOS TASK 2: NETWORK TASK =================
void TaskNetwork(void *pvParameters) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!client.connected()) {
        if (client.connect(("Boxing-" + myID).c_str(), mqtt_user, mqtt_pass)) {
          client.subscribe(("student/boxing/" + remoteID + "/#").c_str());
        }
      }
      client.loop();
    }

    // Process outgoing MQTT messages from the queue
    MqttMessage outMsg;
    if (xQueueReceive(mqttQueue, &outMsg, 0) == pdPASS) {
      String topic = "student/boxing/" + myID + "/sensor/" + String(outMsg.id);
      String payload = String(outMsg.val, 2);
      client.publish(topic.c_str(), payload.c_str());
    }
    
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
}

void setup() {
  Serial.begin(115200);

  // 1. Initialize LED strips and perform calibration
  for(int i = 0; i < 4; i++) {
    strips[i].begin();
    strips[i].setBrightness(50);
    strips[i].show();
    long sum = 0;
    for(int j = 0; j < 30; j++) sum += analogRead(sensors[i].pin);
    sensors[i].offset = (int)(sum / 30);
    sensors[i].filteredVal = (float)sensors[i].offset; // [DSP Initialization]
  }

  // 2. Network connection (WiFi.begin logic simplified)
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // 3. Create RTOS Resources
  mqttQueue = xQueueCreate(10, sizeof(MqttMessage));
  ledMutex  = xSemaphoreCreateMutex();

  // Start Tasks
  xTaskCreate(TaskSensing, "Sensing", 512, NULL, 2, NULL); 
  xTaskCreate(TaskNetwork, "Network", 1024, NULL, 2, NULL); 

  vTaskStartScheduler();
}

void loop() {
  // FreeRTOS takes over control; loop remains empty
}
