#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <FreeRTOS_SAMD21.h>

// ================= 1. 核心硬件配置文件 =================
bool IS_BOARD_A = false; // Board A 设为 true，Board B 设为 false

uint32_t getLedType(int index) {
  if (IS_BOARD_A) {
    if (index == 1) return NEO_GRBW + NEO_KHZ800; 
    return NEO_GRB + NEO_KHZ800;
  } else {
    // B板：S2 (索引1) 是特殊的 RGBW 型号
    if (index == 1) return NEO_GRBW + NEO_KHZ800; 
    return NEO_GRB + NEO_KHZ800;
  }
}

// ================= 2. 网络配置 =================
const char* ssid     = "CE-Hub-Student";
const char* password = "casa-ce-gagarin-public-service";
const char* mqtt_server = "mqtt.cetools.org";
const int   mqtt_port   = 1884;
const char* mqtt_user   = "student";
const char* mqtt_pass   = "ce2021-mqtt-forget-whale";

String myID     = IS_BOARD_A ? "Board_A" : "Board_B";
String remoteID = IS_BOARD_A ? "Board_B" : "Board_A";

// ================= 3. 硬件状态 =================
#define NUM_LEDS_PER_STRIP 7
Adafruit_NeoPixel strips[4] = {
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 0),
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 1),
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 2),
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 3)
};

float activeWeight[4]      = {0, 0, 0, 0}; 
bool  weStartedThis[4]     = {false, false, false, false}; 
unsigned long startTime[4] = {0, 0, 0, 0};      
unsigned long lastClearTime[4] = {0, 0, 0, 0};  
const unsigned long TIMEOUT_MS  = 7000;         
const unsigned long COOLDOWN_MS = 750;          

struct FSRSensor { int pin; int offset; int maxAdcInWindow; };
FSRSensor sensors[4] = {{A1, 0, 0}, {A2, 0, 0}, {A3, 0, 0}, {A4, 0, 0}};
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

void TaskSensing(void *pvParameters) {
  unsigned long lastWindowTime = millis();
  for (;;) {
    unsigned long currentMillis = millis();
    for(int i = 0; i < 4; i++) {
      int val = analogRead(sensors[i].pin);
      if (val > sensors[i].maxAdcInWindow) sensors[i].maxAdcInWindow = val;
    }

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

    if (currentMillis - lastWindowTime >= 700) {
      bool isAnyActive = false;
      for(int j=0; j<4; j++) { if(activeWeight[j] > 0) isAnyActive = true; }
      for(int i = 0; i < 4; i++) {
        int peak = constrain(sensors[i].maxAdcInWindow, sensors[i].offset, ADC_MAX_VAL);
        float weight = map(peak, sensors[i].offset, ADC_MAX_VAL, 0, 10000) / 1000.0;
        
        Serial.print("S"); Serial.print(i + 1);
        Serial.print(": "); Serial.print(weight, 2);
        Serial.print("kg | ");

        if (weight >= HIT_THRESHOLD) {
          if (activeWeight[i] == 0) {
            if (!isAnyActive && (currentMillis - lastClearTime[i] >= COOLDOWN_MS)) {
              activeWeight[i] = weight; weStartedThis[i] = true;
              startTime[i] = currentMillis; triggerLED(i, weight, true);
              MqttMessage m = {i, weight}; xQueueSend(mqttQueue, &m, 0);
            }
          } else if (!weStartedThis[i]) {
            if (weight >= activeWeight[i]) {
              activeWeight[i] = 0; weStartedThis[i] = false;
              lastClearTime[i] = currentMillis; triggerLED(i, 0, true);
              MqttMessage m = {i, 0.0}; xQueueSend(mqttQueue, &m, 0);
            }
          }
        }
        sensors[i].maxAdcInWindow = 0; 
      }
      Serial.println();
      lastWindowTime = currentMillis;
    }
    vTaskDelay(pdMS_TO_TICKS(40)); 
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

  // 1. WiFi 连接优先
  Serial.print("Connecting WiFi...");
  WiFi.begin(ssid, password);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry++ < 40) {
    delay(500); Serial.print(".");
  }

  // 2. 连上后初始化灯带
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi OK!");
    for(int i = 0; i < 4; i++) {
      strips[i].updateType(getLedType(i)); 
      strips[i].begin();
      strips[i].setBrightness(50);
    }
    
    // --- 恢复开机闪烁功能 ---
    for(int i = 0; i < 4; i++) {
      strips[i].fill(strips[i].Color(120, 120, 120)); // 中等亮度白光
      strips[i].show();
    }
    delay(1000); // 亮 1 秒
    for(int i = 0; i < 4; i++) {
      strips[i].clear();
      strips[i].show();
    }
    // ----------------------
  }

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  for(int i = 0; i < 4; i++) {
    long sum = 0; for(int j = 0; j < 30; j++) sum += analogRead(sensors[i].pin);
    sensors[i].offset = sum / 30;
  }

  mqttQueue = xQueueCreate(10, sizeof(MqttMessage));
  ledMutex  = xSemaphoreCreateMutex();

  xTaskCreate(TaskSensing, "Sens", 512, NULL, 2, NULL);
  xTaskCreate(TaskNetwork, "Net", 1024, NULL, 1, NULL);

  vTaskStartScheduler();
}

void loop() {}
