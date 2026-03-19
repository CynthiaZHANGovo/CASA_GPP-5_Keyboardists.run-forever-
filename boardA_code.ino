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
const unsigned long TIMEOUT_MS  = 7000;         
const unsigned long COOLDOWN_MS = 750;          

int fsrToLocalLED[4] = {0, 1, 2, 3}; 

#define NUM_LEDS_PER_STRIP 7
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
FSRSensor sensors[4] = {{A1, 0, 0}, {A2, 0, 0}, {A3, 0, 0}, {A4, 0, 0}};

const int ADC_MAX_VAL = 630;
const float HIT_THRESHOLD = 4.5; 

WiFiClient mkrClient;
PubSubClient client(mkrClient);

// ================= RTOS 资源 =================
struct MqttMessage {
  int id;
  float val;
};
QueueHandle_t mqttQueue;
SemaphoreHandle_t ledMutex; // 互斥锁：保护灯带操作不被中断

// ================= FUNCTIONS =================

void triggerLED(int ledIdx, float weight) {
  if (ledIdx < 0 || ledIdx > 3) return;
  
  // 获取互斥锁，确保操作灯带时不被打断
  if (xSemaphoreTake(ledMutex, portMAX_DELAY) == pdTRUE) {
    if (weight < 0.1) strips[ledIdx].clear();
    else {
      float constrainedWeight = constrain(weight, 4.5, 10.0);
      uint16_t hue = map(constrainedWeight * 100, 450, 1000, 21845, 0); 
      strips[ledIdx].fill(strips[ledIdx].ColorHSV(hue, 255, 255));
    }
    strips[ledIdx].show();
    xSemaphoreGive(ledMutex); // 释放锁
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
    triggerLED(targetLED, receivedWeight);
  } else {
    activeWeight[targetLED] = 0;
    lastClearTime[targetLED] = millis(); 
    triggerLED(targetLED, 0);
  }
}

// ================= RTOS 任务 1: 采样与业务逻辑 =================
void TaskSensing(void *pvParameters) {
  unsigned long lastWindowTime = millis();
  
  for (;;) {
    unsigned long currentMillis = millis();

    for(int i = 0; i < 4; i++) {
      int val = analogRead(sensors[i].pin);
      if (val > sensors[i].maxAdcInWindow) sensors[i].maxAdcInWindow = val;
    }

    // 超时检测
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

    // 结算逻辑
    if (currentMillis - lastWindowTime >= 700) {
      bool isAnyActive = false;
      for(int j=0; j<4; j++) { if(activeWeight[j] > 0) isAnyActive = true; }

      for(int i = 0; i < 4; i++) {
        int peak = constrain(sensors[i].maxAdcInWindow, sensors[i].offset, ADC_MAX_VAL);
        float weight = map(peak, sensors[i].offset, ADC_MAX_VAL, 0, 10000) / 1000.0;
        int ledIdx = fsrToLocalLED[i];

        if (weight >= HIT_THRESHOLD) {
          if (activeWeight[ledIdx] == 0) {
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
        sensors[i].maxAdcInWindow = 0; 
      }
      lastWindowTime = currentMillis;
    }
    // 关键：给 WiFi 留出呼吸空间
    vTaskDelay(pdMS_TO_TICKS(25)); 
  }
}

// ================= RTOS 任务 2: 网络任务 =================
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

    // 处理发送队列
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

  // 1. 初始化灯带和校准
  for(int i = 0; i < 4; i++) {
    strips[i].begin();
    strips[i].setBrightness(50);
    strips[i].show();
    long sum = 0;
    for(int j = 0; j < 30; j++) sum += analogRead(sensors[i].pin);
    sensors[i].offset = sum / 30;
  }

  // 2. 先连 WiFi，再开 RTOS
  Serial.print("Connecting WiFi...");
  WiFi.begin(ssid, password);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    // 开机闪烁
    for(int i=0; i<4; i++) strips[i].fill(strips[i].Color(100,100,100));
    for(int i=0; i<4; i++) strips[i].show();
    delay(1000);
    for(int i=0; i<4; i++) strips[i].clear();
    for(int i=0; i<4; i++) strips[i].show();
  }

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // 3. 创建 RTOS 资源
  mqttQueue = xQueueCreate(10, sizeof(MqttMessage));
  ledMutex  = xSemaphoreCreateMutex();

  // 网络任务优先级设为 2，采样设为 2，让它们平等竞争 CPU
  xTaskCreate(TaskSensing, "Sensing", 512, NULL, 2, NULL); 
  xTaskCreate(TaskNetwork, "Network", 1024, NULL, 2, NULL); 

  vTaskStartScheduler();
}

void loop() {}
