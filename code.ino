#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>

// ================= CONFIGURATION =================
const char* ssid     = "CE-Hub-Student";
const char* password = "casa-ce-gagarin-public-service";
const char* mqtt_server = "mqtt.cetools.org";
const int   mqtt_port   = 1884;
const char* mqtt_user   = "student";
const char* mqtt_pass   = "ce2021-mqtt-forget-whale";

bool IS_BOARD_A = false; 

String myID     = IS_BOARD_A ? "Board_A" : "Board_B";
String remoteID = IS_BOARD_A ? "Board_B" : "Board_A";

// ================= GAME STATE & TIMERS =================
float activeWeight[4]      = {0, 0, 0, 0}; // Current active weight for each channel
bool  weStartedThis[4]     = {false, false, false, false}; // Flag to track if local board triggered the event
unsigned long startTime[4] = {0, 0, 0, 0};      // Timestamp when the LED was turned on
unsigned long lastClearTime[4] = {0, 0, 0, 0};  // Timestamp when the LED was last turned off
const unsigned long TIMEOUT_MS  = 7000;         // Auto-turn off after 7 seconds
const unsigned long COOLDOWN_MS = 750;          // Cooldown period before a new hit is accepted

int fsrToLocalLED[4] = {0, 1, 2, 3}; 

#define NUM_LEDS_PER_STRIP 7

// --- CRITICAL FIX: Changed strips[1] from NEO_GRBW to NEO_GRB to match others ---
Adafruit_NeoPixel strips[4] = {
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 0, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(NUM_LEDS_PER_STRIP, 1, NEO_GRB + NEO_KHZ800), // Fixed to standard RGB
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
const float HIT_THRESHOLD = 4.5; // Minimum kg required to register a hit
unsigned long lastWindowTime = 0;
const unsigned long WINDOW_MS = 700;

WiFiClient mkrClient;
PubSubClient client(mkrClient);

// ================= FUNCTIONS =================

void triggerLED(int ledIdx, float weight) {
  if (ledIdx < 0 || ledIdx > 3) return;
  
  if (weight < 0.1) {
    strips[ledIdx].clear();
  } else {
    float constrainedWeight = constrain(weight, 4.5, 10.0);
    // Map hue: from Green (21845) to Red (0) based on weight
    uint16_t hue = map(constrainedWeight * 100, 450, 1000, 21845, 0); 
    strips[ledIdx].fill(strips[ledIdx].ColorHSV(hue, 255, 255));
  }
  strips[ledIdx].show();
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  
  // Extract the sensor ID from the end of the MQTT topic
  int receivedID = String(topic).substring(String(topic).lastIndexOf('/') + 1).toInt();
  float receivedWeight = msg.toFloat();

  // Logic to swap target IDs for spatial mapping between Board A and B
  int targetLED = receivedID;
  if (receivedID == 0) targetLED = 2;      
  else if (receivedID == 2) targetLED = 0; 

  if (receivedWeight > 0) {
    activeWeight[targetLED] = receivedWeight;
    weStartedThis[targetLED] = false; // Triggered by remote, not local
    startTime[targetLED] = millis(); 
    triggerLED(targetLED, receivedWeight);
    Serial.print("\n>>> Remote ON (Swapped): LED "); Serial.println(targetLED);
  } else {
    activeWeight[targetLED] = 0;
    lastClearTime[targetLED] = millis(); 
    triggerLED(targetLED, 0);
    Serial.print("\n>>> Remote OFF (Swapped): LED "); Serial.println(targetLED);
  }
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect(("Boxing-" + myID).c_str(), mqtt_user, mqtt_pass)) {
      client.subscribe(("student/boxing/" + remoteID + "/#").c_str());
      Serial.println("\nMQTT Connected");
    } else { 
      delay(5000); 
    }
  }
}

void setup() {
  Serial.begin(115200);

  for(int i=0; i<4; i++) fsrToLocalLED[i] = i;

  for(int i = 0; i < 4; i++) {
    strips[i].begin();
    strips[i].setBrightness(50);
    
    // Calibrate sensor offset by averaging 30 readings
    long sum = 0;
    for(int j = 0; j < 30; j++) sum += analogRead(sensors[i].pin);
    sensors[i].offset = sum / 30;
    
    strips[i].show();
  }

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { 
    delay(500); 
    Serial.print("."); 
  }

  // Startup sequence: flash all LEDs white
  for(int i = 0; i < 4; i++) { 
    strips[i].fill(strips[i].Color(100, 100, 100)); 
    strips[i].show(); 
  }
  delay(1000);
  for(int i = 0; i < 4; i++) { 
    strips[i].clear(); 
    strips[i].show(); 
  }

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long currentMillis = millis();

  // Continuously monitor FSR pins to find peak value in the window
  for(int i = 0; i < 4; i++) {
    int val = analogRead(sensors[i].pin);
    if (val > sensors[i].maxAdcInWindow) sensors[i].maxAdcInWindow = val;
  }

  // 1. Timeout detection: Automatically clear LED if active too long
  for(int i = 0; i < 4; i++) {
    if (activeWeight[i] > 0 && (currentMillis - startTime[i] >= TIMEOUT_MS)) {
      activeWeight[i] = 0;
      lastClearTime[i] = currentMillis; 
      triggerLED(i, 0); 
      if (weStartedThis[i]) {
        client.publish(("student/boxing/" + myID + "/sensor/" + String(i)).c_str(), "0.00");
      }
      weStartedThis[i] = false;
    }
  }

  // 2. Main Processing Logic (Evaluated every 0.7s)
  if (currentMillis - lastWindowTime >= WINDOW_MS) {
    bool isAnyActive = false;
    for(int j=0; j<4; j++) { 
      if(activeWeight[j] > 0) isAnyActive = true; 
    }

    for(int i = 0; i < 4; i++) {
      int peak = constrain(sensors[i].maxAdcInWindow, sensors[i].offset, ADC_MAX_VAL);
      float weight = map(peak, sensors[i].offset, ADC_MAX_VAL, 0, 10000) / 1000.0;
      int ledIdx = fsrToLocalLED[i];

      // Debug serial output
      Serial.print("S"); Serial.print(i + 1);
      Serial.print(": "); Serial.print(weight, 2);
      Serial.print("kg | ");

      if (weight >= HIT_THRESHOLD) {
        // CASE A: Current LED is idle, try to activate it locally
        if (activeWeight[ledIdx] == 0) {
          if (!isAnyActive && (currentMillis - lastClearTime[ledIdx] >= COOLDOWN_MS)) {
            activeWeight[ledIdx] = weight;
            weStartedThis[ledIdx] = true;
            startTime[ledIdx] = currentMillis; 
            triggerLED(ledIdx, weight);
            client.publish(("student/boxing/" + myID + "/sensor/" + String(ledIdx)).c_str(), String(weight, 2).c_str());
            Serial.print("\n[Local ON] LED "); Serial.println(ledIdx);
          }
        } 
        // CASE B: LED was activated by remote, try to "hit it back" to clear it
        else if (!weStartedThis[ledIdx]) {
          if (weight >= activeWeight[ledIdx]) { // Only clears if local hit is stronger or equal
            activeWeight[ledIdx] = 0;
            weStartedThis[ledIdx] = false;
            lastClearTime[ledIdx] = currentMillis; 
            triggerLED(ledIdx, 0);
            client.publish(("student/boxing/" + myID + "/sensor/" + String(ledIdx)).c_str(), "0.00");
            Serial.print("\n[Local OFF] LED "); Serial.println(ledIdx);
          }
        }
      }
      sensors[i].maxAdcInWindow = 0; // Reset peak value for the next window
    }
    Serial.println(); 
    lastWindowTime = currentMillis;
  }
}
