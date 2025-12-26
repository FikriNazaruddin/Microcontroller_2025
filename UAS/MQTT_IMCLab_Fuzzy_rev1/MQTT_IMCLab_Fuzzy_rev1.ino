/*************************************************
 * IMCLab - Fuzzy Adaptive PID Motor Control
 * RPM Filter + RPM Average + MQTT Logging
 *************************************************/

#include <WiFi.h>
#include <PubSubClient.h>

/* ===== PIN CONFIG ===== */
#define MOTOR_PIN1 27
#define MOTOR_PIN2 26
#define ENABLE_PIN 12
#define RPM_PIN    13

/* ===== RPM CONFIG ===== */
#define PPR 12
#define SAMPLE_TIME 0.1   // 100 ms

/* ===== FILTER CONFIG ===== */
#define RPM_ALPHA 0.3

/* ===== FUZZY WINDOW ===== */
#define AVG_WINDOW_TIME 3000   // ms
const int MAX_SAMPLES = AVG_WINDOW_TIME / 100;

/* ===== WIFI ===== */
const char* ssid = "vivo";
const char* password = "12341234";

/* ===== MQTT ===== */
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char* topic_sub = "esp32/motor/cmd";
const char* topic_pub = "esp32/motor/status";
const char* topic_log = "esp32/motor/log";

WiFiClient espClient;
PubSubClient client(espClient);

/* ===== PID ===== */
float Kp = 0.6;
float Ki = 0.35;
float Kd = 0.05;

/* ===== FUZZY STEP ===== */
#define KP_STEP 0.03
#define KI_STEP 0.02

/* ===== CONTROL ===== */
float setpointRPM = 400;
float error = 0, prevError = 0;
float integral = 0;
float controlPWM = 0;

/* ===== RPM ===== */
volatile int pulseCount = 0;
float rpmRaw = 0;
float rpmFiltered = 0;

/* ===== RPM AVERAGE ===== */
float rpmBuffer[MAX_SAMPLES];
int rpmIndex = 0;
int rpmCount = 0;
float rpmAvg = 0;

/* ===== TIMER ===== */
unsigned long lastControl = 0;
unsigned long lastFuzzy   = 0;
unsigned long lastLog     = 0;

/* ===== INTERRUPT ===== */
void IRAM_ATTR rpmISR() {
  pulseCount++;
}

/* ===== RPM RAW ===== */
float getRPM() {
  noInterrupts();
  int pulses = pulseCount;
  pulseCount = 0;
  interrupts();

  return (pulses / (float)PPR) * (60.0 / SAMPLE_TIME);
}

/* ===== RPM FILTER ===== */
float filterRPM(float raw) {
  rpmFiltered = RPM_ALPHA * raw + (1.0 - RPM_ALPHA) * rpmFiltered;
  return rpmFiltered;
}

/* ===== UPDATE RPM AVERAGE ===== */
void updateRPMAverage(float rpm) {
  rpmBuffer[rpmIndex] = rpm;
  rpmIndex = (rpmIndex + 1) % MAX_SAMPLES;

  if (rpmCount < MAX_SAMPLES) rpmCount++;

  float sum = 0;
  for (int i = 0; i < rpmCount; i++) sum += rpmBuffer[i];
  rpmAvg = sum / rpmCount;
}

/* ===== FUZZY ===== */
void fuzzyAdjustPID(float avgError, float avgDError) {

  if (abs(avgError) < 15) return;

  if (abs(avgError) > 80) {
    Kp += KP_STEP;
    Ki += KI_STEP;
  }
  else if (abs(avgError) < 40 && abs(avgDError) < 20) {
    Kp -= KP_STEP * 0.5;
    Ki -= KI_STEP * 0.5;
  }

  Kp = constrain(Kp, 0.4, 2.0);
  Ki = constrain(Ki, 0.05, 1.5);
}

/* ===== MQTT CALLBACK ===== */
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  if (msg.startsWith("SET")) {
    setpointRPM = msg.substring(3).toFloat();
  }
}

/* ===== MQTT RECONNECT ===== */
void reconnect() {
  while (!client.connected()) {
    if (client.connect("IMCLabClient")) {
      client.subscribe(topic_sub);
    } else {
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(RPM_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmISR, RISING);

  digitalWrite(MOTOR_PIN1, HIGH);
  digitalWrite(MOTOR_PIN2, LOW);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  rpmFiltered = 0;

  Serial.println("IMCLab PID + Fuzzy + MQTT Logging Started");
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long now = millis();

  /* ===== CONTROL LOOP ===== */
  if (now - lastControl >= SAMPLE_TIME * 1000) {
    lastControl = now;

    rpmRaw = getRPM();
    rpmFiltered = filterRPM(rpmRaw);
    updateRPMAverage(rpmFiltered);

    error = setpointRPM - rpmFiltered;
    float dError = error - prevError;

    integral += error * SAMPLE_TIME;
    integral = constrain(integral, -300, 300);

    float derivative = dError / SAMPLE_TIME;

    controlPWM = (Kp * error) + (Ki * integral) + (Kd * derivative);
    controlPWM = constrain(controlPWM, 0, 255);

    analogWrite(ENABLE_PIN, controlPWM);
    prevError = error;
  }

  /* ===== FUZZY LOOP ===== */
  if (now - lastFuzzy >= AVG_WINDOW_TIME && rpmCount >= MAX_SAMPLES) {
    lastFuzzy = now;

    static float prevAvgError = 0;
    float avgError = setpointRPM - rpmAvg;
    float avgDError = avgError - prevAvgError;

    fuzzyAdjustPID(avgError, avgDError);
    prevAvgError = avgError;
  }

  /* ===== MQTT LOGGING (UNTUK GRAFIK) ===== */
  if (now - lastLog >= 500) {
    lastLog = now;

    String logData =
      String(now) + "," +
      String(setpointRPM, 1) + "," +
      String(rpmRaw, 1) + "," +
      String(rpmFiltered, 1) + "," +
      String(rpmAvg, 1) + "," +
      String(controlPWM, 0);

    client.publish(topic_log, logData.c_str());
  }
}
