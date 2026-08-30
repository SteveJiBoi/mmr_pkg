#include <WiFi.h>
#include <AsyncUDP.h>
#include <math.h>
#define PI 3.14159265359
const char* ssid     = "test";
const char* password = "steve123";
const int   PORT     = 1234;


const int dirPin[3]   = {19, 25, 14};
const int brakePin[3] = {22, 33, 27};   // HIGH = run, LOW = stop (flip if won't stop)
const int speedPin[3] = {23, 26, 12};
const float wheelDeg[3] = {60, 180, 300};
const int PWM_FREQ = 20000;

const int PWM_RES  = 8;
const unsigned long DEADMAN_MS = 400;
AsyncUDP udp;
volatile int   inAngle = 0, inSpeed = 0, inRot = 0;
volatile unsigned long lastPacket = 0;
void pwm(int pin, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);        // core 3.x: write the PIN, not the channel
#endif
}
void setMotor(int i, int power) {        // -255..255
  digitalWrite(brakePin[i], HIGH);
  digitalWrite(dirPin[i], power >= 0 ? LOW : HIGH);
  pwm(speedPin[i], constrain(abs(power), 0, 255));
}
void stopAll() {
  for (int i = 0; i < 3; i++) { pwm(speedPin[i], 0); digitalWrite(brakePin[i], LOW); }
}
void drive(int angleDeg, int spd, int rot) {
  float theta = (180 - angleDeg) * PI / 180.0;      // (180 - a) fixes left/right
  for (int i = 0; i < 3; i++) {
    float wa = wheelDeg[i] * PI / 180.0;
    int power = int(spd * cos(theta - wa)) - rot;   // - rot fixes rotation
    setMotor(i, constrain(power, -255, 255));
  }
}
void parsePacket(AsyncUDPPacket& packet) {

  char buf[64];

  int n = min(packet.length(), (size_t)63);
  memcpy(buf, packet.data(), n);
  buf[n] = '\0';

  // Print every received packet
  Serial.print("================================\n");
  Serial.print("PACKET RECEIVED\n");
  Serial.print("Data: ");
  Serial.println(buf);

  Serial.print("Length: ");
  Serial.print(packet.length());
  Serial.println(" bytes");

  Serial.print("From: ");
  Serial.print(packet.remoteIP());
  Serial.print(":");
  Serial.println(packet.remotePort());

  // PING
  if (strncmp(buf, "PING", 4) == 0) {

    Serial.println("Type: PING");
    Serial.println("Sending: PONG");

    packet.printf("PONG");

    return;
  }

  // Command packet
  char m;
  int a, s, r;

  if (sscanf(buf, "%c,%d,%d,%d", &m, &a, &s, &r) == 4) {

    inAngle = a;
    inSpeed = s;
    inRot = r;

    lastPacket = millis();

    Serial.println("Type: MOTOR COMMAND");

    Serial.print("Mode: ");
    Serial.println(m);

    Serial.print("Angle: ");
    Serial.println(a);

    Serial.print("Speed: ");
    Serial.println(s);

    Serial.print("Rotation: ");
    Serial.println(r);

    packet.printf("ACK %d %d %d", a, s, r);

  } else {

    Serial.println("Type: UNKNOWN");
    Serial.println("Sending: ERR");

    packet.printf("ERR");
  }

  Serial.println("================================");
}
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++) {
    pinMode(dirPin[i], OUTPUT);
    pinMode(brakePin[i], OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(speedPin[i], PWM_FREQ, PWM_RES);
#endif
  }
  stopAll();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi failed");
    while (1) delay(1000);
  }
  Serial.print("Ready. IP: ");
  Serial.println(WiFi.localIP());   // <-- put this IP in ESP32_IP on the Python side
  if (udp.listen(PORT)) {
    Serial.printf("UDP listening on port %d\n", PORT);
    udp.onPacket([](AsyncUDPPacket packet) { parsePacket(packet); });
  }
}
void loop() {
  if (millis() - lastPacket > DEADMAN_MS) {          // no fresh command -> stop
    stopAll();
  } else if (inSpeed == 0 && inRot == 0) {
    stopAll();
  } else {
    drive(inAngle, inSpeed, inRot);
  }
  delay(10);
}