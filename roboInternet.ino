#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// WiFi credentials
const char* ssid = "";
const char* password = "";

// HiveMQ Cloud credentials
const char* mqtt_server = "";
const int mqtt_port = 8883;
const char* mqtt_user = "";
const char* mqtt_pass = "";

// Inbuilt Blue LED (usually on GPIO 2)
#define LED_BUILTIN 2

WiFiClientSecure espClient;
PubSubClient client(espClient);

// Servo joint structure
struct ServoJoint {
  Servo servo;
  int pin;
  const char* topicName;
  int angle;
};

// Define all joints with initial angles
ServoJoint joints[] = {
  { Servo(), 27 , "base", 90 },
  { Servo(), 26 , "shoulder", 120 },
  { Servo(), 25 , "elbow", 180 },
  { Servo(), 33 , "gripper", 0 }
};

const int numJoints = sizeof(joints) / sizeof(joints[0]);

// Callback when MQTT message is received
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String topicStr = String(topic);
  int angle = atoi((char*)payload);

  for (int i = 0; i < numJoints; i++) {
    String jointTopic = String("robot/control/") + joints[i].topicName;
    if (topicStr == jointTopic) {
      if (angle >= 0 && angle <= 180) {
        joints[i].servo.write(angle);
        joints[i].angle = angle;
        Serial.printf("Moved %s to angle %d\n", joints[i].topicName, angle);
      }
    }
  }
}

// Reconnect MQTT if needed
void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("connected!");
      for (int i = 0; i < numJoints; i++) {
        String topic = "robot/control/" + String(joints[i].topicName);
        client.subscribe(topic.c_str());
        Serial.println("Subscribed to: " + topic);
      }
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Attach and set initial angle
  for (int i = 0; i < numJoints; i++) {
    joints[i].servo.attach(joints[i].pin);
    joints[i].servo.write(joints[i].angle);
    Serial.printf("Initialized %s to angle %d\n", joints[i].topicName, joints[i].angle);
  }

  pinMode(LED_BUILTIN, OUTPUT);

  // WiFi connect with blinking LED
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(300);
    digitalWrite(LED_BUILTIN, LOW);
    delay(300);
    Serial.print(".");
  }

  Serial.println("\nConnected to WiFi");

  // Turn LED ON after connection
  digitalWrite(LED_BUILTIN, HIGH);

  // MQTT setup
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
