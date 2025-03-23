// Code for Sparkfun GT-NUCL1633 fingerprint sensor

#include <SoftwareSerial.h>
#include <Servo.h>
#include <LowPower.h>  // For low power sleep

#define SENSOR_RX_PIN 6      // Sensor TX → Arduino RX
#define SENSOR_TX_PIN 7      // Sensor RX → Arduino TX
#define SERVO_PIN 9          // Servo control pin
#define TOUCH_PIN 2          // Interrupt-capable pin for sensor touch signal
#define SENSOR_POWER_PIN 10  // This pin controls the high side switch transistor

SoftwareSerial fingerSerial(SENSOR_RX_PIN, SENSOR_TX_PIN); // RX, TX
Servo doorServo; // Servo object

// Packet definitions
#define START_CODE 0xF5
#define END_CODE   0xF5

// Command codes
#define CMD_OPEN         0xA0
#define CMD_IS_PRESS     0xB5
#define CMD_IDENTIFY     0x0C
#define CMD_LED_CONTROL  0xB4  // LED control command

// ACK codes
#define ACK_SUCCESS                0x00
#define ACK_FINGER_IS_NOT_PRESSED  0xB1

// Timeout for sensor responses (milliseconds)
unsigned long responseTimeout = 2000;

byte computeChecksum(byte *data, int len) {
  byte cs = 0;
  for (int i = 0; i < len; i++) {
    cs ^= data[i];
  }
  return cs;
}

bool sendCommand(byte cmd, byte p1, byte p2, byte p3, byte p4,
                 byte* responseBuffer = NULL, bool suppressErrors = false) {
  byte packet[8];
  packet[0] = START_CODE;
  packet[1] = cmd;
  packet[2] = p1;
  packet[3] = p2;
  packet[4] = p3;
  packet[5] = p4;
  packet[6] = computeChecksum(&packet[1], 5);
  packet[7] = END_CODE;
  
  // Flush any existing data in the buffer.
  while (fingerSerial.available()) {
    fingerSerial.read();
  }
  
  fingerSerial.write(packet, 8);
  fingerSerial.setTimeout(responseTimeout);
  byte resp[8] = {0};
  int count = fingerSerial.readBytes(resp, 8);
  if (count != 8) return false;
  if (resp[0] != START_CODE || resp[7] != END_CODE) return false;
  byte cs = computeChecksum(&resp[1], 5);
  if (cs != resp[6]) return false;
  
  if (responseBuffer != NULL) {
    for (int i = 0; i < 8; i++) {
      responseBuffer[i] = resp[i];
    }
  }
  
  if (cmd == CMD_IS_PRESS) {
    if (resp[4] != ACK_SUCCESS && resp[4] != ACK_FINGER_IS_NOT_PRESSED) return false;
  } else {
    if (resp[4] != ACK_SUCCESS) return false;
  }
  
  return true;
}

bool setSensorLED(bool on) {
  byte param = on ? 0 : 1;  // 0 = LED on, 1 = LED off.
  return sendCommand(CMD_LED_CONTROL, param, 0, 0, 0);
}

bool isFingerPressed() {
  byte resp[8];
  if (!sendCommand(CMD_IS_PRESS, 0, 0, 0, 0, resp, true)) return false;
  return (resp[2] == 1);
}

uint16_t identifyFingerprint() {
  byte resp[8];
  if (!sendCommand(CMD_IDENTIFY, 0, 0, 0, 0, resp)) return 0;
  uint16_t id = (resp[2] << 8) | resp[3];
  return id;
}

// A flag set by the interrupt when a touch is detected.
volatile bool touchDetected = false;

// Interrupt service routine for the touch pin.
void touchISR() {
  touchDetected = true;
}

void setup() {
  Serial.begin(115200);
  fingerSerial.begin(115200);
  
  Serial.println("System starting up...");
  
  // Configure touch pin as input with internal pullup.
  pinMode(TOUCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TOUCH_PIN), touchISR, RISING);
  
  // Configure SENSOR_POWER_PIN as output and ensure sensor is off initially.
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, LOW);
  
  doorServo.attach(SERVO_PIN);
  doorServo.write(0);
  doorServo.detach();
  
  Serial.println("System initialized. Waiting for touch...");
}

void loop() {
  // Sleep until a touch event wakes the MCU.
  if (!touchDetected) {
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
  
  // Clear the flag.
  touchDetected = false;
  
  Serial.println("Touch detected! Powering on sensor...");
  // Power on the sensor MCU using the high side switch.
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  
  // Allow time for the sensor's MCU to boot up.
  delay(500);
  
  // Initialize the sensor (for example, send CMD_OPEN).
  if (!sendCommand(CMD_OPEN, 0, 0, 0, 0)) {
    Serial.println("Sensor initialization failed!");
  } else {
    Serial.println("Sensor initialized.");
  }
  
  // Turn the sensor LED on immediately.
  if (setSensorLED(true)) {
    Serial.println("Sensor LED turned on.");
  } else {
    Serial.println("Failed to turn sensor LED on.");
  }
  
  // Attempt to identify the fingerprint for 30 seconds.
  unsigned long startTime = millis();
  bool identified = false;
  uint16_t id = 0;
  
  Serial.println("Attempting fingerprint identification for 30 seconds...");
  while (millis() - startTime < 30000) {  // 30 seconds timeout
    if (isFingerPressed()) {
      id = identifyFingerprint();
      if (id != 0) {
        Serial.print("Fingerprint identified. ID: ");
        Serial.println(id);
        identified = true;
        doorServo.attach(SERVO_PIN);
        doorServo.write(90);
        delay(2000);
        doorServo.write(0);
        delay(1000);
        doorServo.detach();
        break;
      } else {
        Serial.println("Fingerprint not recognized. Retrying...");
      }
    } else {
      Serial.println("No finger detected. Waiting...");
    }
    delay(500);  // Poll every 500ms.
  }
  
  if (!identified) {
    Serial.println("30 seconds elapsed without successful identification.");
  }
  
  // Turn the sensor LED off.
  if (setSensorLED(false)) {
    Serial.println("Sensor LED turned off.");
  } else {
    Serial.println("Failed to turn sensor LED off.");
  }
  
  // Wait until the finger is removed before powering down.
  Serial.println("Waiting for finger removal...");
  while (isFingerPressed()) {
    delay(100);
  }
  
  digitalWrite(SENSOR_POWER_PIN, LOW);
  Serial.println("Sensor powered off. Waiting for next touch...\n");
}
