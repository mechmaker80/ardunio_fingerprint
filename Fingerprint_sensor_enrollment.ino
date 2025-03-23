#include <SoftwareSerial.h>

#define RX_PIN 6  // Connect to TX of fingerprint sensor
#define TX_PIN 7  // Connect to RX of fingerprint sensor

SoftwareSerial fingerprintSerial(RX_PIN, TX_PIN); // Using SoftwareSerial on Arduino Uno

// Function to send command packet to fingerprint sensor
bool sendCommand(byte command, byte p1 = 0, byte p2 = 0, byte p3 = 0, byte p4 = 0) {
    byte checksum = command ^ p1 ^ p2 ^ p3 ^ p4;
    byte packet[8] = {0xF5, command, p1, p2, p3, p4, checksum, 0xF5};
    
    fingerprintSerial.write(packet, sizeof(packet));
    delay(100); // Give time for the sensor to process

    // Read ACK response
    if (fingerprintSerial.available()) {
        byte response[8];
        fingerprintSerial.readBytes(response, 8);
        return (response[4] == 0x00); // 0x00 means ACK_SUCCESS
    }
    return false; // No response from sensor
}

// Function to check if fingerprint sensor is detected
bool checkSensor() {
    Serial.println("Checking fingerprint sensor...");
    if (!sendCommand(0xA0)) {  // Open sensor command
        Serial.println("ERROR: Fingerprint sensor NOT detected!");
        return false;
    }
    Serial.println("Fingerprint sensor detected successfully!");
    return true;
}

// Function to get available ID for new fingerprint enrollment
int getEntryID() {
    Serial.println("Requesting available User ID...");
    sendCommand(0x0D);  // Get Entry ID command
    delay(500);

    if (fingerprintSerial.available()) {
        byte response[8];
        fingerprintSerial.readBytes(response, 8);
        if (response[1] == 0x0D) {  // Check if response is for Entry ID
            return (response[2] << 8) | response[3];  // Combine high and low bytes
        }
    }
    return -1;  // No available ID
}

// Function to enroll a fingerprint
void enrollFingerprint() {
    int userID = getEntryID();
    if (userID == -1) {
        Serial.println("No available ID for enrollment.");
        return;
    }

    Serial.print("Enrolling at ID: ");
    Serial.println(userID);

    // Step 1: Send initial enroll command
    if (!sendCommand(0x01, userID >> 8, userID & 0xFF, 0, 0)) {
        Serial.println("Failed to start enrollment.");
        return;
    }

    // Step 2: Repeat enroll command 8 times as required by the sensor
    for (int i = 0; i < 8; i++) {
        Serial.print("Place your finger... Step ");
        Serial.println(i + 1);
        delay(3000);

        if (!sendCommand(0x01, 0, 0, 0, 0)) {
            Serial.println("Failed during enrollment.");
            return;
        }

        byte response[8];
        fingerprintSerial.readBytes(response, 8);
        if (response[1] == 0x03) {  // 0x03 means enrollment completed
            Serial.println("Fingerprint enrolled successfully!");
            return;
        }
    }
    Serial.println("Enrollment failed.");
}

// Function to verify a fingerprint (1:N identification)
void verifyFingerprint() {
    Serial.println("Place your finger on the sensor...");
    if (!sendCommand(0x0C)) { // Identify command
        Serial.println("Error sending Identify command.");
        return;
    }

    delay(3000);  // Allow processing time

    if (fingerprintSerial.available()) {
        byte response[8];
        fingerprintSerial.readBytes(response, 8);
        if (response[1] == 0x0C) { // Correct response from sensor
            int userID = (response[2] << 8) | response[3];
            if (userID != 0) {
                Serial.print("Fingerprint matched with ID: ");
                Serial.println(userID);
            } else {
                Serial.println("No match found.");
            }
        } else {
            Serial.println("Invalid response.");
        }
    } else {
        Serial.println("No response from fingerprint sensor.");
    }
}

// Arduino Setup Function
void setup() {
    Serial.begin(115200);  
    fingerprintSerial.begin(115200);  

    Serial.println("Initializing Fingerprint Sensor...");
    if (!checkSensor()) {
        while (1);  // Stop execution if sensor is not detected
    }
}

// Arduino Main Loop Function
void loop() {
    Serial.println("\nChoose an option:");
    Serial.println("1. Enroll Fingerprint");
    Serial.println("2. Verify Fingerprint");

    while (!Serial.available()); // Wait for user input

    String input = Serial.readStringUntil('\n');  // Read full input line
    input.trim(); // Remove extra spaces or newlines

    if (input == "1") {
        enrollFingerprint();
    } else if (input == "2") {
        verifyFingerprint();
    } else {
        Serial.println("Invalid choice. Try again.");
    }
}