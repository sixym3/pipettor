// Arduino RS485 Receiver using Arduino RS485 Library
// Install library: Tools -> Manage Libraries -> Search "RS485" -> Install "RS485" by Arduino
// Note: Cannot use Serial Monitor as RS485 uses pins 0 (RX) and 1 (TX)

#include <ArduinoRS485.h>
#include <Adafruit_NeoPixel.h>

#define PIN 12
#define NUMPIXELS 7

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500

#define LED_BUILTIN 13      // Built-in LED pin

// Timing constants
#define STARTUP_FLASH_DELAY 50     // Fast flash on startup
#define HEARTBEAT_INTERVAL 5000    // Heartbeat every 5 seconds
#define HEARTBEAT_DURATION 50      // Duration of heartbeat pulse

// Global variables
unsigned long lastHeartbeat = 0;
String receiveBuffer = "";
bool builtInLEDState = false;
unsigned long startTime = 0;

// New global variables for NeoPixel state
uint8_t lastR = 0, lastG = 0, lastB = 0;
uint8_t lastBrightness = 50; // Default brightness

// Actuator control variables
int plungerPosition = 0;    // Current plunger position in mm
int tipEjectPosition = 0;   // Current tip eject position in mm
int plungerMaxTravel = 10;  // Maximum plunger travel in mm
int tipEjectMaxTravel = 5;  // Maximum tip eject travel in mm

// Pin definitions for actuators (example pins - adjust for your hardware)
#define PLUNGER_STEP_PIN 2
#define PLUNGER_DIR_PIN 3
#define PLUNGER_ENABLE_PIN 4
#define TIP_EJECT_STEP_PIN 5
#define TIP_EJECT_DIR_PIN 6
#define TIP_EJECT_ENABLE_PIN 7

// Steps per mm for actuators (adjust based on your stepper motor setup)
#define STEPS_PER_MM 200  // Example: 200 steps per mm

// Function prototypes
void setLEDColor(uint8_t r, uint8_t g, uint8_t b);
void setLEDBrightness(uint8_t brightness);
void turnOffLEDs();
void turnOnLEDs();
String getHeartbeatStatus();

void setup() {
  // Initialize LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Flash LED 10 times quickly to indicate ready
  for(int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(STARTUP_FLASH_DELAY);
    digitalWrite(LED_BUILTIN, LOW);
    delay(STARTUP_FLASH_DELAY);
  }
  
  // Start RS485 communication
  // Uses hardware serial (pins 0 and 1) at 115200 baud
  RS485.begin(115200);
  
  // Set receive mode
  RS485.receive();
  
  // Send startup message
  sendMessage("Arduino RS485 Receiver Ready");
  sendMessage("Using pins " + String(0) + " (RX) and " + String(1) + " (TX)");
  sendMessage("Heartbeat interval: " + String(HEARTBEAT_INTERVAL / 1000) + " seconds");
  
  // Record start time
  startTime = millis();
  pixels.begin();
  setLEDBrightness(lastBrightness);
  setLEDColor(0, 0, 0); // Start off
}

void loop() {
  // Check for incoming RS485 data
  while(RS485.available()) {
    char c = RS485.read();
    
    // Build command string until newline
    if(c == '\n' || c == '\r') {
      if(receiveBuffer.length() > 0) {
        // Process the received command
        processCommand(receiveBuffer);
        receiveBuffer = "";  // Clear buffer
      }
    } else {
      receiveBuffer += c;
    }
  }
  
  // Heartbeat - send message every 5 seconds
  unsigned long currentMillis = millis();
  if(currentMillis - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    lastHeartbeat = currentMillis;
    
    // Send heartbeat message
    sendMessage(getHeartbeatStatus());
    
    // Brief LED pulse for heartbeat
    if(!builtInLEDState) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(HEARTBEAT_DURATION);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

void processCommand(String command) {
  // Trim whitespace
  command.trim();
  
  // Echo the received command
  sendMessage("Received command: " + command);

  // Command handling for LED strip control
  if (command.startsWith("SETCOLOR")) {
    int r, g, b;
    int n = sscanf(command.c_str(), "SETCOLOR %d %d %d", &r, &g, &b);
    if (n == 3 && r >= 0 && r <= 255 && g >= 0 && g <= 255 && b >= 0 && b <= 255) {
      setLEDColor((uint8_t)r, (uint8_t)g, (uint8_t)b);
      sendMessage("Set LED color to R:" + String(r) + " G:" + String(g) + " B:" + String(b));
    } else {
      sendMessage("Invalid SETCOLOR command. Usage: SETCOLOR r g b");
    }
  } else if (command.startsWith("SETBRIGHTNESS")) {
    int brightness;
    int n = sscanf(command.c_str(), "SETBRIGHTNESS %d", &brightness);
    if (n == 1 && brightness >= 0 && brightness <= 255) {
      setLEDBrightness((uint8_t)brightness);
      sendMessage("Set LED brightness to " + String(brightness));
    } else {
      sendMessage("Invalid SETBRIGHTNESS command. Usage: SETBRIGHTNESS x (0-255)");
    }
  } else if (command == "LEDOFF") {
    turnOffLEDs();
    sendMessage("LED strip turned off");
  } else if (command == "LEDON") {
    turnOnLEDs();
    sendMessage("LED strip turned on");
  }
  
  
  // Base Commands
  else if(command == "V" || command == "v") {
    // Toggle Built-in LED
    builtInLEDState = !builtInLEDState;
    digitalWrite(LED_BUILTIN, builtInLEDState);
    sendMessage("Built-in LED toggled - Now: " + String(builtInLEDState ? "ON" : "OFF"));
  } else if(command.startsWith("CMD")) {
    sendMessage("Processing command: " + command);
  } else if(command == "STATUS") {
    // Status command to check system state
    sendMessage(getHeartbeatStatus());
  } else {
    sendMessage("Unknown command: " + command);
  }
}

void sendMessage(String message) {
  RS485.beginTransmission();
  RS485.println(message);
  RS485.endTransmission();
}

// New functions for LED control
void setLEDColor(uint8_t r, uint8_t g, uint8_t b) {
  lastR = r;
  lastG = g;
  lastB = b;
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}

void setLEDBrightness(uint8_t brightness) {
  lastBrightness = brightness;
  pixels.setBrightness(brightness);
  // Re-apply last color to update brightness
  setLEDColor(lastR, lastG, lastB);
}

void turnOffLEDs() {
  setLEDColor(0, 0, 0);
}

void turnOnLEDs() {
  setLEDColor(lastR, lastG, lastB);
}

String getHeartbeatStatus() {
  unsigned long uptime = (millis() - startTime) / 1000;
  return "Status: Built-in LED=" + String(builtInLEDState ? "ON" : "OFF") + ", Uptime=" + String(uptime) + "s";
}