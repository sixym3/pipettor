// Arduino Pipette Actuator Control with RS485 Communication
// Modified version to use plunger position for green LED scale and tip eject for red LED scale
// Based on the original LED control sketch but extended for actuator control

#include <ArduinoRS485.h>
#include <Adafruit_NeoPixel.h>

// LED Strip Configuration
#define LED_PIN 12
#define NUM_PIXELS 24
Adafruit_NeoPixel pixels(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Built-in LED
#define LED_BUILTIN 13

// Actuator Pin Definitions (adjust for your hardware)
#define PLUNGER_STEP_PIN 2
#define PLUNGER_DIR_PIN 3
#define PLUNGER_ENABLE_PIN 4
#define TIP_EJECT_STEP_PIN 5
#define TIP_EJECT_DIR_PIN 6
#define TIP_EJECT_ENABLE_PIN 7

// Actuator Configuration
#define STEPS_PER_MM 200        // Steps per mm (adjust based on your stepper setup)
#define PLUNGER_MAX_MM 10       // Maximum plunger travel in mm
#define TIP_EJECT_MAX_MM 5      // Maximum tip eject travel in mm
#define ACTUATOR_SPEED_DELAY 2  // Delay between steps in milliseconds

// Timing constants
#define STARTUP_FLASH_DELAY 50
#define HEARTBEAT_INTERVAL 5000
#define HEARTBEAT_DURATION 50

// Global variables
unsigned long lastHeartbeat = 0;
String receiveBuffer = "";
bool builtInLEDState = false;
unsigned long startTime = 0;

// LED state variables
uint8_t lastR = 0, lastG = 0, lastB = 0;
uint8_t lastBrightness = 50;

// Actuator position variables
int plungerPosition = 0;     // Current position in mm
int tipEjectPosition = 0;    // Current position in mm

// Function prototypes
void setLEDColor(uint8_t r, uint8_t g, uint8_t b);
void setLEDBrightness(uint8_t brightness);
void turnOffLEDs();
void turnOnLEDs();
String getHeartbeatStatus();
void updateLEDFromPositions();

// Actuator control functions
void initializeActuators();
void setPlungerPosition(int position_mm);
void setTipEjectPosition(int position_mm);
void moveActuator(int stepPin, int dirPin, int enablePin, int steps, bool direction);
String getActuatorStatus();

void setup() {
  // Initialize built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Flash LED 10 times to indicate ready
  for(int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(STARTUP_FLASH_DELAY);
    digitalWrite(LED_BUILTIN, LOW);
    delay(STARTUP_FLASH_DELAY);
  }
  
  // Initialize actuators
  initializeActuators();
  
  // Initialize LED strip
  pixels.begin();
  setLEDBrightness(lastBrightness);
  updateLEDFromPositions(); // Set initial LED color based on positions
  
  // Start RS485 communication at 115200 baud
  RS485.begin(115200);
  RS485.receive();
  
  // Send startup messages
  sendMessage("Pipette Actuator Control Ready (LED Test Mode)");
  sendMessage("Plunger: 0-" + String(PLUNGER_MAX_MM) + "mm (controls green LED), Tip Eject: 0-" + String(TIP_EJECT_MAX_MM) + "mm (controls red LED)");
  sendMessage("Commands: SETPOSITION <plunger> <tip>, SETPLUNGER <pos>, SETTIPEJECT <pos>, SETCOLOR <r> <g> <b>");
  sendMessage("Heartbeat interval: " + String(HEARTBEAT_INTERVAL / 1000) + " seconds");
  
  startTime = millis();
}

void loop() {
  // Check for incoming RS485 data
  while(RS485.available()) {
    char c = RS485.read();
    
    if(c == '\n' || c == '\r') {
      if(receiveBuffer.length() > 0) {
        processCommand(receiveBuffer);
        receiveBuffer = "";
      }
    } else {
      receiveBuffer += c;
    }
  }
  
  // Heartbeat
  unsigned long currentMillis = millis();
  if(currentMillis - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    lastHeartbeat = currentMillis;
    sendMessage(getHeartbeatStatus());
    
    if(!builtInLEDState) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(HEARTBEAT_DURATION);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

void processCommand(String command) {
  command.trim();
  sendMessage("Received: " + command);

  // Actuator Commands
  if (command.startsWith("SETPOSITION")) {
    int plunger_pos, tip_pos;
    int n = sscanf(command.c_str(), "SETPOSITION %d %d", &plunger_pos, &tip_pos);
    if (n == 2 && plunger_pos >= 0 && plunger_pos <= PLUNGER_MAX_MM && 
        tip_pos >= 0 && tip_pos <= TIP_EJECT_MAX_MM) {
      // Move both actuators
      setPlungerPosition(plunger_pos);
      setTipEjectPosition(tip_pos);
      // Update LED once after both movements
      updateLEDFromPositions();
      
      uint8_t greenValue = map(plunger_pos, 0, PLUNGER_MAX_MM, 0, 255);
      uint8_t redValue = map(tip_pos, 0, TIP_EJECT_MAX_MM, 0, 255);
      sendMessage("Position set - Plunger: " + String(plunger_pos) + "mm, Tip: " + String(tip_pos) + 
                  "mm (LED R:" + String(redValue) + " G:" + String(greenValue) + ")");
    } else {
      sendMessage("Invalid SETPOSITION. Usage: SETPOSITION <plunger_0-" + String(PLUNGER_MAX_MM) + 
                  "> <tip_0-" + String(TIP_EJECT_MAX_MM) + ">");
    }
  }
  else if (command.startsWith("SETPLUNGER")) {
    int position;
    int n = sscanf(command.c_str(), "SETPLUNGER %d", &position);
    if (n == 1 && position >= 0 && position <= PLUNGER_MAX_MM) {
      setPlungerPosition(position);
      updateLEDFromPositions(); // Update LED after position change
      sendMessage("Plunger set to " + String(position) + "mm (Green: " + String(map(position, 0, PLUNGER_MAX_MM, 0, 255)) + ")");
    } else {
      sendMessage("Invalid SETPLUNGER. Usage: SETPLUNGER <0-" + String(PLUNGER_MAX_MM) + ">");
    }
  }
  else if (command.startsWith("SETTIPEJECT")) {
    int position;
    int n = sscanf(command.c_str(), "SETTIPEJECT %d", &position);
    if (n == 1 && position >= 0 && position <= TIP_EJECT_MAX_MM) {
      setTipEjectPosition(position);
      updateLEDFromPositions(); // Update LED after position change
      sendMessage("Tip eject set to " + String(position) + "mm (Red: " + String(map(position, 0, TIP_EJECT_MAX_MM, 0, 255)) + ")");
    } else {
      sendMessage("Invalid SETTIPEJECT. Usage: SETTIPEJECT <0-" + String(TIP_EJECT_MAX_MM) + ">");
    }
  }
  
  // LED Commands - SETCOLOR still available for manual override
  else if (command.startsWith("SETCOLOR")) {
    int r, g, b;
    int n = sscanf(command.c_str(), "SETCOLOR %d %d %d", &r, &g, &b);
    if (n == 3 && r >= 0 && r <= 255 && g >= 0 && g <= 255 && b >= 0 && b <= 255) {
      setLEDColor((uint8_t)r, (uint8_t)g, (uint8_t)b);
      sendMessage("LED color: R:" + String(r) + " G:" + String(g) + " B:" + String(b) + " (manual override)");
    } else {
      sendMessage("Invalid SETCOLOR. Usage: SETCOLOR r g b");
    }
  }
  
  // System Commands
  else if (command == "STATUS") {
    sendMessage(getActuatorStatus());
  }
  else if (command == "INIT") {
    sendMessage("System initialized");
  }
  else if (command == "V" || command == "v") {
    builtInLEDState = !builtInLEDState;
    digitalWrite(LED_BUILTIN, builtInLEDState);
    sendMessage("Built-in LED: " + String(builtInLEDState ? "ON" : "OFF"));
  }
  else {
    sendMessage("Unknown command: " + command);
  }
}

void sendMessage(String message) {
  RS485.beginTransmission();
  RS485.println(message);
  RS485.endTransmission();
}

// LED Functions
void setLEDColor(uint8_t r, uint8_t g, uint8_t b) {
  lastR = r;
  lastG = g;
  lastB = b;
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}

void setLEDBrightness(uint8_t brightness) {
  lastBrightness = brightness;
  pixels.setBrightness(brightness);
  setLEDColor(lastR, lastG, lastB);
}

void turnOffLEDs() {
  setLEDColor(0, 0, 0);
}

void turnOnLEDs() {
  setLEDColor(lastR, lastG, lastB);
}

// New function to update LED color based on actuator positions
void updateLEDFromPositions() {
  // Map plunger position (0-10mm) to green intensity (0-255)
  uint8_t greenValue = map(plungerPosition, 0, PLUNGER_MAX_MM, 0, 255);
  
  // Map tip eject position (0-5mm) to red intensity (0-255)
  uint8_t redValue = map(tipEjectPosition, 0, TIP_EJECT_MAX_MM, 0, 255);
  
  // Blue remains 0 for this test
  uint8_t blueValue = 0;
  
  setLEDColor(redValue, greenValue, blueValue);
}

// Actuator Functions
void initializeActuators() {
  // Configure actuator pins
  pinMode(PLUNGER_STEP_PIN, OUTPUT);
  pinMode(PLUNGER_DIR_PIN, OUTPUT);
  pinMode(PLUNGER_ENABLE_PIN, OUTPUT);
  pinMode(TIP_EJECT_STEP_PIN, OUTPUT);
  pinMode(TIP_EJECT_DIR_PIN, OUTPUT);
  pinMode(TIP_EJECT_ENABLE_PIN, OUTPUT);
  
  // Enable actuators (LOW = enabled for most stepper drivers)
  digitalWrite(PLUNGER_ENABLE_PIN, LOW);
  digitalWrite(TIP_EJECT_ENABLE_PIN, LOW);
  
  // Set initial direction (adjust as needed)
  digitalWrite(PLUNGER_DIR_PIN, LOW);
  digitalWrite(TIP_EJECT_DIR_PIN, LOW);
  
  // Initialize positions
  plungerPosition = 0;
  tipEjectPosition = 0;
}

void setPlungerPosition(int position_mm) {
  if (position_mm < 0) position_mm = 0;
  if (position_mm > PLUNGER_MAX_MM) position_mm = PLUNGER_MAX_MM;
  
  int targetSteps = position_mm * STEPS_PER_MM;
  int currentSteps = plungerPosition * STEPS_PER_MM;
  int stepDifference = targetSteps - currentSteps;
  
  if (stepDifference != 0) {
    bool direction = stepDifference > 0;
    moveActuator(PLUNGER_STEP_PIN, PLUNGER_DIR_PIN, PLUNGER_ENABLE_PIN, 
                 abs(stepDifference), direction);
    plungerPosition = position_mm;
  }
}

void setTipEjectPosition(int position_mm) {
  if (position_mm < 0) position_mm = 0;
  if (position_mm > TIP_EJECT_MAX_MM) position_mm = TIP_EJECT_MAX_MM;
  
  int targetSteps = position_mm * STEPS_PER_MM;
  int currentSteps = tipEjectPosition * STEPS_PER_MM;
  int stepDifference = targetSteps - currentSteps;
  
  if (stepDifference != 0) {
    bool direction = stepDifference > 0;
    moveActuator(TIP_EJECT_STEP_PIN, TIP_EJECT_DIR_PIN, TIP_EJECT_ENABLE_PIN, 
                 abs(stepDifference), direction);
    tipEjectPosition = position_mm;
  }
}

void moveActuator(int stepPin, int dirPin, int enablePin, int steps, bool direction) {
  // Set direction
  digitalWrite(dirPin, direction ? HIGH : LOW);
  
  // Enable actuator
  digitalWrite(enablePin, LOW);
  
  // Move steps
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);  // Adjust for your stepper motor speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
    
    // Small delay between steps
    if (ACTUATOR_SPEED_DELAY > 0) {
      delay(ACTUATOR_SPEED_DELAY);
    }
  }
}

String getHeartbeatStatus() {
  unsigned long uptime = (millis() - startTime) / 1000;
  return "Status: LED=" + String(builtInLEDState ? "ON" : "OFF") + 
         ", Uptime=" + String(uptime) + "s";
}

String getActuatorStatus() {
  uint8_t greenValue = map(plungerPosition, 0, PLUNGER_MAX_MM, 0, 255);
  uint8_t redValue = map(tipEjectPosition, 0, TIP_EJECT_MAX_MM, 0, 255);
  
  return "PLUNGER:" + String(plungerPosition) + "mm,TIP:" + String(tipEjectPosition) + 
         "mm,LED_R:" + String(redValue) + ",LED_G:" + String(greenValue) + 
         ",BuiltIn=" + String(builtInLEDState ? "ON" : "OFF");
}