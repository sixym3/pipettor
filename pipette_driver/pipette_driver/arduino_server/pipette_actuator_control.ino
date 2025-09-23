// Arduino Pipette Linear Actuator Control with RS485 Communication
// Controls 2 linear actuators (plunger and tip ejection) plus LED strip
// Uses PWM control for linear actuators instead of stepper motors

#include <ArduinoRS485.h>
#include <Adafruit_NeoPixel.h>

// LED Strip Configuration
#define LED_PIN 12
#define NUM_PIXELS 24
Adafruit_NeoPixel pixels(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Built-in LED
#define LED_BUILTIN 13

// Linear Actuator Pin Definitions
#define PLUNGER_PWM_PIN 9      // PWM control for plunger actuator
#define TIP_EJECT_PWM_PIN 10   // PWM control for tip eject actuator

// Actuator Configuration (positions as percentage 0.0-1.0)
#define PLUNGER_MIN_POS 0.0    // Fully retracted
#define PLUNGER_MAX_POS 1.0    // Fully extended
#define TIP_EJECT_MIN_POS 0.0  // Fully retracted
#define TIP_EJECT_MAX_POS 1.0  // Fully extended

// Timing constants
#define STARTUP_FLASH_DELAY 50
#define HEARTBEAT_INTERVAL 5000
#define HEARTBEAT_DURATION 50
#define ACTUATOR_SETTLE_TIME 100  // Time to wait after position change

// Global variables
unsigned long lastHeartbeat = 0;
String receiveBuffer = "";
bool builtInLEDState = false;
unsigned long startTime = 0;

// LED state variables
uint8_t lastR = 0, lastG = 0, lastB = 0;
uint8_t lastBrightness = 50;

// Actuator position variables (0.0 = retracted, 1.0 = extended)
float plungerPosition = 0.0;
float tipEjectPosition = 0.0;

// Function prototypes
void setLEDColor(uint8_t r, uint8_t g, uint8_t b);
void setLEDBrightness(uint8_t brightness);
void turnOffLEDs();
void turnOnLEDs();
String getHeartbeatStatus();

// Actuator control functions
void initializeActuators();
void setPlungerPosition(float position);
void setTipEjectPosition(float position);
void setActuatorPWM(int pin, float position);
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
  setLEDColor(0, 0, 0); // Start off
  
  // Start RS485 communication at 115200 baud
  RS485.begin(115200);
  RS485.receive();
  
  // Send startup messages
  sendMessage("Pipette Linear Actuator Control Ready");
  sendMessage("Position range: 0.0 (retracted) to 1.0 (extended)");
  sendMessage("Commands: SETPOSITION <plunger> <tip>, SETPLUNGER <pos>, SETTIPEJECT <pos>, SETCOLOR <r> <g> <b>");
  sendMessage("Position examples: 0.0, 0.25, 0.5, 0.75, 1.0");
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
    String params = command.substring(12);  // "SETPOSITION " = 12 chars
    params.trim();  // Remove whitespace
    
    int spaceIndex = params.indexOf(' ');
    if (spaceIndex > 0 && spaceIndex < params.length() - 1) {
      String plungerStr = params.substring(0, spaceIndex);
      String tipStr = params.substring(spaceIndex + 1);
      plungerStr.trim();
      tipStr.trim();
      
      if (plungerStr.length() > 0 && tipStr.length() > 0) {
        float plunger_pos = plungerStr.toFloat();
        float tip_pos = tipStr.toFloat();
        
        if (plunger_pos >= PLUNGER_MIN_POS && plunger_pos <= PLUNGER_MAX_POS && 
            tip_pos >= TIP_EJECT_MIN_POS && tip_pos <= TIP_EJECT_MAX_POS) {
          // Move both actuators
          setPlungerPosition(plunger_pos);
          setTipEjectPosition(tip_pos);
          sendMessage("Position set - Plunger: " + String(plunger_pos, 3) + ", Tip: " + String(tip_pos, 3));
        } else {
          sendMessage("Invalid SETPOSITION values: Plunger=" + String(plunger_pos, 3) + ", Tip=" + String(tip_pos, 3));
          sendMessage("Range: 0.0-1.0 for both");
        }
      } else {
        sendMessage("Invalid SETPOSITION format. Usage: SETPOSITION <plunger_0.0-1.0> <tip_0.0-1.0>");
        sendMessage("Example: SETPOSITION 0.5 0.25");
      }
    } else {
      sendMessage("Invalid SETPOSITION. Usage: SETPOSITION <plunger_0.0-1.0> <tip_0.0-1.0>");
      sendMessage("Example: SETPOSITION 0.5 0.25");
    }
  }
  else if (command.startsWith("SETPLUNGER")) {
    String posStr = command.substring(11);  // "SETPLUNGER " = 11 chars
    posStr.trim();  // Remove whitespace
    
    if (posStr.length() > 0) {
      float position = posStr.toFloat();
      if (position >= PLUNGER_MIN_POS && position <= PLUNGER_MAX_POS) {
        setPlungerPosition(position);
        sendMessage("Plunger set to " + String(position, 3));
      } else {
        sendMessage("Invalid SETPLUNGER position: " + String(position, 3) + ". Range: 0.0-1.0");
      }
    } else {
      sendMessage("Invalid SETPLUNGER. Usage: SETPLUNGER <0.0-1.0>");
      sendMessage("Example: SETPLUNGER 0.75");
    }
  }
  else if (command.startsWith("SETTIPEJECT")) {
    String posStr = command.substring(12);  // "SETTIPEJECT " = 12 chars
    posStr.trim();  // Remove whitespace
    
    if (posStr.length() > 0) {
      float position = posStr.toFloat();
      if (position >= TIP_EJECT_MIN_POS && position <= TIP_EJECT_MAX_POS) {
        setTipEjectPosition(position);
        sendMessage("Tip eject set to " + String(position, 3));
      } else {
        sendMessage("Invalid SETTIPEJECT position: " + String(position, 3) + ". Range: 0.0-1.0");
      }
    } else {
      sendMessage("Invalid SETTIPEJECT. Usage: SETTIPEJECT <0.0-1.0>");
      sendMessage("Example: SETTIPEJECT 0.3");
    }
  }
  
  // Convenience Commands
  else if (command == "RETRACT") {
    setPlungerPosition(0.0);
    setTipEjectPosition(0.0);
    sendMessage("Both actuators retracted to 0.0");
  }
  else if (command == "EXTEND") {
    setPlungerPosition(1.0);
    setTipEjectPosition(1.0);
    sendMessage("Both actuators extended to 1.0");
  }
  else if (command == "RETRACTPLUNGER") {
    setPlungerPosition(0.0);
    sendMessage("Plunger retracted to 0.0");
  }
  else if (command == "EXTENDPLUNGER") {
    setPlungerPosition(1.0);
    sendMessage("Plunger extended to 1.0");
  }
  else if (command == "RETRACTTIP") {
    setTipEjectPosition(0.0);
    sendMessage("Tip eject retracted to 0.0");
  }
  else if (command == "EXTENDTIP") {
    setTipEjectPosition(1.0);
    sendMessage("Tip eject extended to 1.0");
  }
  
  // LED Commands
  else if (command.startsWith("SETCOLOR")) {
    int r, g, b;
    int n = sscanf(command.c_str(), "SETCOLOR %d %d %d", &r, &g, &b);
    if (n == 3 && r >= 0 && r <= 255 && g >= 0 && g <= 255 && b >= 0 && b <= 255) {
      setLEDColor((uint8_t)r, (uint8_t)g, (uint8_t)b);
      sendMessage("LED color: R:" + String(r) + " G:" + String(g) + " B:" + String(b));
    } else {
      sendMessage("Invalid SETCOLOR. Usage: SETCOLOR r g b");
      sendMessage("Example: SETCOLOR 255 0 128");
    }
  }
  else if (command == "LEDON") {
    turnOnLEDs();
    sendMessage("LEDs turned on");
  }
  else if (command == "LEDOFF") {
    turnOffLEDs();
    sendMessage("LEDs turned off");
  }
  
  // System Commands
  else if (command == "STATUS") {
    sendMessage(getActuatorStatus());
  }
  else if (command == "INIT") {
    initializeActuators();
    sendMessage("Actuators initialized (both retracted to 0.0)");
  }
  else if (command == "V" || command == "v") {
    builtInLEDState = !builtInLEDState;
    digitalWrite(LED_BUILTIN, builtInLEDState);
    sendMessage("Built-in LED: " + String(builtInLEDState ? "ON" : "OFF"));
  }
  else if (command == "HELP") {
    sendMessage("=== PIPETTE ACTUATOR COMMANDS ===");
    sendMessage("SETPOSITION <plunger> <tip> - Set both positions (0.0-1.0)");
    sendMessage("SETPLUNGER <pos> - Set plunger position (0.0-1.0)");
    sendMessage("SETTIPEJECT <pos> - Set tip eject position (0.0-1.0)");
    sendMessage("RETRACT/EXTEND - Move both to 0.0/1.0");
    sendMessage("RETRACTPLUNGER/EXTENDPLUNGER - Move plunger to 0.0/1.0");
    sendMessage("RETRACTTIP/EXTENDTIP - Move tip to 0.0/1.0");
    sendMessage("SETCOLOR <r> <g> <b> - Set LED color");
    sendMessage("STATUS - Show current positions");
    sendMessage("INIT - Initialize actuators");
  }
  else {
    sendMessage("Unknown command: " + command + " (send HELP for commands)");
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

// Linear Actuator Functions
void initializeActuators() {
  // Configure PWM pins
  pinMode(PLUNGER_PWM_PIN, OUTPUT);
  pinMode(TIP_EJECT_PWM_PIN, OUTPUT);
  
  // Initialize both actuators to retracted position
  setPlungerPosition(0.0);
  setTipEjectPosition(0.0);
  
  sendMessage("Linear actuators initialized - Pin " + String(PLUNGER_PWM_PIN) + 
              " (plunger), Pin " + String(TIP_EJECT_PWM_PIN) + " (tip eject)");
}

void setPlungerPosition(float position) {
  // Constrain position to valid range
  if (position < PLUNGER_MIN_POS) position = PLUNGER_MIN_POS;
  if (position > PLUNGER_MAX_POS) position = PLUNGER_MAX_POS;
  
  // Update position and set PWM
  plungerPosition = position;
  setActuatorPWM(PLUNGER_PWM_PIN, position);
  
  // Small delay to allow actuator to start moving
  delay(ACTUATOR_SETTLE_TIME);
}

void setTipEjectPosition(float position) {
  // Constrain position to valid range
  if (position < TIP_EJECT_MIN_POS) position = TIP_EJECT_MIN_POS;
  if (position > TIP_EJECT_MAX_POS) position = TIP_EJECT_MAX_POS;
  
  // Update position and set PWM
  tipEjectPosition = position;
  setActuatorPWM(TIP_EJECT_PWM_PIN, position);
  
  // Small delay to allow actuator to start moving
  delay(ACTUATOR_SETTLE_TIME);
}

void setActuatorPWM(int pin, float position) {
  // Convert float position (0.0-1.0) to PWM value (0-255)
  int pwmValue = (int)(position * 255);
  
  // Ensure PWM value is within bounds
  if (pwmValue < 0) pwmValue = 0;
  if (pwmValue > 255) pwmValue = 255;
  
  // Set PWM output
  analogWrite(pin, pwmValue);
}

String getHeartbeatStatus() {
  unsigned long uptime = (millis() - startTime) / 1000;
  return "Heartbeat: Plunger=" + String(plungerPosition, 3) + 
         ", Tip=" + String(tipEjectPosition, 3) + 
         ", Uptime=" + String(uptime) + "s";
}

String getActuatorStatus() {
  return "STATUS: PLUNGER=" + String(plungerPosition, 3) + 
         ", TIP=" + String(tipEjectPosition, 3) + 
         ", LED=" + String(builtInLEDState ? "ON" : "OFF") + 
         ", PWM_PINS=" + String(PLUNGER_PWM_PIN) + "," + String(TIP_EJECT_PWM_PIN);
}