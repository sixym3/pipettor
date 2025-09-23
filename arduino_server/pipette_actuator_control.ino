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
  sendMessage("Commands: PXXX (plunger %), TXXX (tip %), SPPPTTT (both %), STATUS, INIT, HELP");
  sendMessage("Examples: P055 (55%), T030 (30%), S055030 (plunger 55%, tip 30%)");
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
  command.toUpperCase();  // Make commands case-insensitive

  // Parse command and sequence number
  String baseCommand = command;
  String sequenceNum = "";

  // Check for sequence number format: COMMAND_XXX
  int underscoreIndex = command.lastIndexOf('_');
  if (underscoreIndex > 0 && underscoreIndex < command.length() - 1) {
    baseCommand = command.substring(0, underscoreIndex);
    sequenceNum = command.substring(underscoreIndex + 1);
  }

  // Send immediate acknowledgment for position commands
  if (baseCommand.startsWith("P") || baseCommand.startsWith("T") || baseCommand.startsWith("S")) {
    sendMessage("ACK " + command);  // Echo full command with sequence
  } else {
    sendMessage("Received: " + command);
  }

  // New compact protocol commands - use baseCommand for parsing, full command for DONE
  if (baseCommand.startsWith("P") && baseCommand.length() == 4) {
    // PXXX format - plunger percentage (P000-P100)
    String percentStr = baseCommand.substring(1);
    int percent = percentStr.toInt();

    if (percent >= 0 && percent <= 100) {
      float position = percent / 100.0;
      setPlungerPosition(position);
      sendMessage("Plunger set to " + String(percent) + "% (" + String(position, 3) + ")");
      delay(500);  // Basic movement time estimate
      sendMessage("DONE " + command);  // Echo full command with sequence
    } else {
      sendMessage("Invalid plunger percentage: " + String(percent) + ". Range: P000-P100");
    }
  }
  else if (baseCommand.startsWith("T") && baseCommand.length() == 4) {
    // TXXX format - tip eject percentage (T000-T100)
    String percentStr = baseCommand.substring(1);
    int percent = percentStr.toInt();

    if (percent >= 0 && percent <= 100) {
      float position = percent / 100.0;
      setTipEjectPosition(position);
      sendMessage("Tip eject set to " + String(percent) + "% (" + String(position, 3) + ")");
      delay(500);  // Basic movement time estimate
      sendMessage("DONE " + command);  // Echo full command with sequence
    } else {
      sendMessage("Invalid tip percentage: " + String(percent) + ". Range: T000-T100");
    }
  }
  else if (baseCommand.startsWith("S") && baseCommand.length() == 7) {
    // SPPPTTT format - combined command (S055030)
    String plungerStr = baseCommand.substring(1, 4);
    String tipStr = baseCommand.substring(4, 7);
    int plungerPercent = plungerStr.toInt();
    int tipPercent = tipStr.toInt();

    if (plungerPercent >= 0 && plungerPercent <= 100 && tipPercent >= 0 && tipPercent <= 100) {
      float plungerPos = plungerPercent / 100.0;
      float tipPos = tipPercent / 100.0;

      setPlungerPosition(plungerPos);
      setTipEjectPosition(tipPos);
      sendMessage("Both set - Plunger: " + String(plungerPercent) + "%, Tip: " + String(tipPercent) + "%");
      delay(750);  // Slightly longer for coordinated movement
      sendMessage("DONE " + command);  // Echo full command with sequence
    } else {
      sendMessage("Invalid combined command: S" + plungerStr + tipStr + ". Range: S000000-S100100");
    }
  }
  
  // Legacy convenience commands (still supported)
  else if (baseCommand == "RETRACT") {
    setPlungerPosition(0.0);
    setTipEjectPosition(0.0);
    sendMessage("Both actuators retracted to 0% (same as S000000)");
  }
  else if (baseCommand == "STOP") {
    // Emergency stop - retract everything to 0
    setPlungerPosition(0.0);
    setTipEjectPosition(0.0);
    sendMessage("STOP - Both actuators retracted to 0% (same as S000000)");
  }

  // System Commands
  else if (baseCommand == "STATUS") {
    sendMessage(getActuatorStatus());
  }
  else if (baseCommand == "INIT") {
    initializeActuators();
    sendMessage("Actuators initialized (both retracted to 0.0)");
  }
  else if (baseCommand == "V" || baseCommand == "v") {
    builtInLEDState = !builtInLEDState;
    digitalWrite(LED_BUILTIN, builtInLEDState);
    sendMessage("Built-in LED: " + String(builtInLEDState ? "ON" : "OFF"));
  }
  else if (baseCommand == "HELP") {
    sendMessage("=== PIPETTE ACTUATOR COMMANDS ===");
    sendMessage("PXXX - Set plunger percentage (P000-P100)");
    sendMessage("TXXX - Set tip eject percentage (T000-T100)");
    sendMessage("SPPPTTT - Set both (S055030 = plunger 55%, tip 30%)");
    sendMessage("RETRACT - Move both to 0% (P000 + T000)");
    sendMessage("STOP - Retract both actuators to 0%");
    sendMessage("STATUS - Show current positions");
    sendMessage("INIT - Initialize actuators to 0%");
    sendMessage("V - Toggle built-in LED");
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

  // No blocking delay - actuator control is now non-blocking
}

void setTipEjectPosition(float position) {
  // Constrain position to valid range
  if (position < TIP_EJECT_MIN_POS) position = TIP_EJECT_MIN_POS;
  if (position > TIP_EJECT_MAX_POS) position = TIP_EJECT_MAX_POS;

  // Update position and set PWM
  tipEjectPosition = position;
  setActuatorPWM(TIP_EJECT_PWM_PIN, position);

  // No blocking delay - actuator control is now non-blocking
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