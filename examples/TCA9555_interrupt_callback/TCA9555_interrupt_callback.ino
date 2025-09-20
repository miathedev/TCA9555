//
//    FILE: TCA9555_interrupt_callback.ino
//  AUTHOR: GitHub Copilot
// PURPOSE: test TCA9555 interrupt handling with callbacks
//     URL: https://github.com/RobTillaart/TCA9555
//
//  Demonstrates the new interrupt handling functionality with callbacks
//  IMPORTANT: You must call processInterrupt() in your main loop to handle interrupts!


#include "TCA9555.h"

#define I2C_SCL 5
#define I2C_SDA 4
#define INT_PIN 6

TCA9555 TCA(0x20);

// Global callback function - called for any pin change
void onAnyPinChange(uint8_t pin, uint8_t state, uint16_t allPins) {
  Serial.print("Global callback - Pin P");
  if (pin < 8) {
    Serial.print("0");
    Serial.print(pin);
  } else {
    Serial.print("1");
    Serial.print(pin - 8);
  }
  Serial.print(" changed to ");
  Serial.print(state ? "HIGH" : "LOW");
  Serial.print(" (All pins: 0x");
  Serial.print(allPins, HEX);
  Serial.println(")");
}

// Specific callback for button on pin P00
void onButton0Change(uint8_t pin, uint8_t state, uint16_t allPins) {
  Serial.print("Button 0 (P00) ");
  Serial.println(state ? "PRESSED" : "RELEASED");
}

// Specific callback for button on pin P01
void onButton1Change(uint8_t pin, uint8_t state, uint16_t allPins) {
  Serial.print("Button 1 (P01) ");
  Serial.println(state ? "PRESSED" : "RELEASED");
}

void setup() {
  Serial.begin(115200);
  Serial.println("TCA9555 Interrupt Callback Example");
  Serial.print("TCA9555_LIB_VERSION: ");
  Serial.println(TCA9555_LIB_VERSION);
  Serial.println();

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Check connection
  if (!TCA.isConnected()) {
    Serial.println("ERROR: TCA9555 not connected!");
    while(1);
  }
  
  // Initialize TCA9555
  if (!TCA.begin()) {
    Serial.println("ERROR: TCA9555 begin failed!");
    while(1);
  }

  // Set all pins as inputs
  Serial.println("Setting all pins as INPUT");
  TCA.pinMode16(0xFFFF);
  
  // Set polarity inversion for pins P00-P05 (button pins)
  // This makes pressed buttons (pulling to ground) read as HIGH
  TCA.setPolarity16(0x003F);  // Invert polarity for pins 0-5
  
  // Set up callbacks
  TCA.setGlobalCallback(onAnyPinChange);        // Global callback for all pins
  TCA.setPinCallback(0, onButton0Change);       // Specific callback for P00
  TCA.setPinCallback(1, onButton1Change);       // Specific callback for P01
  
  // Enable interrupt handling with automatic callback management
  if (!TCA.enableInterrupt(INT_PIN)) {
    Serial.println("ERROR: Failed to enable interrupt!");
    while(1);
  }
  
  Serial.println("Setup complete - ready for button presses");
  Serial.println("Press buttons on P00-P05 to see callbacks in action");
  Serial.println("IMPORTANT: The main loop calls processInterrupt() to handle interrupts safely");
  
  // Print initial state
  TCA.debugPrintGPIOs();
}

void loop() {
  // CRITICAL: Call processInterrupt() to handle any pending interrupts
  // This processes the heavy I2C work in main loop context, not in ISR
  // Without this call, your callbacks will never be triggered!
  TCA.processInterrupt();
  
  // Your main application logic goes here
  delay(10);  // Small delay to prevent overwhelming the system
  
  // Optional: You can still manually check the current state if needed
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 5000) {  // Print state every 5 seconds
    Serial.println("\n--- Current Status (every 5s) ---");
    uint16_t currentState = TCA.getCurrentState();
    Serial.print("Current state: 0x");
    Serial.println(currentState, HEX);
    lastPrint = millis();
  }
}