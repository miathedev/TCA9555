//
//    FILE: TCA9555.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.4.3
// PURPOSE: Arduino library for I2C TCA9555 16 channel port expander
//    DATE: 2021-06-09
//     URL: https://github.com/RobTillaart/TCA9555


#include "TCA9555.h"

// Static member definitions
TCA9555* TCA9555::_instances[8] = {nullptr};
uint8_t TCA9555::_instanceCount = 0;


//  REGISTERS P23-24
#define TCA9555_INPUT_PORT_REGISTER_0     0x00    //  read()
#define TCA9555_INPUT_PORT_REGISTER_1     0x01
#define TCA9555_OUTPUT_PORT_REGISTER_0    0x02    //  write()
#define TCA9555_OUTPUT_PORT_REGISTER_1    0x03
#define TCA9555_POLARITY_REGISTER_0       0x04    //  get/setPolarity()
#define TCA9555_POLARITY_REGISTER_1       0x05
#define TCA9555_CONFIGURATION_PORT_0      0x06    //  pinMode()
#define TCA9555_CONFIGURATION_PORT_1      0x07


TCA9555::TCA9555(uint8_t address, TwoWire *wire)
{
  _address = address;
  _wire    = wire;
  _error   = TCA9555_OK;
  _type    = 55;
  
  // Initialize interrupt handling members
  _interruptPin = 255;  // Invalid pin
  _interruptEnabled = false;
  _lastState = 0;
  _currentState = 0;
  _interruptFlag = false;
  _globalCallback = nullptr;
  _instanceIndex = 255;  // Invalid index
  
  // Clear pin callbacks
  for (uint8_t i = 0; i < 16; i++) {
    _pinCallbacks[i] = nullptr;
  }
}


bool TCA9555::begin(uint8_t mode, uint16_t mask)
{
  if ((_address < 0x20) || (_address > 0x27)) return false;
  if (! isConnected()) return false;

  if (mode == OUTPUT)
  {
    pinMode16(0x0000);
    write16(mask);
  } else {
    pinMode16(0xFFFF);
  }
  return true;
}


bool TCA9555::isConnected()
{
  _wire->beginTransmission(_address);
  return (_wire->endTransmission() == 0);
}


uint8_t TCA9555::getAddress()
{
  return _address;
}


//////////////////////////////////////////////////////////
//
//  1 PIN INTERFACE
//
//  pin = 0..15
//  mode = INPUT, OUTPUT
bool TCA9555::pinMode1(uint8_t pin, uint8_t mode)
{
  if (pin > 15)
  {
    _error = TCA9555_PIN_ERROR;
    return false;
  }
  if ( (mode != INPUT) && (mode != OUTPUT) )
  {
    _error = TCA9555_VALUE_ERROR;
    return false;
  }
  uint8_t CONFREG = TCA9555_CONFIGURATION_PORT_0;
  if (pin > 7)
  {
    CONFREG = TCA9555_CONFIGURATION_PORT_1;
    pin -= 8;
  }
  uint8_t val = readRegister(CONFREG);
  uint8_t prevVal = val;
  uint8_t mask = 1 << pin;
  if (mode == INPUT)  val |= mask;
  else                val &= ~mask;
  if (val != prevVal)
  {
    return writeRegister(CONFREG, val);
  }
  _error = TCA9555_OK;
  return true;
}


//  pin = 0..15
//  value = LOW(0), HIGH(not 0)
bool TCA9555::write1(uint8_t pin, uint8_t value)
{
  if (pin > 15)
  {
    _error = TCA9555_PIN_ERROR;
    return false;
  }
  uint8_t OPR = TCA9555_OUTPUT_PORT_REGISTER_0;
  if (pin > 7)
  {
    OPR = TCA9555_OUTPUT_PORT_REGISTER_1;
    pin -= 8;
  }
  uint8_t val = readRegister(OPR);
  uint8_t prevVal = val;
  uint8_t mask = 1 << pin;
  if (value) val |= mask;  //  all values <> 0 are HIGH.
  else       val &= ~mask;
  if (val != prevVal)
  {
    return writeRegister(OPR, val);
  }
  _error = TCA9555_OK;
  return true;
}


//  pin = 0..15
uint8_t TCA9555::read1(uint8_t pin)
{
  if (pin > 15)
  {
    _error = TCA9555_PIN_ERROR;
    return TCA9555_INVALID_READ;
  }
  uint8_t IPR = TCA9555_INPUT_PORT_REGISTER_0;
  if (pin > 7)
  {
    IPR = TCA9555_INPUT_PORT_REGISTER_1;
    pin -= 8;
  }
  uint8_t val = readRegister(IPR);
  uint8_t mask = 1 << pin;
  _error = TCA9555_OK;
  if (val & mask) return HIGH;
  return LOW;
}


//  pin = 0..15
//  value = LOW(0), HIGH ?
bool TCA9555::setPolarity(uint8_t pin, uint8_t value)
{
  if (pin > 15)
  {
    _error = TCA9555_PIN_ERROR;
    return false;
  }
  if ((value != LOW) && (value != HIGH))
  {
    _error = TCA9555_VALUE_ERROR;
    return false;
  }
  uint8_t POLREG = TCA9555_POLARITY_REGISTER_0;
  if (pin > 7)
  {
    POLREG = TCA9555_POLARITY_REGISTER_1;
    pin -= 8;
  }
  uint8_t val = readRegister(POLREG);
  uint8_t prevVal = val;
  uint8_t mask = 1 << pin;
  if (value == HIGH) val |= mask;
  else               val &= ~mask;
  if (val != prevVal)
  {
    return writeRegister(POLREG, val);
  }
  _error = TCA9555_OK;
  return true;
}


//  pin = 0..15
uint8_t TCA9555::getPolarity(uint8_t pin)
{
  if (pin > 15)
  {
    _error = TCA9555_PIN_ERROR;
    return false;
  }
  uint8_t POLREG = TCA9555_POLARITY_REGISTER_0;
  if (pin > 7) POLREG = TCA9555_POLARITY_REGISTER_1;
  _error = TCA9555_OK;
  uint8_t mask = readRegister(POLREG);
  return (mask >> pin) == 0x01;
}


//////////////////////////////////////////////////////////
//
//  8 PIN INTERFACE
//
//  port = 0..1
bool TCA9555::pinMode8(uint8_t port, uint8_t mask)
{
  if (port > 1)
  {
    _error = TCA9555_PORT_ERROR;
    return false;
  }
  _error = TCA9555_OK;
  if (port == 0) return writeRegister(TCA9555_CONFIGURATION_PORT_0, mask);
  return writeRegister(TCA9555_CONFIGURATION_PORT_1, mask);
}


//  port = 0..1
//  mask = 0x00..0xFF
bool TCA9555::write8(uint8_t port, uint8_t mask)
{
  if (port > 1)
  {
    _error = TCA9555_PORT_ERROR;
    return false;
  }
  _error = TCA9555_OK;
  if (port == 0) return writeRegister(TCA9555_OUTPUT_PORT_REGISTER_0, mask);
  return writeRegister(TCA9555_OUTPUT_PORT_REGISTER_1, mask);
}


//  port = 0..1
int TCA9555::read8(uint8_t port)
{
  if (port > 1)
  {
    _error = TCA9555_PORT_ERROR;
    return TCA9555_INVALID_READ;
  }
  _error = TCA9555_OK;
  if (port == 0) return readRegister(TCA9555_INPUT_PORT_REGISTER_0);
  return readRegister(TCA9555_INPUT_PORT_REGISTER_1);
}


//  port = 0..1
//  mask = 0x00..0xFF
bool TCA9555::setPolarity8(uint8_t port, uint8_t mask)
{
  if (port > 1)
  {
    _error = TCA9555_PORT_ERROR;
    return false;
  }
  _error = TCA9555_OK;
  if (port == 0) return writeRegister(TCA9555_POLARITY_REGISTER_0, mask);
  return writeRegister(TCA9555_POLARITY_REGISTER_1, mask);
}


//  port = 0..1
uint8_t TCA9555::getPolarity8(uint8_t port)
{
  if (port > 1)
  {
    _error = TCA9555_PORT_ERROR;
    return 0;
  }
  _error = TCA9555_OK;
  if (port == 0) return readRegister(TCA9555_POLARITY_REGISTER_0);
  return readRegister(TCA9555_POLARITY_REGISTER_1);
}


//////////////////////////////////////////////////////////
//
// 16 PIN INTERFACE
//
//  mask = 0x0000..0xFFFF
bool TCA9555::pinMode16(uint16_t mask)
{
  bool b = true;
  b &= pinMode8(0, mask & 0xFF);
  b &= pinMode8(1, mask >> 8);
  return b;
}


//  mask = 0x0000..0xFFFF
bool TCA9555::write16(uint16_t mask)
{
  bool b = true;
  b &= write8(0, mask & 0xFF);
  b &= write8(1, mask >> 8);
  return b;
}


uint16_t TCA9555::read16()
{
  uint16_t rv = 0;
  rv |= (read8(1) << 8);
  rv |= read8(0);
  return rv;
}


//  mask = 0x0000..0xFFFF
bool TCA9555::setPolarity16(uint16_t mask)
{
  bool b = true;
  b &= setPolarity8(0, mask & 0xFF);
  b &= setPolarity8(1, mask >> 8);
  return b;
}


uint8_t TCA9555::getPolarity16()
{
  uint16_t rv = 0;
  rv |= (getPolarity8(1) << 8);
  rv |= getPolarity8(0);
  return rv;
}


//////////////////////////////////////////////////////////
//
//  OTHER
//
int TCA9555::lastError()
{
  int error = _error;
  _error = TCA9555_OK;  //  reset error after read.
  return error;
}


uint8_t TCA9555::getType()
{
  return _type;
}


////////////////////////////////////////////////////
//
//  DEBUG
//
void TCA9555::debugPrintGPIOs()
{
  uint16_t inputValues = read16();
  
  Serial.println("=== TCA9555 GPIO Status ===");
  Serial.print("Address: 0x");
  Serial.println(_address, HEX);
  Serial.print("Type: TCA");
  Serial.println(_type);
  Serial.println("PIN | PORT | STATE");
  Serial.println("----|------|------");
  
  for (uint8_t pin = 0; pin < 16; pin++)
  {
    uint8_t port = (pin < 8) ? 0 : 1;
    uint8_t localPin = (pin < 8) ? pin : pin - 8;
    bool state = (inputValues & (1 << pin)) ? true : false;
    
    Serial.print("P");
    Serial.print(port);
    Serial.print(localPin);
    Serial.print(" |  ");
    Serial.print(port);
    Serial.print("   |  ");
    Serial.println(state ? "HIGH" : "LOW");
  }
  
  Serial.print("Port 0 (P00-P07): 0b");
  Serial.print(inputValues & 0xFF, BIN);
  Serial.print(" (0x");
  Serial.print(inputValues & 0xFF, HEX);
  Serial.println(")");
  
  Serial.print("Port 1 (P10-P17): 0b");
  Serial.print((inputValues >> 8) & 0xFF, BIN);
  Serial.print(" (0x");
  Serial.print((inputValues >> 8) & 0xFF, HEX);
  Serial.println(")");
  
  Serial.print("All 16 pins: 0b");
  Serial.print(inputValues, BIN);
  Serial.print(" (0x");
  Serial.print(inputValues, HEX);
  Serial.println(")");
  Serial.println("===========================");
}


////////////////////////////////////////////////////
//
//  INTERRUPT HANDLING
//
bool TCA9555::enableInterrupt(uint8_t interruptPin, TCA9555_Callback callback)
{
  if (_instanceCount >= 8) {
    _error = TCA9555_VALUE_ERROR;
    return false;
  }
  
  // Find an available instance slot
  for (uint8_t i = 0; i < 8; i++) {
    if (_instances[i] == nullptr) {
      _instances[i] = this;
      _instanceIndex = i;
      break;
    }
  }
  
  if (_instanceIndex == 255) {
    _error = TCA9555_VALUE_ERROR;
    return false;
  }
  
  _instanceCount++;
  _interruptPin = interruptPin;
  _globalCallback = callback;
  _interruptEnabled = true;
  
  // Read initial state
  _lastState = read16();
  _currentState = _lastState;
  
  // Configure interrupt pin
  pinMode(_interruptPin, INPUT_PULLUP);
  
  // Attach appropriate ISR based on instance index
  switch (_instanceIndex) {
    case 0: attachInterrupt(digitalPinToInterrupt(_interruptPin), _handleISR0, FALLING); break;
    case 1: attachInterrupt(digitalPinToInterrupt(_interruptPin), _handleISR1, FALLING); break;
    case 2: attachInterrupt(digitalPinToInterrupt(_interruptPin), _handleISR2, FALLING); break;
    case 3: attachInterrupt(digitalPinToInterrupt(_interruptPin), _handleISR3, FALLING); break;
    case 4: attachInterrupt(digitalPinToInterrupt(_interruptPin), _handleISR4, FALLING); break;
    case 5: attachInterrupt(digitalPinToInterrupt(_interruptPin), _handleISR5, FALLING); break;
    case 6: attachInterrupt(digitalPinToInterrupt(_interruptPin), _handleISR6, FALLING); break;
    case 7: attachInterrupt(digitalPinToInterrupt(_interruptPin), _handleISR7, FALLING); break;
  }
  
  _error = TCA9555_OK;
  return true;
}

void TCA9555::disableInterrupt()
{
  if (_interruptEnabled && _interruptPin != 255) {
    detachInterrupt(digitalPinToInterrupt(_interruptPin));
    _interruptEnabled = false;
    
    // Clear instance from array
    if (_instanceIndex < 8) {
      _instances[_instanceIndex] = nullptr;
      _instanceCount--;
    }
    
    _interruptPin = 255;
    _instanceIndex = 255;
  }
}

void TCA9555::handleInterrupt()
{
  // This is called from ISR context - keep it as short as possible!
  // Only set a flag, don't do any I2C communication or heavy operations
  _interruptFlag = true;
}

bool TCA9555::processInterrupt()
{
  // This method should be called from the main loop to process pending interrupts
  if (!_interruptFlag || !_interruptEnabled) {
    return false;
  }
  
  // Clear the flag first
  _interruptFlag = false;
  
  // Now do the heavy work (I2C communication) in main loop context
  _currentState = read16();
  
  // Find changed pins
  uint16_t changedPins = _currentState ^ _lastState;
  
  if (changedPins == 0) {
    return false; // No actual changes
  }
  
  // Call callbacks for changed pins
  for (uint8_t pin = 0; pin < 16; pin++) {
    if (changedPins & (1 << pin)) {
      uint8_t currentPinState = (_currentState >> pin) & 0x01;
      
      // Call pin-specific callback if set
      if (_pinCallbacks[pin] != nullptr) {
        _pinCallbacks[pin](pin, currentPinState, _currentState);
      }
      
      // Call global callback if set
      if (_globalCallback != nullptr) {
        _globalCallback(pin, currentPinState, _currentState);
      }
    }
  }
  
  // Update last state
  _lastState = _currentState;
  
  return true; // Interrupt was processed
}

void TCA9555::setPinCallback(uint8_t pin, TCA9555_Callback callback)
{
  if (pin < 16) {
    _pinCallbacks[pin] = callback;
  }
}

void TCA9555::setGlobalCallback(TCA9555_Callback callback)
{
  _globalCallback = callback;
}

uint16_t TCA9555::getLastInterruptState()
{
  return _lastState;
}

uint16_t TCA9555::getCurrentState()
{
  // Always read fresh state from the chip
  _currentState = read16();
  return _currentState;
}

// Static ISR handlers
void TCA9555::_handleISR0() { if (_instances[0]) _instances[0]->handleInterrupt(); }
void TCA9555::_handleISR1() { if (_instances[1]) _instances[1]->handleInterrupt(); }
void TCA9555::_handleISR2() { if (_instances[2]) _instances[2]->handleInterrupt(); }
void TCA9555::_handleISR3() { if (_instances[3]) _instances[3]->handleInterrupt(); }
void TCA9555::_handleISR4() { if (_instances[4]) _instances[4]->handleInterrupt(); }
void TCA9555::_handleISR5() { if (_instances[5]) _instances[5]->handleInterrupt(); }
void TCA9555::_handleISR6() { if (_instances[6]) _instances[6]->handleInterrupt(); }
void TCA9555::_handleISR7() { if (_instances[7]) _instances[7]->handleInterrupt(); }


////////////////////////////////////////////////////
//
//  PROTECTED
//
bool TCA9555::writeRegister(uint8_t reg, uint8_t value)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(value);
  if (_wire->endTransmission() != 0)
  {
    _error = TCA9555_I2C_ERROR;
    return false;
  }
  _error = TCA9555_OK;
  return true;
}


uint8_t TCA9555::readRegister(uint8_t reg)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  int rv = _wire->endTransmission();
  if (rv != 0)
  {
    _error = TCA9555_I2C_ERROR;
    return rv;
  }
  else
  {
    _error = TCA9555_OK;
  }
  _wire->requestFrom(_address, (uint8_t)1);
  return _wire->read();
}


/////////////////////////////////////////////////////////////////////////////
//
//  DERIVED CLASSES TCA9535 PCA9555 PCA9535 CAT9555
//
TCA9535::TCA9535(uint8_t address, TwoWire *wire)
        :TCA9555(address, wire)
{
  _type = 35;
}

PCA9555::PCA9555(uint8_t address, TwoWire *wire)
        :TCA9555(address, wire)
{
  _type = 55;
}

PCA9535::PCA9535(uint8_t address, TwoWire *wire)
        :TCA9555(address, wire)
{
  _type = 35;
}

CAT9555::CAT9555(uint8_t address, TwoWire *wire)
        :TCA9555(address, wire)
{
  _type = 55;
}


//  -- END OF FILE --

