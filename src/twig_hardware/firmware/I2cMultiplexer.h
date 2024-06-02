#ifndef I2C_MULTIPLEXER_h
#define I2C_MULTIPLEXER_h

#include <Wire.h>

struct I2cMultiplexer
{
  int multiplexerAddress;
  TwoWire &bus;

  I2cMultiplexer(TwoWire &bus, int multiplexerAddress) : bus(bus), multiplexerAddress(multiplexerAddress)
  {
  }

  bool deviceExists(uint8_t address)
  {
    bus.beginTransmission(address);
    return bus.endTransmission() == 0;
  }

  // https://randomnerdtutorials.com/tca9548a-i2c-multiplexer-esp32-esp8266-arduino/
  bool selectChannel(uint8_t channel)
  {
    bus.beginTransmission(multiplexerAddress);
    bus.write(1 << channel);
    if (bus.endTransmission() != 0)
    {
      Serial.println("I2C Multiplexer 0x" + String(multiplexerAddress, HEX) + " failed to select channel " + String(channel));
      return false;
    }
    return true;
  }
};

#endif
