#pragma once

#include <FastGPIO.h>
#include <Arduino.h>

template <uint8_t ...pins>
struct Pins_5{
  static_assert(sizeof...(pins) == 5, "5 pins required");
  static uint8_t constexpr constValue[5] = {pins...};
  static uint8_t const value[5]; 
};

template <uint8_t ...pins>
uint8_t const Pins_5<pins...>::value[] = {pins...};

template <typename Pins>
class SensorDigital {
  public:
    inline void begin() {
      FastGPIO::Pin<Pins::constValue[0]>::setInputPulledUp();
      FastGPIO::Pin<Pins::constValue[1]>::setInputPulledUp();
      FastGPIO::Pin<Pins::constValue[2]>::setInputPulledUp();
      FastGPIO::Pin<Pins::constValue[3]>::setInputPulledUp();
      FastGPIO::Pin<Pins::constValue[4]>::setInputPulledUp();
    }
    inline bool readSensor(char n) __attribute__((always_inline)){
      switch (n){
        case 0:  return FastGPIO::Pin<Pins::constValue[0]>::isInputHigh();
        case 1:  return FastGPIO::Pin<Pins::constValue[1]>::isInputHigh();
        case 2:  return FastGPIO::Pin<Pins::constValue[2]>::isInputHigh();
        case 3:  return FastGPIO::Pin<Pins::constValue[3]>::isInputHigh();
        case 4:  return FastGPIO::Pin<Pins::constValue[4]>::isInputHigh();
        default: return false;   
      }
    }
};
