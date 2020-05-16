#pragma once

#include <Arduino.h>
#include <FastGPIO.h>
#include <avdweb_AnalogReadFast.h>

template <uint8_t ...pins>
struct Pins_3{
  static_assert(sizeof...(pins) == 3, "3 pins required");
  static uint8_t constexpr constValue[3] = {pins...};
  static uint8_t const value[3]; 
};

template <uint8_t ...pins>
uint8_t const Pins_3<pins...>::value[] = {pins...};

template <uint8_t ...pins>
struct Pins_8{
  static_assert(sizeof...(pins) == 8, "8 pins required");
  static uint8_t constexpr constValue[8] = {pins...};
  static uint8_t const value[8];
};

template <uint8_t ...pins>
uint8_t const Pins_8<pins...>::value[] = {pins...};


template <typename Pins, uint8_t analogPin, typename Remapper>
class Sensor4051 {
  public:
    inline void begin() {
      FastGPIO::Pin<Pins::constValue[0]>::setOutputLow();
      FastGPIO::Pin<Pins::constValue[1]>::setOutputLow();
      FastGPIO::Pin<Pins::constValue[2]>::setOutputLow();

      //pinMode(Pins::constValue[0], OUTPUT);
      //pinMode(Pins::constValue[1], OUTPUT);
      //pinMode(Pins::constValue[2], OUTPUT);
    }
    int readSensor(char n){
      FastGPIO::Pin<Pins::constValue[0]>::setOutputValue(Remapper::value[n] & (1 << 0));
      FastGPIO::Pin<Pins::constValue[1]>::setOutputValue(Remapper::value[n] & (1 << 1));
      FastGPIO::Pin<Pins::constValue[2]>::setOutputValue(Remapper::value[n] & (1 << 2));
      delayMicroseconds(200);
      return analogReadFast(analogPin);
    }
};
