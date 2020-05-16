#pragma once

#include <Arduino.h>
#include <FastGPIO.h>

template <int clk1, int cnt1, int clk2, int cnt2>
class Motor9110 {
public:
  Motor9110 (){}

  inline void begin () __attribute__((always_inline)){
//    FastGPIO::Pin<clk1>::setOutputHigh();  
//    FastGPIO::Pin<cnt1>::setOutputHigh();  
//    FastGPIO::Pin<clk2>::setOutputHigh();  
//    FastGPIO::Pin<cnt2>::setOutputHigh();  
//    FastGPIO::Pin<clk1>::setPWM(255);
//    FastGPIO::Pin<cnt1>::setPWM(255);
//    FastGPIO::Pin<clk2>::setPWM(255);
//    FastGPIO::Pin<cnt2>::setPWM(255);
    pinMode(clk1, OUTPUT);
    pinMode(cnt1, OUTPUT);
    pinMode(clk2, OUTPUT);
    pinMode(cnt2, OUTPUT);
    analogWrite(clk1, 255);
    analogWrite(cnt1, 255);
    analogWrite(clk2, 255);
    analogWrite(cnt2, 255);
  }

  void inline setMotorLeft(bool backward, unsigned char speed)__attribute__((always_inline)){
    if (speed == 0){
//      FastGPIO::Pin<cnt1>::setPWMValue(255);
//      FastGPIO::Pin<clk1>::setPWMValue(255);
      analogWrite(cnt1, 255);
      analogWrite(clk1, 255);
    }
    else if (backward) {
//      FastGPIO::Pin<cnt1>::setPWMValue(255);
//      FastGPIO::Pin<clk1>::setPWMValue(255 - speed);
      analogWrite(cnt1, 255);
      analogWrite(clk1, 255 - speed);
    }
    else {
//      FastGPIO::Pin<clk1>::setPWMValue(255);
//      FastGPIO::Pin<cnt1>::setPWMValue(255 - speed);
      analogWrite(clk1, 255);
      analogWrite(cnt1, 255 - speed);  
    }  
  }

  void inline setMotorRight(bool backward, unsigned char speed)__attribute__((always_inline)){
    if (speed == 0){
//      FastGPIO::Pin<cnt2>::setPWMValue(255);
//      FastGPIO::Pin<clk2>::setPWMValue(255);
      analogWrite(cnt2, 255);
      analogWrite(clk2, 255);
    }
    else if (backward) {
//      FastGPIO::Pin<cnt2>::setPWMValue(255);
//      FastGPIO::Pin<clk2>::setPWMValue(255 - speed);
      analogWrite(cnt2, 255);
      analogWrite(clk2, 255 - speed);
    }
    else {
//      FastGPIO::Pin<clk2>::setPWMValue(255);
//      FastGPIO::Pin<cnt2>::setPWMValue(255 - speed);
      analogWrite(clk2, 255);
      analogWrite(cnt2, 255 - speed);  
    }  
  }

  inline void setMotor (unsigned char motorId, bool backward, unsigned char speed){
    if (motorId == 1){
      setMotorLeft(backward, speed);
    }
    else if (motorId == 2){
      setMotorRight(backward, speed);
    }
  }
  inline void setMotor(int speedL, int speedR){
    if(speedL > 0){
      setMotorLeft(false, speedL);  
    }
    else{
      setMotorLeft(true, -speedL);  
    }

    if(speedR > 0){
      setMotorRight(false, speedR);
    }
    else{
      setMotorRight(true, -speedR);
    }
  }
};

#define SETMOTOR(motor, speedL, speedR) do{\
  if((speedL) > 0){\
    (motor).setMotorLeft(false, (speedL));\
  }\
  else{\
    (motor).setMotorLeft(true, -(speedL));\
  }\
  if(speedR > 0){\
    (motor).setMotorRight(false, (speedR));\
  }\
  else{\
    (motor).setMotorRight(true, -(speedR));\
  }\
}while(0);
