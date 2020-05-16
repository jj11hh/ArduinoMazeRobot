//#include <U8g2lib.h>
//#include <U8x8lib.h>

#include "Motor9110.h"
#include "SensorMatch.h"
#include "FastGPIO.h"
#include "MazeSolver.h"

//U8X8_SSD1306_128X64_NONAME_HW_I2C
//u8x8(U8G2_R0);

//U8X8LOG u8x8log;
//uint8_t log_buf [16 * 8];

void beep(){
  uint8_t i, j;
  i = 255;
  while(i != 0){
    j = i;
    FastGPIO::Pin<A1>::setOutputValueLow();
    while (j != 0){
      delayMicroseconds(2);
      j --;
    }
    FastGPIO::Pin<A1>::setOutputValueHigh();
    j = 255 - i;
    while (j != 0){
      delayMicroseconds(2);
      j --;
    }
    -- i;
  }  
}

class Logger{
  public:
    Logger* log(int info){
      beep();
      Serial.print(info);
      return this;  
    }
    Logger* log(const char * info){
      beep();
      Serial.print(info);
      return this;  
    }
};

Motor9110<9, 6, 3, 5> motor;
SensorDigital<Pins_5<A0, 7, 12, 11, 10>> sensor;
Logger logger;
MazeSolver<typeof(motor), typeof(sensor), Logger, 32> solver(motor, sensor, logger);

void setup() {
  FastGPIO::Pin<A1>::setOutputHigh();
  FastGPIO::Pin<A2>::setOutputLow();

  Serial.begin(115200);
  logger.log("beginning sensor\n");
  sensor.begin();
  logger.log("beginning motor\n");

  FastGPIO::Pin<2>::setInputPulledUp();

  logger.log("solving\n");
  if (solver.solve()){
    logger.log("PRS TO REP\n");
    while(digitalRead(2) == HIGH);
    logger.log("REPLAYING\n");
    solver.replay();
  }
}

void loop() {
}
