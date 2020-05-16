#pragma once
#include "SensorMatch.h"
#include <Arduino.h>
#include <stdint.h>

#define DASH_TIME      350
#define BLANK_TIME     200
#define CRUISE_SPEED  200
#define DEBONUS_TIME  20
#define TURN_SPEED    200
#define TURN_TIME     650

#define JUNC_DEFS \
  JUNC_DEF(ROADEND, 0b0010)\
  JUNC_DEF(EXIT,    0b0000)\
  JUNC_DEF(LCORN,   0b1010)\
  JUNC_DEF(LJUNC,   0b1110)\
  JUNC_DEF(RCORN,   0b0011)\
  JUNC_DEF(RJUNC,   0b0111)\
  JUNC_DEF(TJUNC,   0b1011)\
  JUNC_DEF(CROSS,   0b1111)\
  JUNC_DEF(NORM,    0b0110)

#define _LEFT  3
#define _FORE  2
#define _BACK  1
#define _RIGHT 0
#define _juncHas(junc, end) ((junc) & (1 << (end)))

typedef uint8_t junction_t;

#define JUNC_DEF(name, mask) constexpr junction_t name = mask;
    JUNC_DEFS
#undef JUNC_DEF

template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE = 64>
class MazeSolver{
  private:
    TMotor    &m_motor;
    TSensor   &m_sensor;
    TLogger   &m_logger;
    uint32_t  m_blankTime = 0;

    junction_t m_stackJunc[STACK_SIZE];
    uint8_t   m_stackChoose[STACK_SIZE];
    uint8_t   m_sp = 0;
    SensorMatch<TSensor, 5> * m_matcher;
    uint32_t  m_debonus = 0;

    void redirect(uint8_t curDirect, uint8_t distDirect);
    void redirectWithLine(uint8_t curDirect, uint8_t distDirect);
    void                stackPush(junction_t, uint8_t);
    void                stackPop(junction_t *, uint8_t *);
    void                stackPeek(junction_t *, uint8_t *);
    void                dash(uint16_t time);
    void                turnLeft();
    void                turnRight();
    void                turnAround();
    void                turnLeftWithLine();
    void                turnRightWithLine();
    void                turnAroundWithLine();
    public :inline void fix() __attribute__((always_inline)){
      m_matcher->update();
      if(MATCH(m_matcher, ABAAA)){
        SETMOTOR(m_motor, -CRUISE_SPEED, CRUISE_SPEED);
      }
      else if(MATCH(m_matcher, AAABA)){
        SETMOTOR(m_motor, CRUISE_SPEED, -CRUISE_SPEED);
      }
      else{
        SETMOTOR(m_motor, CRUISE_SPEED, CRUISE_SPEED);
      }
    }
  public:
    MazeSolver(TMotor &motor, TSensor &sensor, TLogger &logger): m_motor(motor), m_sensor(sensor), m_logger(logger){
      m_matcher = new SensorMatch<TSensor, 5>(m_sensor);  
    }
    SensorMatch<TSensor, 5>* getMatcher();
    bool                solve();
    junction_t          cruise();
    void                replay();
    static const char*  juncToName(junction_t);
};



template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE>
const static char* MazeSolver<TMotor, TSensor,  TLogger, STACK_SIZE>::juncToName(junction_t junc) {
  switch (junc){
#define JUNC_DEF(name, mask) case mask: return #name; break;
    JUNC_DEFS
#undef JUNC_DEF
    default:
      return "UNDEFINED";
  }
}

template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE>
void MazeSolver<TMotor, TSensor,  TLogger, STACK_SIZE>::stackPush(junction_t junc, uint8_t choose){
  m_stackJunc[m_sp] = junc;
  m_stackChoose[m_sp] = choose;
  m_sp ++;
}

template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE>
void MazeSolver<TMotor, TSensor,  TLogger, STACK_SIZE>::stackPop(junction_t* junc, uint8_t* choose){
  m_sp --;
  *junc  = m_stackJunc[m_sp];
  *choose = m_stackChoose[m_sp];
}

template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE>
void MazeSolver<TMotor, TSensor,  TLogger, STACK_SIZE>::stackPeek(junction_t* junc, uint8_t* choose){
  *junc  = m_stackJunc[m_sp - 1];
  *choose = m_stackChoose[m_sp - 1];
}

template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE>
void MazeSolver<TMotor, TSensor,  TLogger, STACK_SIZE>::redirect(uint8_t curDirect, uint8_t distDirect){
  if (curDirect == distDirect){
    return;  // do nothing  
  }    

#define m(cur, dist) (curDirect == (cur) && distDirect == (dist))
  if (m(_FORE, _BACK) || m(_BACK, _FORE) || m(_LEFT, _RIGHT) || m(_RIGHT, _LEFT)){
    turnAround();
    return;  
  }

  if (m(_FORE, _LEFT) || m(_BACK, _RIGHT) || m(_LEFT, _BACK) || m(_RIGHT, _FORE)){
    turnLeft();
    return;  
  }

  if (m(_FORE, _RIGHT) || m(_BACK, _LEFT) || m(_LEFT, _FORE) || m(_RIGHT, _BACK)){
    turnRight();                                                                            
    return;  
  }
#undef m
}

template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE>
void MazeSolver<TMotor, TSensor,  TLogger, STACK_SIZE>::redirectWithLine(uint8_t curDirect, uint8_t distDirect){
  if (curDirect == distDirect){
    return;  // do nothing  
  }    

#define m(cur, dist) (curDirect == (cur) && distDirect == (dist))
  if (m(_FORE, _BACK) || m(_BACK, _FORE) || m(_LEFT, _RIGHT) || m(_RIGHT, _LEFT)){
    turnAroundWithLine();
    return;  
  }

  if (m(_FORE, _LEFT) || m(_BACK, _RIGHT) || m(_LEFT, _BACK) || m(_RIGHT, _FORE)){
    turnLeftWithLine();
    return;  
  }

  if (m(_FORE, _RIGHT) || m(_BACK, _LEFT) || m(_LEFT, _FORE) || m(_RIGHT, _BACK)){
    turnRightWithLine();                                                                            
    return;  
  }
#undef m
}


template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE>
void MazeSolver<TMotor, TSensor,  TLogger, STACK_SIZE>::dash(uint16_t time){
  SETMOTOR(m_motor, CRUISE_SPEED, CRUISE_SPEED);
  delay(time);
  SETMOTOR(m_motor, 0, 0);
}


template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE>
bool MazeSolver<TMotor, TSensor,  TLogger, STACK_SIZE>::solve(){
  junction_t junc;
  uint8_t    choose;
  while ((junc = cruise()) == NORM);

  m_logger.log(juncToName(junc))->log(" found\n");

  uint8_t curDirect;
  curDirect = _FORE;

  if (_juncHas(junc, _LEFT)){
    redirect(curDirect, _LEFT);
    stackPush(junc, _LEFT);
    if (solve()) return true;
    stackPop(&junc, &choose);
          m_matcher->update();
    while(!(MATCH(m_matcher, AAAAB) || MATCH(m_matcher, BAAAA))){
      m_matcher->update();
      fix();
    }
    SETMOTOR(m_motor, CRUISE_SPEED, CRUISE_SPEED);
    delay(DASH_TIME);
    SETMOTOR(m_motor, 0, 0);

    curDirect = _RIGHT;
  }
  if (_juncHas(junc, _FORE)){
    redirect(curDirect, _FORE);
    stackPush(junc, _FORE);
    if (solve()) return true;
    stackPop(&junc, &choose);
          m_matcher->update();
    while(!(MATCH(m_matcher, AAAAB) || MATCH(m_matcher, BAAAA))){
      m_matcher->update();
      fix();  
    }
    SETMOTOR(m_motor, CRUISE_SPEED, CRUISE_SPEED);
    delay(DASH_TIME);
    SETMOTOR(m_motor, 0, 0);
    
    curDirect = _BACK;
  }
  if (_juncHas(junc, _RIGHT)){
    redirect(curDirect, _RIGHT);
    stackPush(junc, _RIGHT);
    if (solve()) return true;
    stackPop(&junc, &choose);
          m_matcher->update();
    while(!(MATCH(m_matcher, AAAAB) || MATCH(m_matcher, BAAAA))){
      m_matcher->update();
      fix();  
    }

    SETMOTOR(m_motor, CRUISE_SPEED, CRUISE_SPEED);
    delay(DASH_TIME);
    SETMOTOR(m_motor, 0, 0);

    curDirect = _LEFT;
  }
  if (_juncHas(junc, _BACK)){
    redirectWithLine(curDirect, _BACK);
    return false;  
  }
  return true;
}

template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE>
void MazeSolver<TMotor, TSensor,  TLogger, STACK_SIZE>::replay(){
  m_logger.log(m_sp);
  for (uint8_t i = 0; i < m_sp; ++ i){
//    if(_juncHas(m_stackJunc[i], _LEFT)){
//      while(!(MATCH(m_matcher, AAAAB) || MATCH(m_matcher, BAAAA))){
//        m_matcher->update();
//        fix();  
//      }
//    }
//    else if(_juncHas(m_stackJunc[i], _RIGHT)){
//      while(!(MATCH(m_matcher, AAAAB) || MATCH(m_matcher, BAAAA))){
//        m_matcher->update();
//        fix();  
//      }
//    }
//    else {
//      while(!(MATCH(m_matcher, AAAAB) || MATCH(m_matcher, BAAAA))){
//        m_matcher->update();
//        fix();  
//      }
//    }

    m_logger.log("IN REPLAY\n");
    m_matcher->update();
    while(!(MATCH(m_matcher, AAAAB) || MATCH(m_matcher, BAAAA))){
      fix();  
    }

    dash(DASH_TIME);

    switch (m_stackChoose[i]){
      case _LEFT:
        turnLeft();
        break;
      case _FORE:
        break;
      case _BACK:
        //turnAround();
        // WTF! impossible!
        break;
      case _RIGHT:
        turnRight();
        break;
      default:
        break;
    }
  }  
  m_matcher->update();
  while(!(MATCH(m_matcher, AAAAB) || MATCH(m_matcher, BAAAA))){
    fix();  
  }
  SETMOTOR(m_motor, 0, 0);
  m_logger.log("SUCC");
}


template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE>
void MazeSolver<TMotor, TSensor,  TLogger, STACK_SIZE>::turnLeft(){
  SETMOTOR(m_motor, 0, 0);
  delay(100);
  SETMOTOR(m_motor, -TURN_SPEED, TURN_SPEED);
  delay(TURN_TIME);
  SETMOTOR(m_motor, 0, 0);
}

template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE>
void MazeSolver<TMotor, TSensor,  TLogger, STACK_SIZE>::turnRight(){
  SETMOTOR(m_motor, 0, 0);
  delay(100);
  SETMOTOR(m_motor, TURN_SPEED, -TURN_SPEED);
  delay(TURN_TIME);
  SETMOTOR(m_motor, 0, 0);
}

template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE>
void MazeSolver<TMotor, TSensor,  TLogger, STACK_SIZE>::turnAround(){
  SETMOTOR(m_motor, 0, 0);
  delay(100);
  SETMOTOR(m_motor, TURN_SPEED, -TURN_SPEED);
  delay(2 * TURN_TIME);
  SETMOTOR(m_motor, 0, 0);
}

template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE>
void MazeSolver<TMotor, TSensor,  TLogger, STACK_SIZE>::turnLeftWithLine(){
  SETMOTOR(m_motor, 0, 0);
  delay(100);
  SETMOTOR(m_motor, -TURN_SPEED, TURN_SPEED);
  delay(0.8 * TURN_TIME);
  do{
    m_matcher->update();  
  }while(!MATCH(m_matcher, AABAA));
  SETMOTOR(m_motor, 0, 0);
}

template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE>
void MazeSolver<TMotor, TSensor,  TLogger, STACK_SIZE>::turnRightWithLine(){
  SETMOTOR(m_motor, 0, 0);
  delay(100);
  SETMOTOR(m_motor, TURN_SPEED, -TURN_SPEED);
  delay(0.8 * TURN_TIME);
  do{
    m_matcher->update();  
  }while(!MATCH(m_matcher, AABAA));
  SETMOTOR(m_motor, 0, 0);
}

template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE>
void MazeSolver<TMotor, TSensor,  TLogger, STACK_SIZE>::turnAroundWithLine(){
  SETMOTOR(m_motor, 0, 0);
  delay(100);
  SETMOTOR(m_motor, TURN_SPEED, -TURN_SPEED);
  delay(1.2 * TURN_TIME);
  do{
    m_matcher->update();  
  }while(!MATCH(m_matcher, AABAA));
  SETMOTOR(m_motor, 0, 0);
}


template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE>
SensorMatch<TSensor, 5>* MazeSolver<TMotor, TSensor,  TLogger, STACK_SIZE>::getMatcher(){
  return m_matcher;
  
}

template <typename TMotor, typename TSensor, typename TLogger, uint8_t STACK_SIZE>
junction_t MazeSolver<TMotor, TSensor,  TLogger, STACK_SIZE>::cruise(){

  m_matcher->update();
  if(MATCH(m_matcher, AWWWA)){
    if( m_blankTime ){
      if (millis() - m_blankTime > BLANK_TIME){
        m_blankTime = 0;
        return ROADEND;   
      }
    }
    else{
      m_blankTime = millis();  
    }

    SETMOTOR(m_motor, CRUISE_SPEED, -CRUISE_SPEED);
  }
  else{
    m_blankTime = 0;

    if(m_matcher->isBlack(0) || m_matcher->isBlack(4)){
      if( ! m_debonus ){
        m_debonus = millis();  
      }
      else if(millis() - m_debonus > DEBONUS_TIME){
        m_debonus = 0;
        uint8_t lineFound = 0;
        lineFound |= m_matcher->isBlack(0) << 0;
        lineFound |= m_matcher->isBlack(4) << 4;
        auto curTime = millis();
        SETMOTOR(m_motor, CRUISE_SPEED, CRUISE_SPEED);
        while (millis() - curTime < DASH_TIME){
          m_matcher->update();
          lineFound |= m_matcher->isBlack(0) << 0;
          lineFound |= m_matcher->isBlack(4) << 4;
        }
        SETMOTOR(m_motor, 0, 0);
        delay(200); // for statbility
        m_matcher->update();

        curTime = millis();

#define TWIST_TIME 300
        SETMOTOR(m_motor, 100, -100);
        while (millis() - curTime < TWIST_TIME){
          m_matcher->update();
          lineFound |= m_matcher->isBlack(1) << 1;
          lineFound |= m_matcher->isBlack(2) << 2;
          lineFound |= m_matcher->isBlack(3) << 3;
          if (lineFound & 0b01110 ) goto LINEFOUND;
        }
        SETMOTOR(m_motor, -100, 100);
        while (millis() - curTime < 3 * TWIST_TIME){
          m_matcher->update();
          lineFound |= m_matcher->isBlack(1) << 1;
          lineFound |= m_matcher->isBlack(2) << 2;
          lineFound |= m_matcher->isBlack(3) << 3;
          if (lineFound & 0b01110 ) goto LINEFOUND;
        }
        SETMOTOR(m_motor, 100, -100);
        while (millis() - curTime < 4 * TWIST_TIME){
          m_matcher->update();
          lineFound |= m_matcher->isBlack(1) << 1;
          lineFound |= m_matcher->isBlack(2) << 2;
          lineFound |= m_matcher->isBlack(3) << 3;
          if (lineFound & 0b01110 ) goto LINEFOUND;
        }

        LINEFOUND:
        SETMOTOR(m_motor, 0, 0);
  
#define DETECT(line, mask) (((line) & (mask)) == (mask))
        if (DETECT(lineFound, 0b01110)) return EXIT;
        if (DETECT(lineFound, 0b10001) && (lineFound & 0b01110)) return CROSS;
        if (DETECT(lineFound, 0b10000) && (lineFound & 0b01110)) return RJUNC;
        if (DETECT(lineFound, 0b00001) && (lineFound & 0b01110)) return LJUNC;
        if (DETECT(lineFound, 0b10001)) return TJUNC;
        if (DETECT(lineFound, 0b10000)) return RCORN;
        if (DETECT(lineFound, 0b00001)) return LCORN;
#undef DETECT
      }
    }
    else if(MATCH(m_matcher, ABAAA)){
      SETMOTOR(m_motor, -CRUISE_SPEED, CRUISE_SPEED);
    }
    else if(MATCH(m_matcher, AAABA)){
      SETMOTOR(m_motor, CRUISE_SPEED, -CRUISE_SPEED);
    }
    else{
      SETMOTOR(m_motor, CRUISE_SPEED, CRUISE_SPEED);
    }
  }
  return NORM;
};

#undef DASH_TIME
#undef BLANK_TIME
#undef CRUISE_SPEED
#undef DEBONUS_TIME
#undef TURN_SPEED
#undef TURN_TIME

#undef _LEFT
#undef _FORE
#undef _BACK
#undef _RIGHT
#undef _juncHas

#undef JUNC_DEFS
#undef MATCH
#undef ELS_MATCH
