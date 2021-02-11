#if !defined(pulsegenerator_h)
#define pulsegenerator_h

#include <Arduino.h>
#include <pins_arduino.h>
#define	GPIO2_PREFER_SPEED	1
#include <DIO2.h>

#include "PulseGenerator_config.h"

#if defined(PG_USE_ALL_TIMERS)
#define PG_USE_TIMER0
#define PG_USE_TIMER1
#define PG_USE_TIMER2
#define PG_USE_TIMER3
#define PG_USE_TIMER4
#endif

typedef void SetEdgeFunction(int v);

class TimerComparator  {
public:
    int pin;
    unsigned int timerNo;
    unsigned int comparatorNo;     //0=A, 1=B 2=C
    unsigned int maxTicks = 0;
    bool assigned = false;
    SetEdgeFunction *setEdge;
    TimerComparator(int pin, int timerNo, int comparatorNo, unsigned int maxTicks, SetEdgeFunction *setEdge){
        this->pin = pin;
        this->timerNo = timerNo;
        this->comparatorNo = comparatorNo;
        this->maxTicks = maxTicks;
        this->setEdge = setEdge;
   }

private:
};

class PulseGenerator;

typedef void CallbackFunction(PulseGenerator *pg);

class PulseGenerator {

public:
    PulseGenerator() ;
    void setPin(int pin);
    bool init();  // Initialise timers etc.
    void setNextPulse(int microSecDelay, int newState); // Set delay and next state
    void setHandler(CallbackFunction *isr) { this->isr = isr; }; // Set interrupt callback
    void interrupt(); // INTERNAL interrupt routine, do not call
    unsigned int periodTicks = 100;  // Number of timer ticks till next state change / interrupt
    int nextState = 0;                // State to be adopted after delay
    unsigned int remainingTicks = 0;  // Number of timer ticks still outstanding
    unsigned int maxTicks = 0;  // Maximum number of ticks in timer register.

private:
    TimerComparator *comparatorUnit = 0;
    int timerNo = 0;
    int comparatorNo = 0;
    int pin = -1;
    GPIO_pin_t dio2Pin;
    bool digIoMode = false;
    CallbackFunction *isr = 0;
    SetEdgeFunction *setEdge = 0; // Function for setting next timer edge generation
};

#endif