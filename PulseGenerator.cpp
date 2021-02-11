#include "PulseGenerator.h"

#define DeclareSetEdge(n,x) \
static void setEdge ## n ## x (int v) {\
    if (v)\
        bitSet(TCCR ## n ## A, COM ## n ## x ## 0);\
    else\
        bitClear(TCCR ## n ## A, COM ## n ## x ## 0);\
}

#if defined(PG_USE_TIMER0)
DeclareSetEdge(0,A);
DeclareSetEdge(0,B);
#endif

#if defined(PG_USE_TIMER1)
DeclareSetEdge(1,A);
DeclareSetEdge(1,B);
#endif

#if defined(PG_USE_TIMER2)
DeclareSetEdge(2,A);
DeclareSetEdge(2,B);
#endif

#if defined (ARDUINO_AVR_MEGA2560)

#if defined(PG_USE_TIMER1)
DeclareSetEdge(1,C);
#endif

#if defined(PG_USE_TIMER3)
DeclareSetEdge(3,A);
DeclareSetEdge(3,B);
DeclareSetEdge(3,C);
#endif

#if defined(PG_USE_TIMER4)
DeclareSetEdge(4,A);
DeclareSetEdge(4,B);
DeclareSetEdge(4,C);
#endif

#endif

// List of comparator objects and associated digital pins.
static TimerComparator comparators[] = 
{
// Registers, sorted according to usefulness
// (16-bit ones first, 8bit and Timer0 last).
#if defined(ARDUINO_AVR_UNO)
    #if defined(PG_USE_TIMER1)
    TimerComparator(9, 1, 0, 65535, setEdge1A),
    TimerComparator(10, 1, 1, 65535, setEdge1B),
    #endif
    #if defined(PG_USE_TIMER2)
    TimerComparator(11, 2, 0, 255, setEdge2A),
    TimerComparator(3, 2, 1, 255, setEdge2B),
    #endif
    #if defined(PG_USE_TIMER0)
    TimerComparator(6, 0, 0, 255, setEdge0A),
    TimerComparator(5, 0, 1, 255, setEdge0B),
    #endif
#elif defined(ARDUINO_AVR_MEGA2560)
    #if defined(PG_USE_TIMER1)
    TimerComparator(11, 1, 0, 65535, setEdge1A),
    TimerComparator(12, 1, 1, 65535, setEdge1B),
    TimerComparator(13, 1, 2, 65535, setEdge1C),
    #endif
    #if defined(PG_USE_TIMER3)
    TimerComparator(5, 3, 0, 65535, setEdge3A),
    TimerComparator(2, 3, 1, 65535, setEdge3B),
    TimerComparator(3, 3, 2, 65535, setEdge3C),
    #endif
    #if defined(PG_USE_TIMER4)
    TimerComparator(6, 4, 0, 65535, setEdge4A),
    TimerComparator(7, 4, 1, 65535, setEdge4B),
    TimerComparator(8, 4, 2, 65535, setEdge4C),
    #endif
    #if defined(PG_USE_TIMER2)
    TimerComparator(10, 2, 0, 255, setEdge2A),
    TimerComparator(9, 2, 1, 255, setEdge2B),
    #endif
    #if defined(PG_USE_TIMER0)
    TimerComparator(13, 0, 0, 255, setEdge0A),
    TimerComparator(4, 0, 1, 255, setEdge0B),
    #endif
#endif
};

// Pulse Generator object references, for use in the interrupt handlers.
static PulseGenerator *pulseGen0A __attribute__((unused)) = 0;
static PulseGenerator *pulseGen0B __attribute__((unused)) = 0;
static PulseGenerator *pulseGen1A __attribute__((unused)) = 0;
static PulseGenerator *pulseGen1B __attribute__((unused)) = 0;
static PulseGenerator *pulseGen2A __attribute__((unused)) = 0;
static PulseGenerator *pulseGen2B __attribute__((unused)) = 0;
#if defined(ARDUINO_AVR_MEGA2560)
static PulseGenerator *pulseGen1C __attribute__((unused)) = 0;
static PulseGenerator *pulseGen3A __attribute__((unused)) = 0;
static PulseGenerator *pulseGen3B __attribute__((unused)) = 0;
static PulseGenerator *pulseGen3C __attribute__((unused)) = 0;
static PulseGenerator *pulseGen4A __attribute__((unused)) = 0;
static PulseGenerator *pulseGen4B __attribute__((unused)) = 0;
static PulseGenerator *pulseGen4C __attribute__((unused)) = 0;
#endif


// This routine is the constructor for the object.  
PulseGenerator::PulseGenerator() {
}

// This sets the pin.  Should be called before any init() calls.  It checks to see if the
// specified pin can be controlled directly by a timer comparator, and if so 
// allocates that comparator.  Otherwise, it defers the allocation until the
// init() function is called.
void PulseGenerator::setPin(int pin) {
    this->pin = pin;
    this->dio2Pin = Arduino_to_GPIO_pin(pin);
    // Find comparator unit associated with the specified pin.
    for (unsigned int i=0; i<sizeof(comparators)/sizeof(TimerComparator); i++) {
        TimerComparator *thisComparator = &comparators[i];
        if (this->pin == thisComparator->pin && !thisComparator->assigned) {
            // Allocate this comparator unit
            thisComparator->assigned = true;
            this->comparatorUnit = thisComparator;
            this->digIoMode = false;
            break;
        }
    }
}

#define ComparatorSetup(n,x) do {\
    bitClear(TCCR ## n ## A, WGM ## n ## 1);\
    bitClear(TCCR ## n ## A, WGM ## n ## 0);\
    bitClear(TCCR ## n ## B, WGM ## n ## 2);\
    if (n != 0) {\
        bitClear(TCCR ## n ## B, CS ## n ## 2);\
        bitSet(TCCR ## n ## B, CS ## n ## 1);\
        bitClear(TCCR ## n ## B, CS ## n ## 0);\
    }\
    pulseGen ## n ## x = this;\
    bitSet(TCCR ## n ## A, COM ## n ## x ## 1);\
    bitSet(TIMSK ## n, OCIE ## n ## x);\
    OCR ## n ## x = TCNT ## n + this->periodTicks;\
} while(0)


// This routine completes the setup of the PulseGenerator object.  First it checks if
// the comparator has already been allocated; if not then it looks for the first 
// available one.  If one is found, then it sets it up and returns true.
// Otherwise it returns false. 
bool PulseGenerator::init() {
    TimerComparator *thisComparator = this->comparatorUnit;
    if (thisComparator == 0) {
        // Look for a free comparator unit
        for (unsigned int i=0; i<sizeof(comparators)/sizeof(TimerComparator); i++) {
            thisComparator = &comparators[i];
            if (!thisComparator->assigned) {
                this->comparatorUnit = thisComparator;
                thisComparator->assigned = true;
                this->digIoMode = true;
                break;
            }
        }
    }
    if (thisComparator == 0) return false;

    this->timerNo = thisComparator->timerNo;
    this->comparatorNo = thisComparator->comparatorNo;
    this->maxTicks = thisComparator->maxTicks;
    this->setEdge = thisComparator->setEdge;

    Serial.print("Constructor pin=");
    Serial.print(this->pin);
    Serial.print(", Comparator=");
    Serial.print(thisComparator->timerNo);
    Serial.print((char)('A' + thisComparator->comparatorNo));
    Serial.print(", digIoMode=");
    Serial.println(this->digIoMode);

    pinMode2f(this->dio2Pin, OUTPUT);
    #if defined(PG_USE_TIMER0)
    if (this->timerNo == 0 && this->comparatorNo == 0)
        ComparatorSetup(0,A);
    else 
    if (this->timerNo == 0 && this->comparatorNo == 1)
        ComparatorSetup(0,B);
    else 
    #endif
    #if defined(PG_USE_TIMER1)
    if (this->timerNo == 1 && this->comparatorNo == 0)
        ComparatorSetup(1,A);
    else 
    if (this->timerNo == 1 && this->comparatorNo == 1)
        ComparatorSetup(1,B);
    else 
    #if defined(ARDUINO_ARCH_MEGA2560)
    if (this->timerNo == 1 && this->comparatorNo == 2)
        ComparatorSetup(1,C);
    else 
    #endif
    #endif
    #if defined(PG_USE_TIMER2)
    if (this->timerNo == 2 && this->comparatorNo == 0)
        ComparatorSetup(2,A);
    else 
    if (this->timerNo == 2 && this->comparatorNo == 1)
        ComparatorSetup(2,B);
    else 
    #endif
    #if defined(ARDUINO_ARCH_MEGA2560)
    #if defined(PG_USE_TIMER3)
    if (this->timerNo == 3 && this->comparatorNo == 0)
        ComparatorSetup(3,A);
    else 
    if (this->timerNo == 3 && this->comparatorNo == 1)
        ComparatorSetup(3,B);
    else 
    if (this->timerNo == 3 && this->comparatorNo == 2)
        ComparatorSetup(3,C);
    else 
    #endif
    #if defined(PG_USE_TIMER4)
    if (this->timerNo == 4 && this->comparatorNo == 0)
        ComparatorSetup(4,A);
    else 
    if (this->timerNo == 4 && this->comparatorNo == 1)
        ComparatorSetup(4,B);
    else 
    if (this->timerNo == 4 && this->comparatorNo == 2)
        ComparatorSetup(4,C);
    else
    #endif
    #endif
    { ; }
    
    return true;
}

// Set up pulse generator so that after the specified delay, the digital output
// is set to the specified new state.  This is done by setting the timer for
// the desired delay.  If the output is directly controlled by the timer, the
// desired state is set in the timer.  Otherwise, it is stored ready for the 
// interrupt code to action it.
void PulseGenerator::setNextPulse(int microSecDelay, int newState) {
    if (this->timerNo == 0)
        this->periodTicks = microSecDelay / 4;    
    else 
        this->periodTicks = microSecDelay * 2;
    this->nextState = newState;

    if (!this->digIoMode) {
        this->setEdge(newState);
    }
}


// Interrupt callback routine.  This writes the digital output value (if the 
// timer was not set up to do it directly), and then calls the user interrupt 
// routine (if configured).
void PulseGenerator::interrupt() {
    if (this->digIoMode)
        digitalWrite2f(this->dio2Pin, this->nextState);
    if (this->isr)
        (this->isr)(this);
}


// The following routines are the interrupt handlers for the 
// timer compare interrupts.  For flexibility, the handlers associated
// with each timer may be suppressed by undefining the related macro 
// PG_USE_TIMERn.2
// The interrupt routine calls the associated PulseGenerator object's 
// interrupt routine and then updates the compare register contents.

#define DeclareISR(n, x)\
ISR(TIMER ## n ## _COMP ## x ## _vect) {\
    if (pulseGen ## n ## x) {\
        pulseGen ## n ## x->interrupt();\
        OCR ## n ## x += pulseGen ## n ## x->periodTicks;\
    }\
}

#if defined(PG_USE_TIMER0)
DeclareISR(0,A);
DeclareISR(0,B);
#endif

#if defined(PG_USE_TIMER1)
DeclareISR(1,A);
DeclareISR(1,B);
#endif

#if defined(PG_USE_TIMER2)
DeclareISR(2,A);
DeclareISR(2,B);
#endif

#if defined(ARDUINO_AVR_MEGA2560)

#if defined(PG_USE_TIMER1)
DeclareISR(1,C);
#endif

#if defined(PG_USE_TIMER3)
DeclareISR(3,A);
DeclareISR(3,B);
DeclareISR(3,C);
#endif

#if defined(PG_USE_TIMER4)
DeclareISR(4,A);
DeclareISR(4,B);
DeclareISR(4,C);
#endif

#endif