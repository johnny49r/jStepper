/**
 * jMacros.h
 *
 * StepperLib Macros 
 * 
 */

#ifndef _JMACROS_H
#define _JMACROS_H

#ifndef _BV
  #define _BV(PIN) (1UL << PIN)
#endif

#define bitV(b) (1UL << (b))

// Small delay for step pulse generation
#define DELAY_1_NOP __asm__("nop\n\t")  // on 16 Mhz cpu - 62.5ns

// Bracket code that shouldn't be interrupted
#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  uint8_t _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif

// Macros for bit masks
#define TEST(n,b) (((n)&_BV(b))!=0)
#define SBI(n,b) (n |= _BV(b))
#define CBI(n,b) (n &= ~_BV(b))
#define SET_BIT(n,b,value) (n) ^= ((-value)^(n)) & (_BV(b))

#define SETb(x,y) x |= (1 << y)
#define CLRb(x,y) x &= ~(1<< y)
#define CMPb(x,y) x & (1 << y)

// Macros for math 
#define LONG_ERROR -2147483648
#ifndef PI
  #define PI 3.14159265358979323846
#endif
#define ONE_MILLION 1000000.0
#define CEILING(x,y) (((x) + (y) - 1) / (y))
#define ATAN2(y, x) atan2(y, x)
#define FABS(x)     fabs(x)
#define POW(x, y)   pow(x, y)
#define SQRT(x)     sqrt(x)
#define CEIL(x)     ceil(x)
#define FLOOR(x)    floor(x)
#define LROUND(x)   lround(x)
#define FMOD(x, y)  fmod(x, y)
#define HYPOT(x,y)  SQRT(HYPOT2(x,y))

// Macros to contrain values
#define NOTLESS(v,n) do{ if (v < n) v = n; }while(0)
#define NOTMORE(v,n) do{ if (v > n) v = n; }while(0)
#define IS_WITHIN(V,L,H) ((V) >= (L) && (V) <= (H))

// Macros for chars
#define IS_NUMERIC(a) WITHIN(a, '0', '9')     // test for numeric vals 0-9
#define IS_ALPHA_UC(a) WITHIN(a, 'A', 'Z')    // test for upper case alpha chars
#define IS_ALPHA_LC(a) WITHIN(a, 'a', 'z')    // test for lower case alpha chars
#define IS_DECIMAL(a) (IS_NUMERIC(a) || a == '.')
#define IS_NUMERIC_SIGNED(a) (IS_NUMERIC(a) || (a) == '-' || (a) == '+')
#define IS_DECIMAL_SIGNED(a) (IS_DECIMAL(a) || (a) == '-' || (a) == '+')


//#############################################################
// macros for performing fast IO
//
#define AVR_ATmega2560_FAMILY (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__))
#define AVR_ATmega328_FAMILY (defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328p__))

/**
 * Include Ports and Functions
 */
#if AVR_ATmega328_FAMILY
  #include "pinmap_328.h"
#elif AVR_ATmega2560_FAMILY
  #include "pinmap_2560.h"
#else
  #error "Pins for this chip not defined in Arduino.h!"
#endif


/**
 * Magic I/O routines
 *
 * Now you can simply SET_OUTPUT(PIN); WRITE(PIN, HIGH); WRITE(PIN, LOW);
 *
 */

#define _READ(IO) ((bool)(DIO ## IO ## _RPORT & _BV(DIO ## IO ## _PIN)))

// On some boards pins > 0x100 are used. These are not converted to atomic actions. A critical section is needed.

#define _WRITE_NC(IO, v)  do { if (v) {DIO ##  IO ## _WPORT |= _BV(DIO ## IO ## _PIN); } else {DIO ##  IO ## _WPORT &= ~_BV(DIO ## IO ## _PIN); }; } while (0)

#define _WRITE_C(IO, v)   do { if (v) { \
                                         CRITICAL_SECTION_START; \
                                         {DIO ##  IO ## _WPORT |= _BV(DIO ## IO ## _PIN); } \
                                         CRITICAL_SECTION_END; \
                                       } \
                                       else { \
                                         CRITICAL_SECTION_START; \
                                         {DIO ##  IO ## _WPORT &= ~_BV(DIO ## IO ## _PIN); } \
                                         CRITICAL_SECTION_END; \
                                       } \
                                     } \
                                     while (0)

#define _WRITE(IO, v) do { if (&(DIO ## IO ## _RPORT) >= (uint8_t *)0x100) {_WRITE_C(IO, v); } else {_WRITE_NC(IO, v); }; } while (0)

#define _TOGGLE(IO) do {DIO ## IO ## _RPORT ^= _BV(DIO ## IO ## _PIN); } while (0)

#define _SET_INPUT(IO) do {DIO ## IO ## _DDR &= ~_BV(DIO ## IO ## _PIN); } while (0)
#define _SET_OUTPUT(IO) do {DIO ## IO ## _DDR |= _BV(DIO ## IO ## _PIN); } while (0)

#define _GET_INPUT(IO) ((DIO ## IO ## _DDR & _BV(DIO ## IO ## _PIN)) == 0)
#define _GET_OUTPUT(IO) ((DIO ## IO ## _DDR & _BV(DIO ## IO ## _PIN)) != 0)
#define _GET_TIMER(IO) (DIO ## IO ## _PWM)

#define READ(IO) _READ(IO)
#define WRITE(IO,V) _WRITE(IO,V)
#define WRITE_NC(IO,V) _WRITE_NC(IO,V)  // 2560 doesn't have 100 pins
#define TOGGLE(IO) _TOGGLE(IO)

#define SET_INPUT(IO) _SET_INPUT(IO)
#define SET_INPUT_PULLUP(IO) do{ _SET_INPUT(IO); _WRITE(IO, HIGH); }while(0)
#define SET_OUTPUT(IO) _SET_OUTPUT(IO)

#define GET_INPUT(IO) _GET_INPUT(IO)
#define GET_OUTPUT(IO) _GET_OUTPUT(IO)
#define GET_TIMER(IO) _GET_TIMER(IO)

#define OUT_WRITE(IO, v) do{ SET_OUTPUT(IO); WRITE(IO, v); }while(0)

/*
 * jStepper specific I/O
 */
#define GEN_STEP_PULSE(IO) do{WRITE(IO, STEP_PULSE_ASSERT); DELAY_1_NOP; WRITE(IO, !STEP_PULSE_ASSERT); }while(0)
#define SET_STEP_DIRECTION(IO, dir) WRITE(IO, dir)
#define SET_STEP_ENABLE(IO, enab) WRITE(IO, enab)
 



#endif // JMACROS_H


