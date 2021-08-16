// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/

#include <ZumoShieldEncoders.h>
#include <FastGPIO.h>
#include <avr/interrupt.h>
#include <Arduino.h>

#define LEFT_XOR   8
#define LEFT_B     IO_E2
#define RIGHT_XOR  7
#define RIGHT_B    23
#define RIGHT_CNT_FLAG 0x0001
#define LEFT_CNT_FLAG  0x0002

static volatile bool lastLeftA;
static volatile bool lastLeftB;
static volatile bool lastRightA;
static volatile bool lastRightB;

static volatile bool errorLeft;
static volatile bool errorRight;

// These count variables are uint16_t instead of int16_t because
// signed integer overflow is undefined behavior in C++.
static volatile uint16_t countLeft;
static volatile uint16_t countRight;
// int countLeft=0;
// int countRight=0;
int flag = 0;

ISR(TIMER1_CAPT_vect)
{ 
  bit_is_clear(TIFR1, ICF1);
  if (digitalRead(5) == HIGH) {
    countRight++;
  } else {
    countRight--;
  }
  flag |= RIGHT_CNT_FLAG;
}

ISR(TIMER3_CAPT_vect)
{ 
  bit_is_clear(TIFR3, ICF3);
  if (digitalRead(11) == LOW) {
    countLeft++;
  } else {
    countLeft--;
  }
  flag |= LEFT_CNT_FLAG;
}

// static void rightISR()
// {
//     bool newRightB = FastGPIO::Pin<RIGHT_B>::isInputHigh();
//     bool newRightA = FastGPIO::Pin<RIGHT_XOR>::isInputHigh() ^ newRightB;

//     countRight += (newRightA ^ lastRightB) - (lastRightA ^ newRightB);

//     if((lastRightA ^ newRightA) & (lastRightB ^ newRightB))
//     {
//         errorRight = true;
//     }

//     lastRightA = newRightA;
//     lastRightB = newRightB;
// }

void ZumoShieldEncoders::init2()
{
//     // Set the pins as pulled-up inputs.
//     FastGPIO::Pin<LEFT_XOR>::setInputPulledUp();
//     FastGPIO::Pin<LEFT_B>::setInputPulledUp();
//     FastGPIO::Pin<RIGHT_XOR>::setInputPulledUp();
//     FastGPIO::Pin<RIGHT_B>::setInputPulledUp();

//     // Enable pin-change interrupt on PB4 for left encoder, and disable other
//     // pin-change interrupts.
//     PCICR = (1 << PCIE0);
//     PCMSK0 = (1 << PCINT4);
//     PCIFR = (1 << PCIF0);  // Clear its interrupt flag by writing a 1.
      /*
       * ICP1はアナログコンパレータと機能を兼用しているので
       * それをDISABLEとする。
       * 「0」でENABLE、「1」でDISABLE
       */
      ACSR = 0x80;
      ADCSRB = 0x00;
      DIDR1 = 0x00;
      /*
       * ICP1(インプットキャプチャー)の設定
       */
      TCCR1A= 0x00;
      TCCR1B = 0x41;  // Disable Input Capture Noise Canceler
                      // Input Capture Edge : RISE
      TIMSK1 = 0x20;  // Enable Input Capture Interrupt
      /*
       * ICP3(インプットキャプチャー)の設定
       */
      TCCR3A= 0x00;
      TCCR3B = 0x41;  // Disable Input Capture Noise Canceler
                      // Input Capture Edge : RISE
      TIMSK3 = 0x20;  // Enable Input Capture Interrupt
      /*
   * DEBUGにシリアルモニタを使用
   */
    // Enable interrupt on PE6 for the right encoder.  We use attachInterrupt
    // instead of defining ISR(INT6_vect) ourselves so that this class will be
    // compatible with other code that uses attachInterrupt.
    //attachInterrupt(4, rightISR, CHANGE);

    // Initialize the variables.  It's good to do this after enabling the
    // interrupts in case the interrupts fired by accident as we were enabling
    // them.
//     lastLeftB = FastGPIO::Pin<LEFT_B>::isInputHigh();
//     lastLeftA = FastGPIO::Pin<LEFT_XOR>::isInputHigh() ^ lastLeftB;
    countLeft = 0;
    errorLeft = 0;

//     lastRightB = FastGPIO::Pin<RIGHT_B>::isInputHigh();
//     lastRightA = FastGPIO::Pin<RIGHT_XOR>::isInputHigh() ^ lastRightB;
    countRight = 0;
    errorRight = 0;
}

int16_t ZumoShieldEncoders::getCountsLeft()
{
    init();

    cli();
    int16_t counts = countLeft;
    sei();
    return counts;
}

int16_t ZumoShieldEncoders::getCountsRight()
{
    init();

    cli();
    int16_t counts = countRight;
    sei();
    return counts;
}

int16_t ZumoShieldEncoders::getCountsAndResetLeft()
{
    init();

    cli();
    int16_t counts = countLeft;
    countLeft = 0;
    sei();
    return counts;
}

int16_t ZumoShieldEncoders::getCountsAndResetRight()
{
    init();

    cli();
    int16_t counts = countRight;
    countRight = 0;
    sei();
    return counts;
}

bool ZumoShieldEncoders::checkErrorLeft()
{
    init();

    bool error = errorLeft;
    errorLeft = 0;
    return error;
}

bool ZumoShieldEncoders::checkErrorRight()
{
    init();

    bool error = errorRight;
    errorRight = 0;
    return error;
}
