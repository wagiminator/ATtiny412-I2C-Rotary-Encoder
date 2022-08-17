// ===================================================================================
// Project:   Rotary Encoder with I2C Interface based on ATtiny202/212/402/412
// Version:   v1.0
// Year:      2022
// Author:    Stefan Wagner
// Github:    https://github.com/wagiminator
// EasyEDA:   https://easyeda.com/wagiminator
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// This is a simple implementation of the rotary encoder with I2C interface.
//
// Wiring:
// -------
//                       +-\/-+
//                 Vdd  1|°   |8  GND
//   ENC B --- TXD PA6  2|    |7  PA3 AIN3 -------- ENC SW
//   ENC A --- RXD PA7  3|    |6  PA0 AIN0 UPDI --- UPDI
// I2C SDA --- SDA PA1  4|    |5  PA2 AIN2 SCL ---- I2C SCL
//                       +----+
//
// Compilation Settings:
// ---------------------
// Core:    megaTinyCore (https://github.com/SpenceKonde/megaTinyCore)
// Board:   ATtiny412/402/212/202
// Chip:    Choose the chip that is installed on the device
// Clock:   10 MHz internal
//
// Leave the rest on default settings. Don't forget to "Burn bootloader". 
// Compile and upload the code.
//
// No Arduino core functions or libraries are used. To compile and upload without
// Arduino IDE download AVR 8-bit toolchain at:
// https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers
// and extract to tools/avr-gcc. Use the makefile to compile and upload.
//
// Fuse Settings: 0:0x00 1:0x00 2:0x02 4:0x00 5:0xC5 6:0x04 7:0x00 8:0x00
//
// Operating Instructions:
// -----------------------
// The device has four 16-bit and two 8-bit registers that can be read and written.
// The 16-bit registers are signed and the least significant byte is always
// transmitted first. With each access (reading or writing), the registers are 
// always transferred starting with the first in the following order:
// 1. Encoder wheel value (16-bit)
// 2. Encoder switch state (8-bit, 0=switch released, 1=switch pressed)
// 3. Encoder wheel value loop flag (8-bit, 0=do not loop, 1=loop around)
// 4. Encoder wheel minimum value (16-bit)
// 5. Encoder wheel maximum value (16-bit)
// 6. Encoder wheel value change step (16-bit)


// ===================================================================================
// Libraries, Definitions and Macros
// ===================================================================================

// Libraries
#include <avr/io.h>                 // for GPIO
#include <avr/interrupt.h>          // for interrupts

// Pin assignments
#define PIN_SDA       PA1           // I2C Serial Data
#define PIN_SCL       PA2           // I2C Serial Clock
#define PIN_ENC_SW    PA3           // rotary encoder switch
#define PIN_ENC_B     PA6           // rotary encoder B
#define PIN_ENC_A     PA7           // rotary encoder A

// Firmware parameters
#define I2C_ADDR      0x36          // I2C address of the device

// Pin manipulation macros
enum {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};    // enumerate pin designators
#define pinInput(x)     VPORTA.DIR &= ~(1<<(x))   // set pin to INPUT
#define pinOutput(x)    VPORTA.DIR |=  (1<<(x))   // set pin to OUTPUT
#define pinLow(x)       VPORTA.OUT &= ~(1<<(x))   // set pin to LOW
#define pinHigh(x)      VPORTA.OUT |=  (1<<(x))   // set pin to HIGH
#define pinToggle(x)    VPORTA.IN  |=  (1<<(x))   // TOGGLE pin
#define pinRead(x)      (VPORTA.IN &   (1<<(x)))  // READ pin
#define pinPullup(x)    (&PORTA.PIN0CTRL)[x] |= PORT_PULLUPEN_bm
#define pinDisable(x)   (&PORTA.PIN0CTRL)[x] |= PORT_ISC_INPUT_DISABLE_gc
#define pinIntEn(x)     (&PORTA.PIN0CTRL)[x] |= PORT_ISC_BOTHEDGES_gc
#define pinIntDis(x)    (&PORTA.PIN0CTRL)[x] &= ~PORT_ISC_gm
#define pinIntClr(x)    VPORTA.INTFLAGS = (1<<(x))

// ===================================================================================
// I2C Slave Implementation
// ===================================================================================

// I2C slave command macros
#define I2C_complete() TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc
#define I2C_response() TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc
#define I2C_sendACK()  TWI0.SCTRLB = TWI_ACKACT_ACK_gc  | TWI_SCMD_RESPONSE_gc
#define I2C_sendNACK() TWI0.SCTRLB = TWI_ACKACT_NACK_gc | TWI_SCMD_RESPONSE_gc
#define I2C_put(x)     TWI0.SDATA  = (x)
#define I2C_get()      TWI0.SDATA

// I2C slave status macros
#define I2C_isAddr()  ((TWI0.SSTATUS & TWI_APIF_bm) && (TWI0.SSTATUS & TWI_AP_bm))
#define I2C_isData()  (TWI0.SSTATUS & TWI_DIF_bm)
#define I2C_isStop()  ((TWI0.SSTATUS & TWI_APIF_bm) && (~TWI0.SSTATUS & TWI_AP_bm))
#define I2C_isIn()    (~TWI0.SSTATUS & TWI_DIR_bm)
#define I2C_isOut()   (TWI0.SSTATUS & TWI_DIR_bm)

// I2C slave registers
uint8_t I2C_REG[10];                              // register array
uint8_t I2C_REG_ptr;                              // register pointer
volatile uint8_t I2C_REG_changed = 0;             // register change flag
volatile uint8_t I2C_busy = 0;                    // I2C busy flag

// I2C slave init
void I2C_init(void) {
  TWI0.SADDR  = I2C_ADDR << 1;                    // set address (LSB is R/W bit)
  TWI0.SCTRLA = TWI_DIEN_bm                       // data interrupt enable
              | TWI_APIEN_bm                      // address or stop interrupt enable
              | TWI_PIEN_bm                       // stop interrupt enable
              | TWI_ENABLE_bm;                    // enable I2C slave
}

// I2C slave interrupt service routine
ISR(TWI0_TWIS_vect) { 
  // Address match interrupt handler
  if(I2C_isAddr()) {                              // address match?
    I2C_sendACK();                                // send ACK to master
    I2C_REG_ptr = 0;                              // reset register pointer
    I2C_busy = 1;                                 // set I2C busy flag
    return;                                       // quit ISR
  }
  
  // Data interrupt handler
  if(I2C_isData()) {                              // data transmission?
    if(I2C_isOut()) {                             // slave writing to master?
      I2C_put(I2C_REG[I2C_REG_ptr]);              // send register value to master
      I2C_response();                             // no ACK needed here
    } else {                                      // slave reading from master?
      I2C_REG[I2C_REG_ptr] = I2C_get();           // read register value from master
      I2C_sendACK();                              // send ACK to master
      I2C_REG_changed = 1;                        // set register changed flag
    }
    if(++I2C_REG_ptr >= sizeof(I2C_REG))          // increase pointer...
      I2C_REG_ptr = 0;                            // ...or wrap around
    return;                                       // quit ISR
  }

  // Stop condition interrupt handler
  if(I2C_isStop()) {                              // stop condition?
    I2C_complete();                               // complete transaction
    I2C_busy = 0;                                 // clear I2C busy flag
  }
}

// ===================================================================================
// Rotary Encoder Implementation using Pin Change Interrupt
// ===================================================================================

// Global variables
volatile uint8_t  ENC_a0, ENC_b0, ENC_ab0, ENC_loop, ENC_changed;
volatile int16_t  ENC_count, ENC_countMin, ENC_countMax, ENC_countStep;

// Init rotary encoder
void ENC_init(void) {
  pinPullup(PIN_ENC_A);                           // enable pullup on encoder pins ...
  pinPullup(PIN_ENC_B);
  pinPullup(PIN_ENC_SW);
}

// Set parameters for rotary encoder
void ENC_set(int16_t rmin, int16_t rmax, int16_t rstep, int16_t rval, uint8_t rloop) {
  pinIntDis(PIN_ENC_A);                           // disable interrupt during setup
  ENC_countMin  = rmin * 2;                       // min wheel value
  ENC_countMax  = rmax * 2;                       // max wheel value
  ENC_countStep = rstep;                          // wheel steps (negative if CCW)
  ENC_count     = rval * 2;                       // actual wheel value
  ENC_loop      = rloop;                          // loop if true
  ENC_a0  = !pinRead(PIN_ENC_A);                  // set initial internal values ...
  ENC_b0  = !pinRead(PIN_ENC_B);
  ENC_ab0 = (ENC_a0 == ENC_b0);
  pinIntEn(PIN_ENC_A);                            // enable pin change interrupt on ENC A
}

// Pin change interrupt service routine for rotary encoder wheel
ISR(PORTA_PORT_vect) {
  pinIntClr(PIN_ENC_A);                           // clear interrupt flag
  uint8_t a = !pinRead(PIN_ENC_A);                // get A state
  uint8_t b = !pinRead(PIN_ENC_B);                // get B state
  if(a != ENC_a0) {                               // A changed?
    ENC_a0 = a;
    if(b != ENC_b0) {                             // B changed?
      ENC_b0 = b;
      ENC_count += (a == b) ? -ENC_countStep : ENC_countStep;
      if((a == b) != ENC_ab0) ENC_count += (a == b) ? -ENC_countStep : ENC_countStep;
      if(ENC_count < ENC_countMin) ENC_count = (ENC_loop) ? ENC_countMax : ENC_countMin;
      if(ENC_count > ENC_countMax) ENC_count = (ENC_loop) ? ENC_countMin : ENC_countMax;
      ENC_ab0 = (a == b);
      ENC_changed = 1;
    }
  }
}

// ===================================================================================
// Main Function
// ===================================================================================

int main(void) {
  // Local variables
  int16_t rval, rmin, rmax, rstep;                // for handling encoder parameters

  // Setup
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 1);         // set clock frequency to 10 MHz
  I2C_init();                                     // setup I2C
  ENC_init();                                     // setup rotary encoder
  sei();                                          // enable interrupts

  // Loop
  while(1) {
    // Update rotary encoder settings if I2C registers have changed
    if(I2C_REG_changed && !I2C_busy) {
      cli();                                      // disable interrupts for atomic ops
      rval  = ((uint16_t)I2C_REG[1] << 8) | (uint16_t)I2C_REG[0];
      rmin  = ((uint16_t)I2C_REG[5] << 8) | (uint16_t)I2C_REG[4];
      rmax  = ((uint16_t)I2C_REG[7] << 8) | (uint16_t)I2C_REG[6];
      rstep = ((uint16_t)I2C_REG[9] << 8) | (uint16_t)I2C_REG[8];
      I2C_REG_changed = 0;                        // clear register changed flag
      sei();                                      // enable interrupts again
      ENC_set(rmin, rmax, rstep, rval, I2C_REG[3]); // set rotary encoder
    }

    // Update I2C registers if rotary encoder wheel value has changed
    if(ENC_changed && !I2C_REG_changed && !I2C_busy) {
      cli();                                      // disable interrupts for atomic ops
      rval = ENC_count / 2;                       // get encoder counter value
      I2C_REG[0] = (uint8_t)(rval & 0xff);        // store low byte to register
      I2C_REG[1] = (uint8_t)(rval >> 8);          // store high byte to register
      ENC_changed = 0;                            // clear encoder changed flag
      sei();                                      // enable interrupts again
    }

    // Update rotary encoder switch register
    I2C_REG[2] = !pinRead(PIN_ENC_SW);
  }
}
