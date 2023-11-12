/*
 *****************************************************************************
 * Copyright by ams OSRAM AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */

#ifndef TMF882X_SHIM_H
#define TMF882X_SHIM_H

// This is the shim for the arduino uno. 
// Any define, macro and/or function herein must be adapted to match your
// target platform

// ---------------------------------------------- includes ----------------------------------------

#include <stdint.h>
#include "Arduino.h"
#include <time.h>
#include <Wire.h>


// ---------------------------------------------- defines -----------------------------------------

#define ARDUINO_MAX_I2C_TRANSFER                  32

// on the arduino uno the enable pin is connected to digital 6, interupt to digital 7
#define ENABLE_PIN                                6
#define INTERRUPT_PIN                             7

// for 2nd tmf8828 on the arduino uno the alternate enable pin is connected to digital 4, alternate interrupt to digital 5
#define ALT_ENABLE_PIN                            4
#define ALT_INTERRUPT_PIN                         5


// ---------------------------------------------- macros ------------------------------------------

// on the arduino the HEX file is placed inside the program memory region, so we need a special
// read function to access it 
#define READ_PROGRAM_MEMORY_BYTE( address )   pgm_read_byte( address )

// control the arduino digital pins for enable and interrupt
#define PIN_OUTPUT( pin )                     pinMode( (pin), OUTPUT )
#define PIN_INPUT( pin )                      pinMode( (pin), INPUT )

#define PIN_HIGH( pin )                       digitalWrite( (pin), HIGH ) 
#define PIN_LOW( pin )                        digitalWrite( (pin), LOW ) 

// to replace the arduino specific printing 
#define PRINT_CHAR(c)                         Serial.print( c )
#define PRINT_INT(i)                          Serial.print( i, DEC )
#define PRINT_INT_HEX(i)                      Serial.print( i, HEX )
#define PRINT_STR(str)                        Serial.print( str )
#define PRINT_LN()                            Serial.print( '\n' )

// Which character to use to seperate the entries in printing
#define SEPERATOR                             ','

// for clock correction insert here the number in relation to your host
#define HOST_TICKS_PER_US                     1         // host counts ticks every microsecond
#define TMF882X_TICKS_PER_US                  5         // tmf882x counts ticks 0.2 mircosecond (5x faster than host)               

// forward declaration of driver structure to avoid cyclic dependancies
typedef struct _tmf882xDriver tmf882xDriver;

// ---------------------------------------------- functions ---------------------------------------

// Function to allow to wait for some time in microseconds
// wait ... number of microseconds to wait before this functionr returns
void delay_in_microseconds( uint32_t wait );

// Function returns the current sys-tick. 
uint32_t get_sys_tick( );

// I2C transmit only function.
// reg ... the register address to write to
// buf ... pointer to a byte-array that will be transmitted
// len ... number of bytes in the buffer to transmit
void i2c_tx( uint8_t slave_addr, uint8_t reg, const uint8_t * buf, uint8_t len );

// I2C receive only function.
// reg ... the register address to read from
// buf ... pointer to a byte-array where the received bytes will be written to
// len ... number of bytes to receive 
void i2c_rx( uint8_t slave_addr, uint8_t reg, uint8_t * buf, uint8_t len );

// Function to print the results in a kind of CSV like format 
// driver ... pointer to the tmf8828 driver structure 
// data ... pointer to the result structure as defined for tmf882x
// len ... number of bytes the pointer points to
void print_results( tmf882xDriver * driver, uint8_t * data, uint8_t len );


// Function to print a histogram part in a kind of CSV like format 
// driver ... pointer to the tmf8828 driver structure 
// data ... pointer to the histogram buffer as defined for tmf882x
// len ... number of bytes the pointer points to
void print_histogram( tmf882xDriver * driver, uint8_t * data, uint8_t len );

#endif
