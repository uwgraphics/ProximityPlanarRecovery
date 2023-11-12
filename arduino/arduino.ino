/*
 *****************************************************************************
 * Copyright by ams OSRAM AG                                                 *
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
//
// tmf8820/1 arduino uno driver example program
//

// ---------------------------------------------- includes ----------------------------------------

#include <Wire.h>
#include "registers_i2c.h"
#include "tmf882x.h"
#include "tmf882x_calib.h"


// ---------------------------------------------- defines -----------------------------------------

#define UART_BAUD_RATE              1000000
#define VERSION                     17

// arduino uno can only do 400 KHz I2C
// actually, it seems like it can do 1 MHz - 1000000
// Sparkfun board seems to be unstable above 400000
#define I2C_CLK_SPEED               400000


// how much logging we want - info prints the FW number
#define MY_LOG_LEVEL                LOG_LEVEL_INFO

// tmf states
#define TMF882X_STATE_DISABLED      0
#define TMF882X_STATE_STANDBY       1     
#define TMF882X_STATE_STOPPED       2
#define TMF882X_STATE_MEASURE       3
#define TMF882X_STATE_ERROR         4

// convendience macro to have a pointer to the driver structure
#define TMF882X_A                   ( &tmf882x )

// ---------------------------------------------- user config ---------------------------------------
#define SPAD_MAP TMF882X_COM_SPAD_MAP_ID__spad_map_id__map_no_1
#define KILO_ITERS 4000 //see https://ams.com/documents/20143/6015057/TMF882X_DS000693_5-00.pdf pg. 21 for kilo_iters relation to period
#define PERIOD 230

// ---------------------------------------------- variables -----------------------------------------

tmf882xDriver tmf882x;            // instance of tmf882x
extern uint8_t logLevel;          // for i2c logging in shim 
int8_t stateTmf882x;              // current state of the device 
int8_t configNr;                  // this sample application has only a few configurations it will loop through, the variable keeps track of that
int8_t clkCorrectionOn;           // if non-zero clock correction is on
int8_t dumpHistogramOn;           // if non-zero, dump all histograms
// ---------------------------------------------- function declaration ------------------------------

// Switch I2C address.
void changeI2CAddress( );

// Select one of the available configurations and configure the device accordingly.
void configure( );

// Function to print the content of these registers, len should be a multiple of 8
void printRegisters( uint8_t regAddr, uint16_t len, char seperator );

// Print the current state (stateTmf882x) in a readable format
void printState( );

// Function checks the UART for received characters and interprets them
void serialInput( );

// Arduino setup function is only called once at startup. Do all the HW initialisation stuff here.
void setup( );

// Arduino main loop function, is executed cyclic
void loop( );


// -------------------------------------------------------------------------------------------------------------

// Arduino setup function is only called once at startup. Do all the HW initialisation stuff here.
void setup ( )
{
  stateTmf882x = TMF882X_STATE_DISABLED;
  configNr = 0;                             // rotate through the given configurations
  clkCorrectionOn = 1;
  dumpHistogramOn = 0;                      // default is off
  tmf882xInitialise( TMF882X_A, ENABLE_PIN, INTERRUPT_PIN );
  logLevel = LOG_LEVEL_INFO;                                             //i2c logging only

  // configure ENABLE pin and interupt pin
  PIN_OUTPUT( ENABLE_PIN );
  PIN_INPUT( INTERRUPT_PIN );

  // start serial
  Serial.end( );                                      // this clears any old pending data 
  Serial.begin( UART_BAUD_RATE );

  // start i2c
  Wire.begin();
  Wire.setClock( I2C_CLK_SPEED );

  //read from register on sensor to see if we need to upload firmware to it
  uint8_t buf[8];
  uint8_t * ptr = buf;    
  i2c_rx(TMF882X_A->i2cSlaveAddress, ENABLE, buf, 8);
  // Serial.print("Register: ");
  // Serial.print( buf[0] );

  if ( buf[0] != 97 ) //97 just determined through running the script and reading the value. TODO understand this / move to constant
  {
    tmf882xDisable( TMF882X_A );                                     // this resets the I2C address in the device
    delay_in_microseconds(CAP_DISCHARGE_TIME_MS * 1000); // wait for a proper discharge of the cap

    //Enable device (same as recieving 'e' on serial input in example script)
    tmf882xEnable( TMF882X_A );
    delay_in_microseconds( ENABLE_TIME_MS * 1000 );
    tmf882xClkCorrection( TMF882X_A, clkCorrectionOn ); 
    tmf882xSetLogLevel( TMF882X_A, MY_LOG_LEVEL );
    tmf882xWakeup( TMF882X_A );
    if ( tmf882xIsCpuReady( TMF882X_A, CPU_READY_TIME_MS ) )
    {
      if ( tmf882xDownloadFirmware( TMF882X_A ) == BL_SUCCESS_OK ) 
      {
        stateTmf882x = TMF882X_STATE_STOPPED;                           // prints on UART usage and waits for user input on serial
      }
      else
      {
        stateTmf882x = TMF882X_STATE_ERROR;
      }
    }
    else
    {
      stateTmf882x = TMF882X_STATE_ERROR;
    }    

    uint8_t buf3[1];
    buf3[0] = 0x6E;
    i2c_tx(TMF882X_A->i2cSlaveAddress, 0x8, buf3, 1); //0x8 is command register. Write command 0x6E to it to switch to short range mode
    delay(4000); //make sure it has time to act

    //Perform factory calibration (might as well do this every time at startup - same as receiving 'f' on serial input in example script)
    tmf882xConfigure( TMF882X_A, 1, 4000, SPAD_MAP, 0 );    // no histogram dumping in factory calibration allowed, 4M iterations for factory calibration recommended
    tmf882xFactoryCalibration( TMF882X_A );

    //Configure the period, number of iterations (KiloIter), and spad map (SpadId)
    tmf882xConfigure( TMF882X_A, PERIOD, KILO_ITERS, SPAD_MAP, 1 );   //configuration I was originally using
    // tmf882xConfigure( TMF882X_A, 1, 9, TMF882X_COM_SPAD_MAP_ID__spad_map_id__map_no_1, 1 );   //try to go fast - it seems like we're limited by i2c speed, not the sensor speed. This leads to same frame rate

    //Begin measuring (same as recieving an 'm' on serial input in example script)
    tmf882xClrAndEnableInterrupts( TMF882X_A, TMF882X_APP_I2C_RESULT_IRQ_MASK | TMF882X_APP_I2C_RAW_HISTOGRAM_IRQ_MASK );
    tmf882xStartMeasurement( TMF882X_A );
    stateTmf882x = TMF882X_STATE_MEASURE;

  } else
  {
    // stop reading measurements (taken from 's' in example script)
    tmf882xStopMeasurement( TMF882X_A );
    tmf882xDisableInterrupts( TMF882X_A, 0xFF );               // just disable all
    stateTmf882x = TMF882X_STATE_STOPPED;

    //Begin measuring (same as recieving an 'm' on serial input in example script)
    tmf882xClrAndEnableInterrupts( TMF882X_A, TMF882X_APP_I2C_RESULT_IRQ_MASK | TMF882X_APP_I2C_RAW_HISTOGRAM_IRQ_MASK );
    tmf882xStartMeasurement( TMF882X_A );
    stateTmf882x = TMF882X_STATE_MEASURE;
  }

  //try to check the value of the BUILD_VERSION register to see if the firmware supports short range measurements
  uint8_t buf2[1];
  i2c_rx(TMF882X_A->i2cSlaveAddress, 0x19, buf2, 1); //0x20 is config id, which should not be 0. 0x3 is build version, 0x19 active range
  Serial.print("Active range: ");
  Serial.print(buf2[0], HEX);
  Serial.print("\n");

  uint8_t buf3[1];
  buf3[0] = 0x6E;
  i2c_tx(TMF882X_A->i2cSlaveAddress, 0x8, buf3, 1); //0x8 is command register. Write command 0x6E to it to switch to short range mode
//
//  delay(5000);
//  uint8_t buf5[1];
//  i2c_rx(TMF882X_A->i2cSlaveAddress, 0x8, buf5, 1);
//  Serial.print("Status: ");
//  Serial.print(buf5[0], HEX);
//  Serial.print("\n");
//
//  uint8_t buf4[1];
//  i2c_rx(TMF882X_A->i2cSlaveAddress, 0x19, buf4, 1);
//  Serial.print("Active range after: ");
//  Serial.print(buf4[0], HEX);
//  Serial.print("\n");
  
}

// Arduino main loop function, is executed cyclic
void loop ( )
{

  int8_t res = APP_SUCCESS_OK;
  uint8_t intStatus = 0;

  if ( stateTmf882x == TMF882X_STATE_STOPPED || stateTmf882x == TMF882X_STATE_MEASURE )
  { 
    intStatus = tmf882xGetAndClrInterrupts( TMF882X_A, TMF882X_APP_I2C_RESULT_IRQ_MASK | TMF882X_APP_I2C_ANY_IRQ_MASK | TMF882X_APP_I2C_RAW_HISTOGRAM_IRQ_MASK );   // always clear also the ANY interrupt
    if ( intStatus & TMF882X_APP_I2C_RESULT_IRQ_MASK )                      // check if a result is available (ignore here the any interrupt)
    {
      res = tmf882xReadResults( TMF882X_A );
    }
    if ( intStatus & TMF882X_APP_I2C_RAW_HISTOGRAM_IRQ_MASK )
    {
      res = tmf882xReadHistogram( TMF882X_A );                                              // read a (partial) raw histogram
    }
  }

  if ( res != APP_SUCCESS_OK )                         // in case that fails there is some error in programming or on the device, this should not happen
  {
    tmf882xStopMeasurement( TMF882X_A );
    tmf882xDisableInterrupts( TMF882X_A, 0xFF );
    stateTmf882x = TMF882X_STATE_STOPPED;
    PRINT_STR( "#Err" );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "inter" );
    PRINT_CHAR( SEPERATOR );
    PRINT_INT( intStatus );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "but no data" );
    PRINT_LN( );
  }
}
