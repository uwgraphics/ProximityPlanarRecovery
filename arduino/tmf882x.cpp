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

//  simple tmf882x driver

// --------------------------------------------------- includes --------------------------------

#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "tmf882x.h"
#include "tmf882x_image.h"
#include "registers_i2c.h"
#include "tmf882x_shim.h"


// --------------------------------------------------- defines --------------------------------

#define TMF882X_COM_APP_ID                                  0x0   // register address
#define TMF882X_COM_APP_ID__application                     0x3   // measurement application id
#define TMF882X_COM_APP_ID__bootloader                      0x80  // bootloader application id


// --------------------------------------------------- bootloader -----------------------------

#define TMF8X2X_BL_MAX_DATA_SIZE                  0x80  // Number of bytes that can be written or read with one BL command
#define TMF882X_COM_CMD_STAT                      0x08

#define TMF882X_COM_CMD_STAT__bl_cmd_ok           0x00  
#define TMF882X_COM_CMD_STAT__bl_cmd_errors       0x0F  // all success/error are below or equal to this number
#define TMF882X_COM_CMD_STAT__bl_cmd_ramremap     0x11  // Bootloader command to remap the vector table into RAM (Start RAM application).
#define TMF882X_COM_CMD_STAT__bl_cmd_r_ram        0x40  // Read from BL RAM.
#define TMF882X_COM_CMD_STAT__bl_cmd_w_ram        0x41  // Write to BL RAM.
#define TMF882X_COM_CMD_STAT__bl_cmd_addr_ram     0x43  // Set address pointer in RAM for Read/Write to BL RAM.

#define BL_HEADER           2     // bootloader header is 2 bytes
#define BL_MAX_DATA_PAYLOAD 128   // bootloader data payload can be up to 128
#define BL_FOOTER           1     // bootloader footer is 1 byte

// Bootloader maximum wait sequences
#define BL_CMD_SET_ADDR_TIMEOUT_MS    1
#define BL_CMD_W_RAM_TIMEOUT_MS       1
#define BL_CMD_RAM_REMAP_TIMEOUT_MS   1

// wait for version readout, to switch from ROM to RAM (and have the version published on I2C)
#define APP_PUBLISH_VERSION_WAIT_TIME_MS 10

// --------------------------------------------------- application ----------------------------

// application status, we check only for ok or accepted, everything between 2 and 15 (inclusive) 
// is an error
#define TMF882X_COM_CMD_STAT__stat_ok                       0x0  // Everything is okay
#define TMF882X_COM_CMD_STAT__stat_accepted                 0x1  // Everything is okay too, send sop to halt ongoing command

// application commands
#define TMF882X_COM_CMD_STAT__cmd_measure                         0x10  // Start a measurement
#define TMF882X_COM_CMD_STAT__cmd_stop                            0xff  // Stop a measurement
#define TMF882X_COM_CMD_STAT__cmd_write_config_page               0x15  // Write the active config page
#define TMF882X_COM_CMD_STAT__cmd_load_config_page_common         0x16  // Load the common config page
#define TMF882X_COM_CMD_STAT__cmd_load_config_page_factory_calib  0x19  // Load the factory calibration config page
#define TMF882X_COM_CMD_STAT__cmd_factory_calibration             0x20  // Perform a factory calibration
#define TMF882X_COM_CMD_STAT__cmd_i2c_slave_address               0x21  // change I2C address


// configuration page addresses and defines
#define TMF882X_COM_PERIOD_MS_LSB                           0x24  // period in milliseconds
#define TMF882X_COM_PERIOD_MS_MSB                           0x25
#define TMF882X_COM_KILO_ITERATIONS_LSB                     0x26  // Kilo (1024) iterations
#define TMF882X_COM_KILO_ITERATIONS_MSB                     0x27
#define TMF882X_COM_SPAD_MAP_ID                             0x34  // configure the SPAD map id, with some example maps
#define TMF882X_COM_SPAD_MAP_ID__map_last                   0x15  // maximum allowed spad map id
#define TMF8X2X_COM_HIST_DUMP                               0x39  // 0 ... all off, 1 ... raw histograms, 2 ... ec histograms
#define TMF8X2X_COM_I2C_SLAVE_ADDRESS                       0x3b  // register that holds the 7-bit shifted slave address

// Application maximum wait sequences
#define APP_CMD_LOAD_CONFIG_TIMEOUT_MS                      3
#define APP_CMD_WRITE_CONFIG_TIMEOUT_MS                     3
#define APP_CMD_MEASURE_TIMEOUT_MS                          5
#define APP_CMD_STOP_TIMEOUT_MS                             25
#define APP_CMD_FACTORY_CALIB_TIMEOUT_MS                    2000
#define APP_CMD_I2C_SLAVE_ADDRESS_TIMEOUT_MS                1

// -------------------------------------------------------- some checks --------------------------------------------

// check that we can read a complete result page also in the dataBuffer
#define DATA_BUFFER_SIZE                  (TMF882X_COM_CONFIG_FACTORY_CALIB__factory_calibration_size)

#if ( ( (BL_HEADER + BL_MAX_DATA_PAYLOAD + BL_FOOTER + 1) > DATA_BUFFER_SIZE ) || ( (TMF882X_COM_CONFIG_RESULT__measurement_result_size) > DATA_BUFFER_SIZE ) )
  #error "Increase data buffer size"
#endif


// clock correction pairs index calculation
#define CLK_CORRECTION_IDX_MODULO( x )    ( (x) & ( (CLK_CORRECTION_PAIRS)-1 ) )

// how accurate the calculation is going to be. The higher the accuracy the less far apart are
// the pairs allowed. An 8 precision means that the factor is 1/256 accurate.
#define CALC_PRECISION                                                  8
// Need this to add to the corrected distance before shifting right
#define HALF_CALC_PRECISION                                             ( 1 << ((CALC_PRECISION) - 1 ) )
#define CALC_DISTANCE_CORR_FACTOR( hostTickDiff, tmf882xTickDiff )      ( ( ( (hostTickDiff) * (TMF882X_TICKS_PER_US) ) << (CALC_PRECISION) ) / ( (tmf882xTickDiff) * (HOST_TICKS_PER_US) ) ) 
// Round before performing the division (right shift), make sure it is a logical shift right and not an arithmetical shift right
#define CALC_DISTANCE( distance, hostTickDiff, tmf882xTickDiff )        ( ( (uint32_t)( (distance) * CALC_DISTANCE_CORR_FACTOR( hostTickDiff, tmf882xTickDiff ) + (HALF_CALC_PRECISION) ) ) >> (CALC_PRECISION) )

// Find the maximum distance values to avoid mathematical errors due to overflow 
#define MAX_HOST_DIFF_VALUE                                             ( ( 0xFFFFFFFFUL / (TMF882X_TICKS_PER_US) ) >> CALC_PRECISION )
#define MAX_TMF882X_DIFF_VALUE                                          ( ( 0xFFFFFFFFUL / (HOST_TICKS_PER_US) )

// Saturation macro for 16-bit
#define SATURATE16( v )                                                 ( (v) > 0xFFFF ? 0xFFFF : (v) )

// For TMF882x sys ticks to be valid the LSB must be set.
#define TMF882X_SYS_TICK_IS_VALID( tick )                               ( (tick) & 1 )

// -------------------------------------------------------- variables ----------------------------------------------

uint8_t dataBuffer[ DATA_BUFFER_SIZE ];           // transfer/receive buffer


// -------------------------------------------------------- functions ----------------------------------------------

static void tmf882xResetClockCorrection( tmf882xDriver * driver );

void tmf882xInitialise ( tmf882xDriver * driver, uint8_t enablePin, uint8_t interruptPin )
{
  tmf882xResetClockCorrection( driver );
  driver->enablePin = enablePin;
  driver->interruptPin = interruptPin;
  driver->i2cSlaveAddress = TMF882X_SLAVE_ADDR;
  driver->clkCorrectionEnable = 1;                  // default is on
  driver->logLevel = LOG_LEVEL_ERROR;
}

// Function to overwrite the default log level
void tmf882xSetLogLevel ( tmf882xDriver * driver, uint8_t level )
{
  driver->logLevel = level;
}

// Function to set clock correction on or off.
// enable ... if <>0 clock correction is enabled (default)
// enable ... if ==0 clock correction is disabled
void tmf882xClkCorrection ( tmf882xDriver * driver, uint8_t enable )
{
  driver->clkCorrectionEnable = !!enable;
}

// Function executes a reset of the device 
void tmf882xReset ( tmf882xDriver * driver ) 
{
  dataBuffer[0] = RESETREASON__soft_reset__MASK;
  i2c_tx( driver->i2cSlaveAddress, RESETREASON, dataBuffer, 1 );
  if ( driver->logLevel >= LOG_LEVEL_VERBOSE ) 
  {
    PRINT_STR( "reset" );
    PRINT_LN( );
  }
  tmf882xResetClockCorrection( driver );
}

// Function sets the enable PIN high
void tmf882xEnable ( tmf882xDriver * driver ) 
{
  PIN_HIGH( driver->enablePin );
  tmf882xInitialise( driver, driver->enablePin, driver->interruptPin );           // when enable gets high, the HW resets to default slave addr
}

// Function clears the enable PIN (=low)
void tmf882xDisable ( tmf882xDriver * driver ) 
{
  PIN_LOW( driver->enablePin );
}


// Function checks if the CPU becomes ready within the given time
int8_t tmf882xIsCpuReady ( tmf882xDriver * driver, uint8_t waitInMs )
{
  i2c_rx( driver->i2cSlaveAddress, ENABLE, dataBuffer, 1 );        // Need to read it twice after a PON=0, so do it 1 additional time 
  do 
  {
    dataBuffer[0] = 0;                                        // clear before reading
    i2c_rx( driver->i2cSlaveAddress, ENABLE, dataBuffer, 1 );      // read the enable register to determine cpu ready
    if ( !!( dataBuffer[0] & ENABLE__cpu_ready__MASK ) )
    {
      if ( driver->logLevel >= LOG_LEVEL_VERBOSE )
      {
        PRINT_STR( "CPU ready" );
        PRINT_LN( );
      }
      return 1;                                               // done                  
    }
    else if ( waitInMs )                                      // only wait until it is the last time we go through the loop, that would be a waste of time to wait again
    {
      delay_in_microseconds( 1000 );
    }
  } while ( waitInMs-- );
  if ( driver->logLevel >= LOG_LEVEL_ERROR )
  {
    PRINT_STR( "#Err" );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "CPU not ready" );
    PRINT_LN( );
  }
  return 0;                                                 // cpu did not get ready
}

// Function attemps a wakeup of the device 
void tmf882xWakeup ( tmf882xDriver * driver ) 
{
  dataBuffer[0] = 0;                                         // clear before reading
  i2c_rx( driver->i2cSlaveAddress, ENABLE, dataBuffer, 1 );      // read the enable register to dermine power state
  if ( ( dataBuffer[0] & ENABLE__cpu_ready__MASK ) == 0 )                  
  {
    dataBuffer[0] = dataBuffer[0] | ENABLE__pon__MASK;      // make sure to keep the remap bits
    i2c_tx( driver->i2cSlaveAddress, ENABLE, dataBuffer, 1 );    // set PON bit in enable register
    if ( driver->logLevel >= LOG_LEVEL_VERBOSE ) 
    {
      PRINT_STR( "PON=1" );
      PRINT_LN( );
    }
  }
  else
  {
    if ( driver->logLevel >= LOG_LEVEL_VERBOSE ) 
    {
      PRINT_STR( "awake ENABLE=0x" );
      PRINT_INT_HEX( dataBuffer[0] );
      PRINT_LN( );
    }  
  }
}

// Function puts the device in standby state
void tmf882xStandby ( tmf882xDriver * driver ) 
{
  dataBuffer[0] = 0;                                                   // clear before reading
  i2c_rx( driver->i2cSlaveAddress, ENABLE, dataBuffer, 1 );      // read the enable register to determine power state
  if ( ( dataBuffer[0] & ENABLE__cpu_ready__MASK ) != 0 )                  
  {
    dataBuffer[0] = dataBuffer[0] & ~ENABLE__pon__MASK;                         // clear only the PON bit
    i2c_tx( driver->i2cSlaveAddress, ENABLE, dataBuffer, 1 );   // clear PON bit in enable register
    if ( driver->logLevel >= LOG_LEVEL_VERBOSE ) 
    {
      PRINT_STR( "PON=0" );
      PRINT_LN( );
    }
  }
  else
  {
    if ( driver->logLevel >= LOG_LEVEL_VERBOSE ) 
    {
      PRINT_STR( "standby ENABLE=0x" );
      PRINT_INT_HEX( dataBuffer[0] );
      PRINT_LN( );
    }  
  }
}

// function to check if a register has a specific value
static int8_t tmf882xCheckRegister ( tmf882xDriver * driver, uint8_t regAddr, uint8_t expected, uint8_t len, uint16_t timeoutInMs )
{
  uint8_t i;
  uint32_t t = get_sys_tick();
  do 
  {
    dataBuffer[0] = ~expected;
    i2c_rx( driver->i2cSlaveAddress, regAddr, dataBuffer, len );
    if ( dataBuffer[0] == expected )
    {
      return APP_SUCCESS_OK; 
    }
    else if ( timeoutInMs )                             // do not wait if timeout is 0
    {
      delay_in_microseconds(1000);  
    }
  } while ( timeoutInMs-- > 0 );
  if ( driver->logLevel >= LOG_LEVEL_ERROR ) 
  {
    t = get_sys_tick() - t;
    PRINT_STR( "#Err" );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "timeout " );
    PRINT_INT( t );
    PRINT_STR( " reg=0x" );
    PRINT_INT_HEX( regAddr );
    for ( i = 0; i < len; i++ )
    {
      PRINT_STR( " 0x" );
      PRINT_INT_HEX( dataBuffer[i] );
    }
    PRINT_LN( );
  }      
  return APP_ERROR_TIMEOUT;        // error timeout
}


// --------------------------------------- bootloader ------------------------------------------

// calculate the checksum according to bootloader spec
static uint8_t tmf882xBootloaderChecksum ( uint8_t * data, uint8_t len )      
{
  uint8_t sum = 0;
  while ( len-- > 0 )
  {
    sum += *data;
    data++;
  }
  sum = sum ^ 0xFF;
  return sum;
}

// execute command to set the RAM address pointer for RAM read/writes
static int8_t tmf882xBootloaderSetRamAddr ( tmf882xDriver * driver, uint16_t addr )
{
  dataBuffer[0] = TMF882X_COM_CMD_STAT__bl_cmd_addr_ram;
  dataBuffer[1] = 2;
  dataBuffer[2] = (uint8_t)addr;        // LSB of addr
  dataBuffer[3] = (uint8_t)(addr>>8);    // MSB of addr
  dataBuffer[4] = tmf882xBootloaderChecksum( dataBuffer, 4 );
  i2c_tx( driver->i2cSlaveAddress, TMF882X_COM_CMD_STAT, dataBuffer, 5 );
  return tmf882xCheckRegister( driver, TMF882X_COM_CMD_STAT, TMF882X_COM_CMD_STAT__bl_cmd_ok, 3, BL_CMD_SET_ADDR_TIMEOUT_MS );      // many BL errors only have 3 bytes 
}

// execute command to write a chunk of data to RAM
static int8_t tmf882xBootloaderWriteRam ( tmf882xDriver * driver, uint8_t len )
{
  dataBuffer[0] = TMF882X_COM_CMD_STAT__bl_cmd_w_ram;
  dataBuffer[1] = len;
  dataBuffer[BL_HEADER+len] = tmf882xBootloaderChecksum( dataBuffer, BL_HEADER+len );
  i2c_tx( driver->i2cSlaveAddress, TMF882X_COM_CMD_STAT, dataBuffer, BL_HEADER+len+BL_FOOTER );
  return tmf882xCheckRegister( driver, TMF882X_COM_CMD_STAT,TMF882X_COM_CMD_STAT__bl_cmd_ok, 3, BL_CMD_W_RAM_TIMEOUT_MS );    // many BL errors only have 3 bytes 
}

// execute command RAM remap to address 0 and continue running from RAM 
static int8_t tmf882xBootloaderRamRemap ( tmf882xDriver * driver, uint8_t appId )
{
  int8_t stat;
  dataBuffer[0] = TMF882X_COM_CMD_STAT__bl_cmd_ramremap;
  dataBuffer[1] = 0;
  dataBuffer[BL_HEADER] = tmf882xBootloaderChecksum( dataBuffer, BL_HEADER );
  i2c_tx( driver->i2cSlaveAddress, TMF882X_COM_CMD_STAT, dataBuffer, BL_HEADER+BL_FOOTER );
  delay_in_microseconds( APP_PUBLISH_VERSION_WAIT_TIME_MS * 1000 );
  // ram remap -> the bootloader will not answer to this command if successfull, so check the application id register instead
  stat = tmf882xCheckRegister( driver, TMF882X_COM_APP_ID, appId, 4, BL_CMD_RAM_REMAP_TIMEOUT_MS );  // tmf882x application has 4 verion bytes
  if ( driver->logLevel >= LOG_LEVEL_INFO )
  {
    PRINT_STR( "#Vers" );
    PRINT_CHAR( SEPERATOR );
    PRINT_INT( dataBuffer[0] );
    PRINT_CHAR(  '.' );
    PRINT_INT( dataBuffer[1] );
    PRINT_CHAR(  '.' );
    PRINT_INT( dataBuffer[2] );
    PRINT_CHAR(  '.' );
    PRINT_INT( dataBuffer[3] );
    PRINT_LN( );
  }
  return stat;
}

// download the image file to RAM
int8_t tmf882xDownloadFirmware ( tmf882xDriver * driver ) 
{
  uint32_t idx = 0;
  int8_t stat = BL_SUCCESS_OK;
  uint8_t chunkLen;
  if ( driver->logLevel >= LOG_LEVEL_VERBOSE ) 
  {
    PRINT_STR( "Image addr=0x" );
    PRINT_INT_HEX( tmf882x_image_start );
    PRINT_STR( " len=" );
    PRINT_INT( tmf882x_image_length );
    for ( idx = 0; idx < 16; idx++ )
    {
      PRINT_STR( " 0x" );
      PRINT_INT_HEX( READ_PROGRAM_MEMORY_BYTE( tmf882x_image + idx ) );      // read from program memory space
    }
    PRINT_LN( );
  }
  stat = tmf882xBootloaderSetRamAddr( driver, tmf882x_image_start );
  idx = 0;  // start again at the image begin
  while ( stat == BL_SUCCESS_OK && idx < tmf882x_image_length )
  {
      if ( driver->logLevel >= LOG_LEVEL_VERBOSE )
      {
        PRINT_STR( "Download addr=0x" );
        PRINT_INT_HEX( idx );
        PRINT_LN( );
      }
      for( chunkLen=0; chunkLen < BL_MAX_DATA_PAYLOAD && idx < tmf882x_image_length; chunkLen++, idx++ )
      {
        dataBuffer[BL_HEADER + chunkLen] = READ_PROGRAM_MEMORY_BYTE( tmf882x_image + idx );              // read from code memory into local ram buffer
      }
      stat = tmf882xBootloaderWriteRam( driver, chunkLen );
  }
  if ( stat == BL_SUCCESS_OK )
  {
    stat = tmf882xBootloaderRamRemap( driver, TMF882X_COM_APP_ID__application );      // if you load a test-application this may have another ID
    if ( stat == BL_SUCCESS_OK )
    {
      if ( driver->logLevel >= LOG_LEVEL_INFO )
      {
        PRINT_STR( "FW downloaded" );
        PRINT_LN( );
      }
      return stat;
    }
  }
  if ( driver->logLevel >= LOG_LEVEL_ERROR ) 
  {
    PRINT_STR( "#Err" );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "FW downl or REMAP" );
    PRINT_LN( );
  }
  return stat;
}

// --------------------------------------- application -----------------------------------------

// Reset clock correction calculation
static void tmf882xResetClockCorrection ( tmf882xDriver * driver )
{
  uint8_t i;
  driver->clkCorrectionIdx = 0;                      // reset clock correction
  for ( i = 0; i < CLK_CORRECTION_PAIRS; i++ )
  {
    driver->hostTicks[ i ] = 0;
    driver->tmf882xTicks[ i ] = 0;                  // initialise the tmf8828Ticks to a value that has the LSB cleared -> can identify that these are no real ticks
  }
  if ( driver->logLevel & LOG_LEVEL_CLK_CORRECTION )
  {
    PRINT_STR( "ClkCorr reset" );
    PRINT_LN( );
  }
}

// Add a host tick and a tmf882x tick to the clock correction list.
static void tmf882xClockCorrectionAddPair ( tmf882xDriver * driver, uint32_t hostTick, uint32_t tmf882xTick )
{
  if ( TMF882X_SYS_TICK_IS_VALID( tmf882xTick ) )                             // only use ticks if tmf882xTick has LSB set
  {
    driver->clkCorrectionIdx = CLK_CORRECTION_IDX_MODULO( driver->clkCorrectionIdx + 1 );     // increment and take care of wrap-over
    driver->hostTicks[ driver->clkCorrectionIdx ] = hostTick;
    driver->tmf882xTicks[ driver->clkCorrectionIdx ] = tmf882xTick;
  }
  else if ( driver->logLevel & LOG_LEVEL_CLK_CORRECTION )
  {
    PRINT_STR( "ClkCorr ticks invalid " );      // this can happen if the host did read out the data very, very fast,
    PRINT_INT( tmf882xTick );                   // and the device was busy handling other higher priority interrupts
    PRINT_LN( );                                // The device does always set the LSB of the sys-tick to indicate that
  }                                             // the device did set the sys-tick.      
}    

// execute command to load a given config page
static int8_t tmf882xLoadConfigPage ( tmf882xDriver * driver, uint8_t pageCmd )
{
  int8_t stat;
  dataBuffer[0] = pageCmd;
  i2c_tx( driver->i2cSlaveAddress, TMF882X_COM_CMD_STAT, dataBuffer, 1 );                                                      // instruct device to load page
  stat = tmf882xCheckRegister( driver, TMF882X_COM_CMD_STAT, TMF882X_COM_CMD_STAT__stat_ok, 1, APP_CMD_LOAD_CONFIG_TIMEOUT_MS );  // check that load command is completed
  if ( stat == APP_SUCCESS_OK )
  {
    stat = tmf882xCheckRegister( driver, TMF882X_COM_CONFIG_RESULT, pageCmd, 1, 0 );                                              // check that correct config page is loaded, no more waiting
  }
  return stat;
}

// Convert 4 bytes in little endian format into an uint32_t  
uint32_t tmf882xGetUint32 ( uint8_t * data ) 
{
  uint32_t t =    data[ 3 ];
  t = (t << 8 ) + data[ 2 ];
  t = (t << 8 ) + data[ 1 ];
  t = (t << 8 ) + data[ 0 ];
  return t;
}

// Function that executes command to write any configuration page
int8_t tmf882xWriteConfigPage ( tmf882xDriver * driver )
{
  int8_t stat;
  dataBuffer[0] = TMF882X_COM_CMD_STAT__cmd_write_config_page;
  i2c_tx( driver->i2cSlaveAddress, TMF882X_COM_CMD_STAT, dataBuffer, 1 );                                                      // instruct device to load page
  stat = tmf882xCheckRegister( driver, TMF882X_COM_CMD_STAT, TMF882X_COM_CMD_STAT__stat_ok, 1, APP_CMD_WRITE_CONFIG_TIMEOUT_MS ); // check that write command is completed
  return stat;
}

// Function to load the common config page into I2C ram that the host can read/write it via I2C
int8_t tmf882xLoadConfigPageCommon ( tmf882xDriver * driver )
{
  return tmf882xLoadConfigPage( driver, TMF882X_COM_CMD_STAT__cmd_load_config_page_common );
}

// Function to load the factory calibration config page into I2C ram that the host can read/write it via I2C
int8_t tmf882xLoadConfigPageFactoryCalib ( tmf882xDriver * driver )
{
  return tmf882xLoadConfigPage( driver, TMF882X_COM_CMD_STAT__cmd_load_config_page_factory_calib );
}

// function to change the I2C address of the device
int8_t tmf882xChangeI2CAddress ( tmf882xDriver * driver, uint8_t newI2cSlaveAddress )
{
  uint8_t oldAddr = driver->i2cSlaveAddress;
  int8_t stat = tmf882xLoadConfigPageCommon( driver );          // first load the page, then only overwrite the registers you want to change 
  if ( stat == APP_SUCCESS_OK )
  {
    dataBuffer[0] = newI2cSlaveAddress << 1;          // i2c slave address is shifted into the upper 7 bits of the 8-bit register
    i2c_tx( driver->i2cSlaveAddress, TMF8X2X_COM_I2C_SLAVE_ADDRESS, dataBuffer, 1 );
    stat = tmf882xWriteConfigPage( driver );                 //  write the config page back
    if ( stat == APP_SUCCESS_OK )
    {
      dataBuffer[0] = TMF882X_COM_CMD_STAT__cmd_i2c_slave_address;
      i2c_tx( driver->i2cSlaveAddress, TMF882X_COM_CMD_STAT, dataBuffer, 1 );      // instruct device to change i2c address
      driver->i2cSlaveAddress = newI2cSlaveAddress;                                 // from now on try to read from new address
      stat = tmf882xCheckRegister( driver, TMF882X_COM_CMD_STAT, TMF882X_COM_CMD_STAT__stat_ok, 1, APP_CMD_I2C_SLAVE_ADDRESS_TIMEOUT_MS ); // check that command ok
     }
  }
  if ( stat != APP_SUCCESS_OK )
  {
    driver->i2cSlaveAddress = oldAddr;                                            // switch failed, use old address again
    if ( driver->logLevel >= LOG_LEVEL_ERROR ) 
    {
      PRINT_STR( "#Err" );
      PRINT_CHAR( SEPERATOR );
      PRINT_STR( "I2C-Addr " );
      PRINT_INT( stat );
      PRINT_LN( );
    }
  }
  return stat; 
}

// configure device according to given parameters
int8_t tmf882xConfigure ( tmf882xDriver * driver, uint16_t periodInMs, uint16_t kiloIterations, uint8_t spadMapId, uint8_t dumpHistogram )
{
  int8_t stat = APP_ERROR_PARAM;
  if ( spadMapId <= TMF882X_COM_SPAD_MAP_ID__map_last )
  {
    stat = tmf882xLoadConfigPageCommon( driver );          // first load the page, then only overwrite the registers you want to change 
    if ( stat == APP_SUCCESS_OK )
    {
      dataBuffer[0] = (uint8_t)periodInMs;            // lsb
      dataBuffer[1] = (uint8_t)(periodInMs>>8);       // msb
      dataBuffer[2] = (uint8_t)kiloIterations;        // lsb  - kilo iterations are right behind the period so we can write with one i2c tx
      dataBuffer[3] = (uint8_t)(kiloIterations>>8);   // msb
      i2c_tx( driver->i2cSlaveAddress, TMF882X_COM_PERIOD_MS_LSB, dataBuffer, 4 );
      dataBuffer[0] = spadMapId;                      // spad map ID is a different reg, so use a seperate i2c tx
      i2c_tx( driver->i2cSlaveAddress, TMF882X_COM_SPAD_MAP_ID, dataBuffer, 1 );
      dataBuffer[0] = dumpHistogram & 0x3;            // only raw histograms and/or EC histograms
      i2c_tx( driver->i2cSlaveAddress, TMF8X2X_COM_HIST_DUMP, dataBuffer, 1 );
      stat = tmf882xWriteConfigPage( driver );               // as a last step write the config page back
    }
  }
  if ( stat != APP_SUCCESS_OK )
  {
    if ( driver->logLevel >= LOG_LEVEL_ERROR ) 
    {
      PRINT_STR( "#Err" );
      PRINT_CHAR( SEPERATOR );
      PRINT_STR( "Config " );
      PRINT_INT( stat );
      PRINT_LN( );
    }
  }
  return stat;
}

// Function to execute a factory calibration
int8_t tmf882xFactoryCalibration ( tmf882xDriver * driver )
{
  dataBuffer[0] = TMF882X_COM_CMD_STAT__cmd_factory_calibration;
  i2c_tx( driver->i2cSlaveAddress, TMF882X_COM_CMD_STAT, dataBuffer, 1 );                                                      // instruct device to load page
  return tmf882xCheckRegister( driver, TMF882X_COM_CMD_STAT, TMF882X_COM_CMD_STAT__stat_ok, 1, APP_CMD_FACTORY_CALIB_TIMEOUT_MS ); // check that factory calib command is done
}

// Function to write a pre-stored calibration page
int8_t tmf882xSetStoredFactoryCalibration ( tmf882xDriver * driver, const uint8_t * calibPage )
{
  dataBuffer[0] = READ_PROGRAM_MEMORY_BYTE( calibPage );
  if ( dataBuffer[0] == TMF882X_COM_CMD_STAT__cmd_load_config_page_factory_calib )
  {
    for ( uint8_t i = 1; i < TMF882X_COM_CONFIG_FACTORY_CALIB__factory_calibration_size; i++ )
    {
      dataBuffer[i] = READ_PROGRAM_MEMORY_BYTE( calibPage + i );
    }
    // actually write the calibration data
    i2c_tx( driver->i2cSlaveAddress, TMF882X_COM_CONFIG_RESULT, dataBuffer, TMF882X_COM_CONFIG_FACTORY_CALIB__factory_calibration_size );
    // issue the write page command
    return tmf882xWriteConfigPage( driver );
  }
  return APP_ERROR_NO_CALIB_PAGE;
}

// function starts a measurement
int8_t tmf882xStartMeasurement ( tmf882xDriver * driver ) 
{
  tmf882xResetClockCorrection( driver );                                                                                         // clock correction only works during measurements
  dataBuffer[0] = TMF882X_COM_CMD_STAT__cmd_measure;
  i2c_tx( driver->i2cSlaveAddress, TMF882X_COM_CMD_STAT, dataBuffer, 1 );                                                      // instruct device to load page
  return tmf882xCheckRegister( driver, TMF882X_COM_CMD_STAT, TMF882X_COM_CMD_STAT__stat_accepted, 1, APP_CMD_MEASURE_TIMEOUT_MS ); // check that measure command is accepted
}

// function stops a measurement
int8_t tmf882xStopMeasurement ( tmf882xDriver * driver ) 
{
  dataBuffer[0] = TMF882X_COM_CMD_STAT__cmd_stop;
  i2c_tx( driver->i2cSlaveAddress, TMF882X_COM_CMD_STAT, dataBuffer, 1 );                                                      // instruct device to load page
  return tmf882xCheckRegister( driver, TMF882X_COM_CMD_STAT, TMF882X_COM_CMD_STAT__stat_ok, 1, APP_CMD_STOP_TIMEOUT_MS );         // check that stop command is accepted
}

// function reads and clears the specified interrupts
uint8_t tmf882xGetAndClrInterrupts ( tmf882xDriver * driver, uint8_t mask )
{
  uint8_t setInterrupts;
  dataBuffer[0] = 0;
  i2c_rx( driver->i2cSlaveAddress, INT_STATUS, dataBuffer, 1 );            // read interrupt status register
  setInterrupts = dataBuffer[0] & mask;
  if ( setInterrupts )
  {
    dataBuffer[0] = dataBuffer[0] & mask;                             // clear only those that were set when we read the register, and only those we want to know
    i2c_tx( driver->i2cSlaveAddress, INT_STATUS, dataBuffer, 1 );       // clear interrupts by pushing a 1 to status register
  }
  return setInterrupts;
}

// function clears and enables the specified interrupts
void tmf882xClrAndEnableInterrupts ( tmf882xDriver * driver, uint8_t mask )
{
  dataBuffer[0] = 0xFF;                                               // clear all interrupts  
  i2c_tx( driver->i2cSlaveAddress, INT_STATUS, dataBuffer, 1 );         // clear interrupts by pushing a 1 to status register
  dataBuffer[0] = 0;
  i2c_rx( driver->i2cSlaveAddress, INT_ENAB, dataBuffer, 1 );            // read current enabled interrupts 
  dataBuffer[0] = dataBuffer[0] | mask;                             // enable those in the mask, keep the others if they were enabled
  i2c_tx( driver->i2cSlaveAddress, INT_ENAB, dataBuffer, 1 );
}

// function disables the specified interrupts
void tmf882xDisableInterrupts ( tmf882xDriver * driver, uint8_t mask )
{
  dataBuffer[0] = 0;
  i2c_rx( driver->i2cSlaveAddress, INT_ENAB, dataBuffer, 1 );            // read current enabled interrupts 
  dataBuffer[0] = dataBuffer[0] & ~mask;                            // clear only those in the mask, keep the others if they were enabled
  i2c_tx( driver->i2cSlaveAddress, INT_ENAB, dataBuffer, 1 );
  dataBuffer[0] = mask; 
  i2c_tx( driver->i2cSlaveAddress, INT_STATUS, dataBuffer, 1 );         // clear interrupts by pushing a 1 to status register
}

// function reads the result page (if there is none the function returns an error, else success)
int8_t tmf882xReadResults ( tmf882xDriver * driver ) 
{
  uint32_t hTick;            // get the sys-tick just before the I2C rx
  dataBuffer[0] = 0;
  hTick = get_sys_tick( );            // get the sys-tick just before the I2C rx
  i2c_rx( driver->i2cSlaveAddress, TMF882X_COM_CONFIG_RESULT, dataBuffer, TMF882X_COM_CONFIG_RESULT__measurement_result_size );
  if ( dataBuffer[0] == TMF882X_COM_CONFIG_RESULT__measurement_result )
  {
    uint32_t tTick = tmf882xGetUint32( dataBuffer + RESULT_REG( SYS_TICK_0 ) );
    tmf882xClockCorrectionAddPair( driver, hTick, tTick );
    print_results( driver, dataBuffer, TMF882X_COM_CONFIG_RESULT__measurement_result_size );
    return APP_SUCCESS_OK;
  }
  return APP_ERROR_NO_RESULT_PAGE;
}

// Correct the distance based on the clock correction pairs 
uint16_t tmf882xCorrectDistance ( tmf882xDriver * driver, uint16_t distance )
{
  if ( driver->clkCorrectionEnable )
  {
    uint8_t idx = driver->clkCorrectionIdx;                                         // last inserted
    uint8_t idx2 = CLK_CORRECTION_IDX_MODULO( idx + CLK_CORRECTION_PAIRS - 1 );     // oldest available
    if ( TMF882X_SYS_TICK_IS_VALID( driver->tmf882xTicks[ idx ] ) && TMF882X_SYS_TICK_IS_VALID( driver->tmf882xTicks[ idx2 ] ) )    // only do a correction if both tmf882x ticks are valid 
    { 
      uint32_t hDiff = driver->hostTicks[ idx ] - driver->hostTicks[ idx2 ];
      uint32_t tDiff = driver->tmf882xTicks[ idx ] - driver->tmf882xTicks[ idx2 ];
      if ( tDiff > 0 && hDiff > 0 )                                                 // do not multiply or divide by 0
      {
        uint32_t d = distance;
        d = CALC_DISTANCE( d, hDiff, tDiff );
        distance = SATURATE16( d );
      }
    }
  }
  return distance;
}

// Function to read histograms and print them on UART. 
int8_t tmf882xReadHistogram ( tmf882xDriver * driver  )
{
  dataBuffer[0] = 0;
  i2c_rx( driver->i2cSlaveAddress, TMF882X_COM_CONFIG_RESULT, dataBuffer, TMF882X_COM_HISTOGRAM_PACKET_SIZE );
  if ( ( dataBuffer[0] & TMF882X_COM_OPTIONAL_SUBPACKET_HEADER_MASK ) == TMF882X_COM_OPTIONAL_SUBPACKET_HEADER_MASK ) // histograms must have MSB set
  {
    print_histogram( driver, dataBuffer, TMF882X_COM_HISTOGRAM_PACKET_SIZE );
    return APP_SUCCESS_OK;
  }
  return APP_ERROR_NO_RESULT_PAGE;
}
