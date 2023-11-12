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

#include "tmf882x_shim.h"
#include "tmf882x.h"

uint8_t logLevel = 0;

void delay_in_microseconds ( uint32_t wait )
{
  delayMicroseconds( wait );
}

uint32_t get_sys_tick ( )
{
  return micros( );
}

void i2c_tx ( uint8_t slave_addr, uint8_t reg, const uint8_t *buf, uint8_t len ) 
{  // split long transfers into max of 32-bytes
  do 
  {
    uint8_t tx;
    if ( len > ARDUINO_MAX_I2C_TRANSFER - 1) 
    {
      tx = ARDUINO_MAX_I2C_TRANSFER - 1;
    }
    else 
    {
      tx = len; // less than 31 bytes 
    }
    if ( logLevel & LOG_LEVEL_I2C ) 
    {
      PRINT_STR( "I2C-TX (0x" );
      PRINT_INT_HEX( slave_addr );
      PRINT_STR( ") Reg=0x" );
      PRINT_INT_HEX( reg );
      uint8_t dump_len = tx;
      if ( dump_len ) 
      {
        PRINT_STR( " len=" );
        PRINT_INT( dump_len );
        if ( logLevel >= LOG_LEVEL_DEBUG ) 
        {
          const uint8_t * dump = buf;
          while ( dump_len-- )
          {
            PRINT_STR( " 0x" );
            PRINT_INT_HEX( *dump );
            dump++;
          }
        }
      }
      PRINT_LN( );
    }

    Wire.beginTransmission( slave_addr );
    Wire.write( reg );
    Wire.write( buf, tx );
    len -= tx;
    buf += tx;
    reg += tx;
    Wire.endTransmission( );
  } while ( len );
}

void i2c_rx ( uint8_t slave_addr, uint8_t reg, uint8_t *buf, uint8_t len ) 
{   // split long transfers into max of 32-bytes
  do 
  {
    if ( logLevel & LOG_LEVEL_I2C ) 
    {
      PRINT_STR( "I2C-RX (0x" );
      PRINT_INT_HEX( slave_addr );
      PRINT_STR( ") Reg=0x" );
      PRINT_INT_HEX( reg );
    }
    Wire.beginTransmission( slave_addr );
    Wire.write( reg );
    Wire.endTransmission( );
    uint8_t rx;
    uint8_t * dump = buf; // in case we dump on uart, we need the pointer
    if ( len > ARDUINO_MAX_I2C_TRANSFER ) 
    {
      rx = ARDUINO_MAX_I2C_TRANSFER;
    }
    else 
    {
      rx = len; // less than 32 bytes 
    }
    Wire.requestFrom( slave_addr, rx );
    rx = 0;
    while ( Wire.available() ) 
    {  // read in all available bytes
      *buf = Wire.read();
      buf++;
      len--;
      reg++;
      rx++;
    }
    if ( logLevel & LOG_LEVEL_I2C ) 
    {
      if ( rx ) 
      {
        PRINT_STR( " len=" );
        PRINT_INT( rx );
        if ( logLevel >= LOG_LEVEL_DEBUG ) 
        {
          while ( rx-- )
          {
            PRINT_STR( " 0x" );
            PRINT_INT_HEX( *dump );
            dump++;
          }
        }
      }
      PRINT_LN( );
    }
  } while ( len );
}


// function prints a single result, and returns incremented pointer
static uint8_t * print_result ( tmf882xDriver * driver, uint8_t * data )
{
  uint8_t confidence = data[0];               // 1st byte is confidence
  uint16_t distance = data[2];                // 3rd byte is MSB distance
  distance = (distance << 8) + data[1];       // 2nd byte is LSB distnace
  distance = tmf882xCorrectDistance( driver, distance );
  PRINT_CHAR( SEPERATOR );
  PRINT_INT( distance );
  PRINT_CHAR( SEPERATOR );
  PRINT_INT( confidence );
  return data+3;                              // for convenience only, return the new pointer
}

// Results printing:
// #Obj,<result_number>,<temperature>,<number_valid_results>,<systick>,<distance_0_mm>,<confidence_0>,<distance_1_mm>,<distance_1>, ...
void print_results ( tmf882xDriver * driver, uint8_t * data, uint8_t len )
{
  if ( len >= TMF882X_COM_CONFIG_RESULT__measurement_result_size )
  {
    int8_t i;
    uint32_t sysTick = tmf882xGetUint32( data + RESULT_REG( SYS_TICK_0 ) );
    PRINT_STR( "#Obj" );
    PRINT_CHAR( SEPERATOR );
    PRINT_INT( driver->i2cSlaveAddress );
    PRINT_CHAR( SEPERATOR );
    PRINT_INT( data[ RESULT_REG( RESULT_NUMBER) ] );
    PRINT_CHAR( SEPERATOR );
    PRINT_INT( data[ RESULT_REG( TEMPERATURE )] );
    PRINT_CHAR( SEPERATOR );
    PRINT_INT( data[ RESULT_REG( NUMBER_VALID_RESULTS )] );
    PRINT_CHAR( SEPERATOR );
    PRINT_INT( sysTick );
    data = data + RESULT_REG( RES_CONFIDENCE_0 );
    for ( i = 0; i < PRINT_NUMBER_RESULTS ; i++ )
    {
      data = print_result( driver, data );
    }
    PRINT_LN( );
  }
  else // result structure too short
  {
    PRINT_STR( "#Err" );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "result too short" );
    PRINT_CHAR( SEPERATOR );
    PRINT_INT( len );
    PRINT_LN( );
  }
}

// Print histograms:
// #Raw,<i2c_slave_address>,<sub_packet_number>,<data_0>,<data_1>,..,,<data_127>
// #Cal,<i2c_slave_address>,<sub_packet_number>,<data_0>,<data_1>,..,,<data_127>
void print_histogram ( tmf882xDriver * driver, uint8_t * data, uint8_t len )
{
  if ( len >= TMF882X_COM_HISTOGRAM_PACKET_SIZE )
  {
    uint8_t i;
    uint8_t * ptr = &( data[ RESULT_REG( SUBPACKET_PAYLOAD_0 ) ] );
    if ( data[0] & TMF882X_COM_HIST_DUMP__histogram__raw_24_bit_histogram )
    { 
      PRINT_STR( "#Raw" );
    }
    else if ( data[0] & TMF882X_COM_HIST_DUMP__histogram__electrical_calibration_24_bit_histogram )
    {
      PRINT_STR( "#Cal" );
    }
    else 
    {
      PRINT_STR( "#???" );
    }
    PRINT_CHAR( SEPERATOR );
    PRINT_INT( driver->i2cSlaveAddress );
    PRINT_CHAR( SEPERATOR );
    PRINT_INT( data[ RESULT_REG( SUBPACKET_NUMBER ) ] );          // print the sub-packet number indicating the third-of-a-channel/tdc the histogram belongs to  
    
    for ( i = 0; i < TMF882X_NUMBER_OF_BINS_PER_CHANNEL ; i++, ptr++ )
    {
      PRINT_CHAR( SEPERATOR );
      PRINT_INT( *ptr );
    }
    PRINT_LN( );
  }
  // else structure too short
}
