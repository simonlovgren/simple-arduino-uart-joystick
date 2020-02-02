
/**
 * *********************************************************************************************************************
 * Includes
 * *********************************************************************************************************************
 */
#include <Arduino.h>
#include <SoftwareSerial.h>

extern "C"
{
    #include <crc8.h>
}

/**
 * *********************************************************************************************************************
 * Defines
 * *********************************************************************************************************************
 */

/**
 * @def   COUNT_OF(x)
 * @brief Get number of elements in array.
 */
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

/**
 * @def   UART_BAUD_RATE
 * @brief Baud rate to use for UART communication.
 */
#define UART_BAUD_RATE ( 115200 )


/**
 *  @def   STX_BYTE
 *  @brief Start byte of command/response
 */
#define STX_BYTE ( 0x53 )

/**
 *  @def   ETX_BYTE
 *  @brief End byte of command/response
 */
#define ETX_BYTE ( 0x4C )

/**
 *  @def   COMMAND_TIMEOUT_MS
 *  @brief Timeout of command (command must be fully received within this timeframe).
 */
#define COMMAND_TIMEOUT_MS ( 300 )

/**
 * *********************************************************************************************************************
 * Types
 * *********************************************************************************************************************
 */


/**
 * *********************************************************************************************************************
 * Commands
 * *********************************************************************************************************************
 */

/**
 *  Commands and responses follow this simple format:
 *
 *  Start byte:        0x53
 *  Version:           0x01    (1 byte)
 *  Payload Length:    0xNNNN  (2 bytes)
 *  Command id:        0xNN    (1 bytes)
 *  CRC8 of header:    0xNN    (1 byte)
 *  Payload:           Variable length, see length field.
 *  CRC8 of whole msg: 0xNN (1 byte)
 *  Stop byte:         0x4C
 *
 *
 *  Requests are on even numbers, and responses are on odd numbers directly following request id.
 *  Ex. request 0x02 will get response 0x03.
 *  0x00 counts as "EVEN" with response 0x01
 */

typedef enum command_id_enum_s
{
    REQ_GET_INPUT = 0, /**< Get input state request */
    RSP_GET_INPUT = 1,  /**< Get input state response */

    STATUS_ERROR  = 0xEE
} command_id_enum_t;

typedef uint8_t command_id_t;

#pragma pack(1)

typedef struct command_header_s
{
    uint8_t   version;
    uint16_t  length;
    command_id_t command;
    uint8_t   crc8;
} command_header_t;

/**
 *  RESPONSES
 */
typedef struct rsp_get_input_s
{
    uint8_t input_state; /**< States of inputs as bit-vector */
} rsp_get_input_t;

/**
 *  STATUSES
 */
typedef struct status_error_s
{
    uint8_t error;
    uint8_t parserState;
} status_error_t;

#pragma pack()

/**
 * @brief Pointers to request data
 */
typedef struct command_s
{
    command_header_t *pHeader;
    uint8_t          *pPayload;
} command_t;

/**
 * @brief States of command parser
 */
typedef enum command_state_e
{
    WAIT_FOR_STX         = 0,
    WAIT_FOR_HEADER      = 1,
    CHECK_HEADER_CRC8    = 2,
    WAIT_FOR_PAYLOAD     = 3,
    CHECK_PAYLOAD_CRC8   = 4,
    WAIT_FOR_ETX         = 5,
    COMMAND_RECEIVED     = 6,
    COMMAND_RESET        = 7
} command_state_t;

/**
 *  @brief ERROR enum
 */
typedef enum error_enum_e
{
    ERROR_NO_ERROR = 0,
    ERROR_BUFFER_FULL,
    ERROR_TIMEOUT,
    ERROR_HEADER_CRC,
    ERROR_MESSAGE_CRC,
} error_enum_t;

/**
 * *********************************************************************************************************************
 * Prototypes
 * *********************************************************************************************************************
 */

/**
 *  @brief      Naive implementation of timestamp check, without wraparound protection.
 *              Checks if 'check' is after 'reference'.
 *  @param[in]  check
 *              Timestamp to check.
 *  @param[in]  reference
 *              Timestamp to check against.
 *  @returns    TRUE if 'check' is after 'reference', FALSE otherwise.
 */
bool is_after( uint32_t check, uint32_t reference );

/**
 * @brief       Read- and parse command from UART/SERIAL
 * @param[out]  pCommand
 *              Pointer to where command info is to be stored.
 * @returns     TRUE if command found, FALSE otherwise.
 */
bool read_command( command_t *pCommand );

/**
 * @brief       Reads and returns input states.
 * @returns     TRUE if command found, FALSE otherwise.
 */
uint8_t read_inputs( void );

/**
 * @brief       Creates a message.
 * @param[out]  pBuffer
 *              Pointer to buffer to build message in
 * @param[in]   bufLen
 *              Length of output buffer
 * @param[in]   command_id
 *              Command ID to use
 * @param[in]   pPayload
 *              Pointer to payload data.
 * @param[in]   payload_len
 *              Length of payload data.
 * @param[out]  pMsg_len
 *              Length of created message.
 * @returns     TRUE if all OK, FALSE otherwise.
 */
bool create_message( uint8_t *pBuffer, uint32_t bufLen, command_id_t command_id, uint8_t *pPayload, uint32_t payload_len, uint32_t* pMsg_len );

/**
 * @brief       Sends error status to UART
 * @returns     -
 */
void send_error( uint8_t error );

/**
 * *********************************************************************************************************************
 * Local variables
 * *********************************************************************************************************************
 */

/**
 * @brief Input pins to use for reading Joystick microswitches.
 */
static uint8_t inputs[] = { 4, 5, 6, 7 };

/**
 * @brief Command input buffer
 */
static uint8_t cmd_buffer[ 256 ];

/**
 * @brief Command parser state
 */
static uint8_t  state = WAIT_FOR_STX;

/**
 * *********************************************************************************************************************
 * Public functions
 * *********************************************************************************************************************
 */

/**
 * ---------------------------------------------------------------------------------------------------------------------
 * Function
 * ---------------------------------------------------------------------------------------------------------------------
 */
void setup( void )
{
    // Set up all inputs with internal pull-up
    for ( uint8_t i = 0; i < COUNT_OF( inputs ); ++i )
    {
        pinMode( inputs[ i ], INPUT_PULLUP );
    }

    // Set up serial
    Serial.begin( UART_BAUD_RATE );
}

/**
 * ---------------------------------------------------------------------------------------------------------------------
 * Function
 * ---------------------------------------------------------------------------------------------------------------------
 */
void loop( void )
{
    static uint8_t response[256];
    command_t command;
    if ( read_command( &command ) )
    {
        switch( command.pHeader->command )
        {
            case REQ_GET_INPUT:
            {
                rsp_get_input_t rsp;
                rsp.input_state = read_inputs();
                uint32_t msg_len = 0;
                if ( create_message( (uint8_t *)&response, sizeof( response ), RSP_GET_INPUT, (uint8_t*)&rsp, sizeof(rsp), &msg_len ) )
                {
                    Serial.write( response, msg_len );
                }
            }
            break;

            default:
            {
                // DO NOTHING
            }
            break;
        }
    }
}

/**
 * *********************************************************************************************************************
 * Private functions
 * *********************************************************************************************************************
 */

/**
 * @brief       Sends error status to UART
 * @returns     -
 */
void send_error( uint8_t error )
{
    status_error_t payload;
    payload.error = error;
    payload.parserState = state;
    static uint8_t response[256];
    uint32_t msg_len = 0;
    if ( create_message( (uint8_t *)&response, sizeof( response ), STATUS_ERROR, (uint8_t*)&payload, sizeof(payload), &msg_len ) )
    {
        Serial.write( response, msg_len );
    }
    else
    {
        Serial.println( "UNABLE TO CREATE MESSAGE" );
    }

}

/**
 * ---------------------------------------------------------------------------------------------------------------------
 * Function
 * ---------------------------------------------------------------------------------------------------------------------
 */
bool create_message( uint8_t *pBuffer, uint32_t bufLen, command_id_t command_id, uint8_t *pPayload, uint32_t payload_len, uint32_t* pMsg_len )
{
    if ( pBuffer == NULL || bufLen < ( sizeof( command_header_t ) + payload_len + 2 ) )
    {
        // Buffer not large enough to work with
        return false;
    }

    if ( pPayload == NULL && payload_len > 0 )
    {
        // Contradicting information
        return false;
    }

    uint32_t         buffer_index = 0;
    command_header_t *pHeader = (command_header_t *)&pBuffer[1];

    // STX
    pBuffer[ buffer_index ] = STX_BYTE;
    ++buffer_index;
    // Header
    pHeader->version = 0x01;
    pHeader->length  = payload_len;
    pHeader->command = command_id;
    pHeader->crc8    = crc8( &pBuffer[1], sizeof( command_header_t ) - 1 ); // CRC8 for all but CRC8 field.
    buffer_index += sizeof( command_header_t );
    // Payload
    if ( pPayload != NULL )
    {
        memcpy( &pBuffer[ buffer_index ], pPayload, payload_len );
        buffer_index += payload_len;
    }
    // Total CRC8
    pBuffer[ buffer_index ] = crc8( &pBuffer[1], buffer_index - 1 );
    ++buffer_index;
    // ETX
    pBuffer[ buffer_index ] = ETX_BYTE;
    ++buffer_index;

    // Return values
    *pMsg_len = buffer_index;
    return true;
}

/**
 * ---------------------------------------------------------------------------------------------------------------------
 * Function
 * ---------------------------------------------------------------------------------------------------------------------
 */
bool is_after( uint32_t check, uint32_t reference )
{
    return ( reference < check );
}

/**
 * ---------------------------------------------------------------------------------------------------------------------
 * Function
 * ---------------------------------------------------------------------------------------------------------------------
 */
bool read_command( command_t *pCommand )
{
    static uint32_t buffer_index  = 0;
    static uint32_t stx_timestamp = 0;
    command_header_t *pHeader     = (command_header_t *)&cmd_buffer[ 1 ];

    // Stateless happenings
    if ( buffer_index + 1 >= sizeof( cmd_buffer ) )
    {
        send_error( ERROR_BUFFER_FULL );
        // Message too big, buffer full, reset
        state        = WAIT_FOR_STX;
        buffer_index = 0;
    }

    // If we time out reading message
    if ( ( buffer_index > 0 ) && ( is_after( millis(), stx_timestamp + COMMAND_TIMEOUT_MS ) ) ) {
        // Command timed out
        send_error( ERROR_TIMEOUT );
        Serial.write( buffer_index );
        state        = WAIT_FOR_STX;
        buffer_index = 0;
    }

    switch( state )
    {
        case WAIT_FOR_STX:
        {
            if ( Serial.available() > 0 )
            {
                uint8_t data = Serial.read();
                if ( data == STX_BYTE )
                {
                    stx_timestamp              = millis();
                    cmd_buffer[ buffer_index ] = data;
                    ++buffer_index;
                    state                      = WAIT_FOR_HEADER;
                }
            }
        }
        break;

        case WAIT_FOR_HEADER:
        {
            if ( Serial.available() > 0 )
            {
                uint8_t data = Serial.read();
                cmd_buffer[ buffer_index ] = data;
                ++buffer_index;
                if ( buffer_index == sizeof( command_header_t ) + 1 )
                {
                    state = CHECK_HEADER_CRC8;
                }
            }
        }
        break;

        case CHECK_HEADER_CRC8:
        {
            uint8_t crc8_calc = crc8( &cmd_buffer[ 1 ], sizeof( command_header_t ) -1 ); // Do not count header CRC
            if ( crc8_calc == pHeader->crc8 )
            {
                // CRC8 OK
                if ( pHeader->length > 0 )
                {
                    // We have a payload
                    state = WAIT_FOR_PAYLOAD;
                }
                else
                {
                    // No paylaod
                    state = CHECK_PAYLOAD_CRC8;
                }
            }
            else
            {
                // CRC8 failed
                state        = WAIT_FOR_STX;
                buffer_index = 0;
                send_error( ERROR_HEADER_CRC );
            }
        }
        break;

        case WAIT_FOR_PAYLOAD:
        {
            if ( Serial.available() > 0 )
            {
                uint8_t data = Serial.read();
                cmd_buffer[ buffer_index ] = data;
                ++buffer_index;
                if ( buffer_index == ( sizeof( command_header_t ) + pHeader->length + 1 ) )
                {
                    state = CHECK_PAYLOAD_CRC8;
                }
                Serial.write( buffer_index );
            }
        }
        break;

        case CHECK_PAYLOAD_CRC8:
        {
            if ( Serial.available() > 0 )
            {
                uint8_t data = Serial.read();
                cmd_buffer[ buffer_index ] = data;
                ++buffer_index;

                uint8_t crc8_calc = crc8( &cmd_buffer[ 1 ], sizeof( command_header_t ) + pHeader->length );
                if ( crc8_calc == data )
                {
                    // CRC8 OK
                    state = WAIT_FOR_ETX;
                }
                else
                {
                    // CRC8 failed
                    state        = WAIT_FOR_STX;
                    buffer_index = 0;
                    send_error( ERROR_MESSAGE_CRC );
                }

            }
        }
        break;

        case WAIT_FOR_ETX:
        {
            if ( Serial.available() > 0 )
            {
                uint8_t data = Serial.read();
                if ( data == ETX_BYTE )
                {
                    cmd_buffer[ buffer_index ] = data;
                    ++buffer_index;
                    state                      = COMMAND_RECEIVED;
                }
            }
        }
        break;

        case COMMAND_RECEIVED:
        {
            // Send out command
            pCommand->pHeader  = (command_header_t *)&cmd_buffer[ 1 ];
            if ( pCommand->pHeader->length > 0 )
            {
                pCommand->pPayload = &cmd_buffer[ sizeof( command_header_t ) + 1 ];
            }
            else
            {
                pCommand->pPayload = NULL;
            }
            // Set state COMMAND_RESET to reset parser on next loop
            state = COMMAND_RESET;
            return true;
        }
        break;

        case COMMAND_RESET:
        {
            // Reset state machine
            state        = WAIT_FOR_STX;
            buffer_index = 0;
        }
        break;
    }

    return false;
}

/**
 * ---------------------------------------------------------------------------------------------------------------------
 * Function
 * ---------------------------------------------------------------------------------------------------------------------
 */
uint8_t read_inputs( void )
{
    uint8_t states = 0;
    for ( uint8_t i = 0; i < COUNT_OF( inputs ); ++i )
    {
        states |= digitalRead( inputs[ i ] ) << i;
    }
    return states;
}