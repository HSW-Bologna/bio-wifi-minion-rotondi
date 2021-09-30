#ifndef SERIAL_H_INCLUDED
#define SERIAL_H_INCLUDED

#include <stdint.h>
#include <stdlib.h>

#define SERIAL_PACKET_MAX_LENGTH 256

#define COMMAND_READ_INPUT              0x0101
#define COMMAND_READ_OUTPUT             0x0102
#define COMMAND_SET_OUTPUT_SINGLE_PULSE 0x0202
#define COMMAND_SET_OUTPUT_MULTI_PULSE  0x0203
#define COMMAND_SET_ID                  0x0003


typedef enum {
    SERIAL_OK                   = 0,
    SERIAL_INCOMPLETE           = -1,
    SERIAL_ERROR_WRONG_PREAMBLE = -2,
    SERIAL_ERROR_WRONG_CRC      = -4,
} serial_error_t;

typedef struct {
    uint16_t command;
    uint32_t dest;
    uint32_t source;
    uint8_t  data[SERIAL_PACKET_MAX_LENGTH];
    size_t   len;
} serial_packet_t;

void serial_init(void);
int  serial_get_packet(serial_packet_t *packet);
void serial_send_response(serial_packet_t *packet, uint8_t *data, size_t len);
void serial_flush(void);

#endif