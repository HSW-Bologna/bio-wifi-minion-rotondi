#include "freertos/projdefs.h"
#include "hardwareprofile.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "config/app_config.h"
#include <sys/types.h>
#include <assert.h>
#include <string.h>
#include "utils/utils.h"
#include "gel/timer/timecheck.h"
#include "peripherals/digin.h"
#include "peripherals/digout.h"
#include "peripherals/storage.h"
#include "serial.h"

#define PORTNUM 1
// Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin
#define ECHO_READ_TOUT   (3)     // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
#define HEADER_LENGTH    15
#define UART_BUFFER_SIZE 1024


static int           serial_parse_command(uint8_t *buffer, size_t len, serial_packet_t *packet, size_t *toflush);
static int           serial_build_response(uint8_t *buffer, size_t len, uint32_t dest, uint32_t source, uint8_t *data,
                                           size_t datalen);
static uint8_t       packet_buffer[UART_BUFFER_SIZE] = {0};
static size_t        packet_index                    = 0;
static unsigned long timestamp                       = 0;

static uint8_t crc_calc(uint8_t *buffer, size_t len);
static int     find_preamble(uint8_t *buffer, size_t len);



static const char *TAG = "Serial";


void serial_init(void) {
    (void)TAG;
    uart_config_t uart_config = {
        .baud_rate           = 9600,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(PORTNUM, &uart_config));

    uart_set_pin(PORTNUM, SERIAL_UART_TXD, SERIAL_UART_RXD, SERIAL_DERE, -1);
    ESP_ERROR_CHECK(uart_driver_install(PORTNUM, 512, 512, 10, NULL, 0));
    ESP_ERROR_CHECK(uart_set_mode(PORTNUM, UART_MODE_RS485_HALF_DUPLEX));
    ESP_ERROR_CHECK(uart_set_rx_timeout(PORTNUM, ECHO_READ_TOUT));
}


int serial_get_packet(serial_packet_t *packet) {
    size_t available = 0;
    if (packet_index > 0 && is_expired(timestamp, get_millis(), 10)) {
        ESP_LOGW(TAG, "Timeout");
        ESP_LOG_BUFFER_HEX(TAG, packet_buffer, packet_index);
        packet_index = 0;
    }

    uart_get_buffered_data_len(PORTNUM, &available);

    if (available == 0 && packet_index == 0) {
        vTaskDelay(1);
        return SERIAL_INCOMPLETE;
    } else if (available > 0) {
        timestamp = get_millis();

        if (packet_index + available > sizeof(packet_buffer)) {
            ESP_LOGW(TAG, "Overflow!");
            packet_index = 0;
            return SERIAL_INCOMPLETE;
        }

        int len = uart_read_bytes(PORTNUM, &packet_buffer[packet_index], available, 0);
        assert(len > 0);
        packet_index += len;
    }

    size_t toflush = 0;
    int    res     = serial_parse_command(packet_buffer, packet_index, packet, &toflush);
    //printf("%zu %zu %zu %i\n", available, packet_index, toflush, res);

    if (toflush > 0) {
        memmove(packet_buffer, &packet_buffer[toflush], packet_index - toflush);
        packet_index -= toflush;
    }

    return res;
}


void serial_flush(void) {
    timestamp = get_millis();
    uart_flush(PORTNUM);
}


void serial_send_response(serial_packet_t *packet, uint8_t *data, size_t len) {
    uint8_t response[SERIAL_PACKET_MAX_LENGTH] = {0};
    size_t  response_len =
        serial_build_response(response, SERIAL_PACKET_MAX_LENGTH, packet->source, packet->dest, data, len);
    uart_write_bytes(PORTNUM, response, response_len);
    ESP_LOGV(TAG, "Sent %zu bytes in response", response_len);
}


static serial_error_t serial_parse_command(uint8_t *buffer, size_t len, serial_packet_t *packet, size_t *toflush) {
    if (len < 2) {
        *toflush = 0;
        return SERIAL_INCOMPLETE;
    }

    int start = find_preamble(buffer, len);

    if (start < 0) {
        *toflush = len;
        return SERIAL_ERROR_WRONG_PREAMBLE;
    } else {
        *toflush = (size_t)start;
    }

    size_t remaining_length = len - start;

    if (remaining_length < HEADER_LENGTH) {     // lunghezza pacchetto troppo corta
        return SERIAL_INCOMPLETE;
    }

    size_t expected_length = (size_t)buffer[start + 2];

    if (expected_length > remaining_length) {     // lunghezza errata
        return SERIAL_INCOMPLETE;
    } else if (crc_calc(&buffer[start], expected_length) == buffer[start + expected_length - 1]) {     // crc corretto
        packet->dest = buffer[start + 4] << 24 | buffer[start + 5] << 16 | buffer[start + 6] << 8 | buffer[start + 7];
        packet->source =
            buffer[start + 8] << 24 | buffer[start + 9] << 16 | buffer[start + 10] << 8 | buffer[start + 11];
        packet->command = (buffer[start + 12] << 8) | buffer[start + 13];
        memcpy(packet->data, &buffer[start + 14], expected_length - HEADER_LENGTH);
        packet->len = expected_length - HEADER_LENGTH;
        *toflush    = start + expected_length;
        return SERIAL_OK;
    } else {     // crc non corretto
        ESP_LOGW(TAG, "Invalid crc: 0x%02X vs 0x%02X", crc_calc(buffer, expected_length), buffer[expected_length - 1]);
        ESP_LOG_BUFFER_HEX(TAG, buffer, expected_length);
        *toflush = start + expected_length;
        return SERIAL_ERROR_WRONG_CRC;
    }
}

static int serial_build_response(uint8_t *buffer, size_t len, uint32_t dest, uint32_t source, uint8_t *data,
                                 size_t datalen) {
    if (len < 14) {     // lunghezza troppo corta
        return 0;
    }
    buffer[0]  = 2;
    buffer[1]  = 1;
    buffer[2]  = 14 + datalen;
    buffer[3]  = 0;
    buffer[4]  = dest >> 24;
    buffer[5]  = dest >> 16;
    buffer[6]  = dest >> 8;
    buffer[7]  = dest;
    buffer[8]  = source >> 24;
    buffer[9]  = source >> 16;
    buffer[10] = source >> 8;
    buffer[11] = source;
    if (datalen > 0) {
        memcpy(&buffer[13], data, datalen);
        buffer[14] = crc_calc(buffer, len - 1);
    } else {
        buffer[13] = crc_calc(buffer, len - 1);
    }
    return 14 + datalen;
}


static uint8_t crc_calc(uint8_t *buffer, size_t len) {
    size_t   i   = 0;
    uint16_t res = 0;
    for (i = 0; i < len - 1; i++) {
        res = (res + buffer[i]) & 0xFF;
    }
    return (uint8_t)res;
}


static int find_preamble(uint8_t *buffer, size_t len) {
    for (int i = 0; i < len - 1; i++) {
        if (buffer[i] == 2 && buffer[i + 1] == 1) {
            return i;
        }
    }

    return -1;
}