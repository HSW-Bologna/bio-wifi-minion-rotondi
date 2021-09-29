#include "freertos/projdefs.h"
#include "hardwareprofile.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "config/app_config.h"
#include <sys/types.h>
#include <assert.h>
#include <string.h>
#include "peripherals/digin.h"
#include "peripherals/digout.h"
#include "peripherals/storage.h"
#include "serial.h"

#define PORTNUM 1
// Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin
#define ECHO_READ_TOUT (3)     // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
#define HEADER_LENGTH  15


static int     serial_parse_command(uint8_t *buffer, size_t len, serial_packet_t *packet);
static int     serial_build_response(uint8_t *buffer, size_t len, uint32_t dest, uint32_t source, uint8_t *data,
                                     size_t datalen);
static uint8_t crc_calc(uint8_t *buffer, size_t len);


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
    uint8_t buffer[SERIAL_PACKET_MAX_LENGTH] = {0};
    int     len = uart_read_bytes(PORTNUM, buffer, SERIAL_PACKET_MAX_LENGTH, pdMS_TO_TICKS(2));
    if (len > 0) {
        return serial_parse_command(buffer, len, packet);
    } else {
        return SERIAL_ERROR_NO_BYTES;
    }
}


void serial_send_response(serial_packet_t *packet, uint8_t *data, size_t len) {
    uint8_t response[SERIAL_PACKET_MAX_LENGTH] = {0};
    size_t  response_len =
        serial_build_response(response, SERIAL_PACKET_MAX_LENGTH, packet->source, packet->dest, data, len);
    uart_write_bytes(PORTNUM, response, response_len);
    ESP_LOGI(TAG, "Sent %zu bytes in response", response_len);
    ESP_LOG_BUFFER_HEX("Special", response, response_len);
}


static int serial_parse_command(uint8_t *buffer, size_t len, serial_packet_t *packet) {
    if (len < HEADER_LENGTH) {     // lunghezza pacchetto troppo corta
        ESP_LOG_BUFFER_HEX(TAG, buffer, len);
        return SERIAL_ERROR_TOO_FEW_BYTES;
    }

    size_t expected_length = (size_t)buffer[2];

    if (buffer[0] != 2) {     // primo byte errato
        ESP_LOG_BUFFER_HEX(TAG, buffer, len);
        return SERIAL_ERROR_WRONG_PREAMBLE;
    } else if (buffer[1] != 1) {     // secondo byte errato
        ESP_LOG_BUFFER_HEX(TAG, buffer, len);
        return SERIAL_ERROR_WRONG_PREAMBLE;
    } else if (expected_length > len) {     // lunghezza errata
        ESP_LOGW(TAG, "Packet says %zu bytes but only received %zu", expected_length, len);
        return -3;
    } else if (crc_calc(buffer, expected_length) == buffer[expected_length - 1]) {     // crc corretto
        packet->dest    = buffer[4] << 24 | buffer[5] << 16 | buffer[6] << 8 | buffer[7];
        packet->source  = buffer[8] << 24 | buffer[9] << 16 | buffer[10] << 8 | buffer[11];
        packet->command = (buffer[12] << 8) | buffer[13];
        memcpy(packet->data, &buffer[14], expected_length - HEADER_LENGTH);
        packet->len = expected_length - HEADER_LENGTH;
        return SERIAL_OK;
    } else {     // crc non corretto
        ESP_LOGW(TAG, "Invalid crc: 0x%02X vs 0x%02X", crc_calc(buffer, expected_length), buffer[expected_length - 1]);
        ESP_LOG_BUFFER_HEX(TAG, buffer, expected_length);
        return -10;
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
    buffer[8] = source >> 24;
    buffer[9] = source >> 16;
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
