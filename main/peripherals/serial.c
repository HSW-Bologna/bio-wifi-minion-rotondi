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

#define PORTNUM 1
// Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin
#define ECHO_READ_TOUT (3)     // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
static const char* TAG = "Serial";
#define ID_KEY "identificativo"
#define COMMAND_READ_INPUT 0x0101
#define COMMAND_READ_OUTPUT 0x0102
#define COMMAND_SET_OUTPUT_SINGLE_PULSE 0x0202
#define COMMAND_SET_OUTPUT_MULTI_PULSE 0x0203
#define COMMAND_SET_ID 0x0003

typedef struct {
    uint16_t command;
    uint8_t data[256];
    size_t len;
} serial_packet_t;

static int serial_parse_command(uint8_t *buffer, size_t len, serial_packet_t *packet);
static int serial_build_response(uint8_t *buffer, size_t len, uint8_t *data, size_t datalen);
static uint8_t crc_calc(uint8_t *buffer, size_t len);
static void serial_set_rele_multi_pulse(uint8_t relemap, uint8_t n_pulse, uint8_t P1, uint8_t P2,uint8_t p1,  uint8_t p2);

uint32_t id;

void serial_init(void) {

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

    if (load_uint32_option(&id,ID_KEY)) {
        id = 0;
    }
    ESP_LOGI(TAG, "Id: %02X", id);

    for (;;) {
        uint8_t buffer[256] = {0};
        serial_packet_t packet;
        int len = uart_read_bytes(PORTNUM, buffer, 256, pdMS_TO_TICKS(20));
        if (len>0) {
            int err = serial_parse_command(buffer, len, &packet);
            if (!err) {
                ESP_LOGI(TAG, "Received packet with command %04X, data %i", packet.command, packet.len);
                uint8_t response[256] = {0};
                switch (packet.command) {
                    case (COMMAND_READ_INPUT): {
                        uint8_t data=digin_get_inputs();
                        ESP_LOGI(TAG, "building response with data %02X", data);
                        int response_len = serial_build_response(response, 256, &data, 1);                            ESP_LOGI(TAG, "Built response");
                        uart_write_bytes(PORTNUM, response, response_len);
                        ESP_LOGI(TAG, "response send");
                        break;
                    } case (COMMAND_READ_OUTPUT): {
                        uint8_t data=digout_get();
                        ESP_LOGI(TAG, "building response with data %02X", data);
                        int response_len = serial_build_response(response, 256, &data, 1);                            ESP_LOGI(TAG, "Built response");
                        uart_write_bytes(PORTNUM, response, response_len);
                        ESP_LOGI(TAG, "response send");
                        break;
                    } case (COMMAND_SET_OUTPUT_SINGLE_PULSE):{
                        uint8_t data=0;
                        ESP_LOGI(TAG, "building response with data %02X", data);
                        int response_len = serial_build_response(response, 256, &data, 1);                            ESP_LOGI(TAG, "Built response");
                        uart_write_bytes(PORTNUM, response, response_len);
                        ESP_LOGI(TAG, "response send");
                        uint8_t relemap = packet.data[0];
                        uint8_t n_pulse = 0;
                        uint8_t P1 = packet.data[1];
                        uint8_t P2 = packet.data[2];
                        serial_set_rele_multi_pulse(relemap,  n_pulse,  P1,  P2, 0,  0);
                        break;
                    } case (COMMAND_SET_OUTPUT_MULTI_PULSE):{
                        uint8_t data=0;
                        ESP_LOGI(TAG, "building response with data %02X", data);
                        int response_len = serial_build_response(response, 256, &data, 1);                            ESP_LOGI(TAG, "Built response");
                        uart_write_bytes(PORTNUM, response, response_len);
                        ESP_LOGI(TAG, "response send");
                        uint8_t relemap = packet.data[0];
                        uint8_t n_pulse = packet.data[1];
                        uint8_t P1 = packet.data[2];
                        uint8_t P2 = packet.data[3];
                        uint8_t p1 = packet.data[4];
                        uint8_t p2 = packet.data[5];
                        serial_set_rele_multi_pulse(relemap,  n_pulse,  P1,  P2, p1,  p2);
                        break;
                    } case (COMMAND_SET_ID): {
                        uint8_t data=0;
                        ESP_LOGI(TAG, "building response with data %02X", data);
                        int response_len = serial_build_response(response, 256, &data, 1);                            ESP_LOGI(TAG, "Built response");
                        uart_write_bytes(PORTNUM, response, response_len);
                        ESP_LOGI(TAG, "response send");
                        uint32_t val = packet.data[0]<<24|packet.data[1]<<16|packet.data[2]<<8|packet.data[3];
                        save_uint32_option(&val, ID_KEY);
                        ESP_LOGI(TAG, "Id: %02X", val);
                        id = val;
                        break;
                    } default:
                    break;
                }
            } else {
                ESP_LOGW(TAG, "Invalid packet, errore %i",err);
            }

            ESP_LOGI(TAG, "%i %c", len, buffer[0]);
        uart_write_bytes(PORTNUM, "X", 1);
        }
    }

}


static int serial_parse_command(uint8_t *buffer, size_t len, serial_packet_t *packet) {
    
    if (len<16) { //lunghezza pacchetto troppo corta
        return -5;
    } else if (buffer[0]!=2) { //primo byte errato
        return -1;
    } else if (buffer[1]!=1) { //secondo byte errato
        return -2;
    } else if (buffer[2]!=len) { //lunghezza errata
        ESP_LOGW(TAG, "len %i %i", buffer[2], len);
       return -3;
    } else if (crc_calc(buffer,len) == buffer[len-1]) { //crc corretto
                uint32_t id_dest = buffer[4]<<24|buffer[5]<<16|buffer[6]<<8|buffer[7];
                packet->command = (buffer[13]<<8)|buffer[14];
                if (id_dest==id ||packet->command==COMMAND_SET_ID) { //sono destinatario o devo cambiare id
                    memcpy(packet->data, &buffer[15], len - 16);
                    packet->len = len - 16;
                    return 0;
                } else //non sono destinatario
                return -4;
            } else { //crc non corretto
            return - 10;
        }
}

static int serial_build_response(uint8_t *buffer, size_t len, uint8_t *data, size_t datalen) {
    if (len<14) { //lunghezza troppo corta
         return 0;
    }
    buffer[0]=2;
    buffer[1]=1;
    buffer[2]= 14+datalen;
    buffer[3]=0;
    buffer[5]=id<<24;
    buffer[6]=id<<16;
    buffer[7]=id<<8;
    buffer[8]=id;
    buffer[9]=0;
    if (datalen>0) {
        memcpy(&buffer[14], data, datalen);
        buffer[15]=crc_calc(buffer, len-1);        
    }
    else {
        buffer[14]=crc_calc(buffer, len-1); 
    }
    return 14+datalen;
}

static uint8_t crc_calc(uint8_t *buffer, size_t len) {
    size_t i = 0;
    uint8_t res = 0;
    for (i=0;i<len-1;i++) {
       res += buffer[i];
    }
    return res = (res & 0xFF);
}

static void serial_set_rele_multi_pulse(uint8_t rele, uint8_t n_pulse, uint8_t P1,  uint8_t P2, uint8_t p1,uint8_t p2) {
    size_t i;
    if (n_pulse==0) 
        n_pulse=1;
    for (i=0;i<n_pulse;i++) {
        if (rele&1) {
            digout_rele_update(RELE1, 1);
        }
         if (rele&2) {
            digout_rele_update(RELE2, 1);
        }
         if (rele&4) {
            digout_rele_update(RELE3, 1);
        }
         if (rele&8) {
            digout_rele_update(RELE4, 1);
        }
        vTaskDelay(pdMS_TO_TICKS((P1*256+P2)*100));
        digout_rele_clear_all();
        vTaskDelay(pdMS_TO_TICKS((p1*256+p2)*100));
    }
}