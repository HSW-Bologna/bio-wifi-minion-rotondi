#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "controller.h"
#include "model/model.h"
#include "peripherals/storage.h"
#include "peripherals/serial.h"
#include "peripherals/digin.h"
#include "peripherals/digout.h"
#include "peripherals/hardwareprofile.h"
#include "esp_log.h"


#define ID_KEY "UNIT_ID"


static void set_rele_multi_pulse(uint8_t rele, uint8_t n_pulse, uint8_t P1, uint8_t P2, uint8_t p1, uint8_t p2);


static const char *TAG = "Controller";

extern uint8_t last_command[];
extern size_t  last_command_size;


void controller_init(model_t *pmodel) {
    storage_init();

    if (load_uint32_option(&pmodel->id, ID_KEY)) {
        pmodel->id = 0x00000001;
    }
    if (pmodel->id == 0) {
        pmodel->id = 0x00000001;
    }

    ESP_LOGI(TAG, "Id: %02X", pmodel->id);
}


void controller_manage_packet(model_t *pmodel) {
    serial_packet_t packet = {0};
    int             res    = serial_get_packet(&packet);

    if (res == SERIAL_OK && ((packet.dest == pmodel->id) || (packet.command == COMMAND_SET_ID))) {
        // ESP_LOGI(TAG, "%X %X %X %X %i", packet.source, packet.dest, pmodel->id, packet.command, packet.len);
        switch (packet.command) {
            case (COMMAND_READ_INPUT): {
                uint8_t data = digin_get_inputs();
                serial_send_response(&packet, &data, 1);
                break;
            }

            case (COMMAND_READ_OUTPUT): {
                uint8_t data = digout_get();
                serial_send_response(&packet, &data, 1);
                break;
            }

            case (COMMAND_SET_OUTPUT_SINGLE_PULSE): {
                uint8_t data = 0;
                serial_send_response(&packet, &data, 1);

                uint8_t relemap = packet.data[0];
                uint8_t n_pulse = 0;
                uint8_t P1      = packet.data[1];
                uint8_t P2      = packet.data[2];
                set_rele_multi_pulse(relemap, n_pulse, P1, P2, 0, 0);
                serial_flush();
                break;
            }

            case (COMMAND_SET_OUTPUT_MULTI_PULSE): {
                uint8_t data = 0;
                serial_send_response(&packet, &data, 1);

                uint8_t relemap = packet.data[0];
                uint8_t n_pulse = packet.data[1];
                uint8_t P1      = packet.data[2];
                uint8_t P2      = packet.data[3];
                uint8_t p1      = packet.data[4];
                uint8_t p2      = packet.data[5];
                set_rele_multi_pulse(relemap, n_pulse, P1, P2, p1, p2);
                serial_flush();
                break;
            }

            case (COMMAND_SET_ID): {
                uint8_t data = 0;
                serial_send_response(&packet, &data, 1);

                uint32_t val = packet.data[0] << 24 | packet.data[1] << 16 | packet.data[2] << 8 | packet.data[3];
                if (val != 0) {
                    save_uint32_option(&val, ID_KEY);
                    ESP_LOGI(TAG, "New id: %02X", val);
                    pmodel->id = val;
                }
                break;
            }

            default:
                break;
        }
        //ESP_LOGI(TAG, "Managed Command %04X", packet.command);
        //ESP_LOG_BUFFER_HEX(TAG, last_command, last_command_size);
    } else if (res != SERIAL_OK && res != SERIAL_INCOMPLETE) {
        ESP_LOGW(TAG, "Received invalid packet: %i", res);
    }
}


static void set_rele_multi_pulse(uint8_t rele, uint8_t n_pulse, uint8_t P1, uint8_t P2, uint8_t p1, uint8_t p2) {
    size_t i;
    if (n_pulse == 0)
        n_pulse = 1;
    for (i = 0; i < n_pulse; i++) {
        if (rele & 1) {
            digout_rele_update(RELE1, 1);
        }
        if (rele & 2) {
            digout_rele_update(RELE2, 1);
        }
        if (rele & 4) {
            digout_rele_update(RELE3, 1);
        }
        if (rele & 8) {
            digout_rele_update(RELE4, 1);
        }
        vTaskDelay(pdMS_TO_TICKS((P1 * 256 + P2) * 100));
        digout_rele_clear_all();
        vTaskDelay(pdMS_TO_TICKS((p1 * 256 + p2) * 100));
    }
}