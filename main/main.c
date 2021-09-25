#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "model/model.h"
#include "controller/controller.h"
#include "peripherals/digin.h"
#include "peripherals/digout.h"
#include "peripherals/hardwareprofile.h"
#include "peripherals/serial.h"
#include "peripherals/storage.h"

static const char *TAG = "Main";

void app_main(void) {
    model_t model;

    model_init(&model);
    // view_init(&model);
    controller_init(&model);
    digout_init();
    digin_init();
    storage_init();
    serial_init();


    ESP_LOGI(TAG, "Begin main loop");
    for (;;) {
        digout_rele_update(RELE1, digin_get(DIGIN_1));
        digout_rele_update(RELE2, digin_get(DIGIN_2));
        digout_rele_update(RELE3, digin_get(DIGIN_3));
        digout_rele_update(RELE4, digin_get(DIGIN_4));        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

