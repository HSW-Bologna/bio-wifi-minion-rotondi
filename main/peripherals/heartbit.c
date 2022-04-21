#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "hardwareprofile.h"
#include "heartbit.h"


static void heartbit_print_heap_status(void);


static const char   *TAG       = "Heartbeat";
static TimerHandle_t timer     = NULL;
static unsigned long hb_period = 1000UL;

/**
 * Timer di attivita'. Accende e spegne il led di attivita'
 */
static void heartbit_timer(void *arg) {
    (void)arg;
    static int blink = 0;

    gpio_set_level(HAP_RUN, blink);
    blink = !blink;

    xTimerChangePeriod(timer, pdMS_TO_TICKS(hb_period), portMAX_DELAY);
}


void heartbit_init(size_t period_ms) {
    gpio_set_direction(HAP_RUN, GPIO_MODE_OUTPUT);
    hb_period = period_ms;
    timer     = xTimerCreate("idle", pdMS_TO_TICKS(hb_period), pdTRUE, NULL, heartbit_timer);
    xTimerStart(timer, portMAX_DELAY);
    heartbit_print_heap_status();
}



static void heartbit_print_heap_status(void) {
    ESP_LOGI(TAG, "Low water mark: %i, free memory %i", xPortGetMinimumEverFreeHeapSize(), xPortGetFreeHeapSize());
}