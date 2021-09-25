#include "hal/gpio_types.h"
#include "peripherals/hardwareprofile.h"
#include "peripherals/digout.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<RELE1) | (1ULL<<RELE2) | (1ULL<<RELE3) | (1ULL<<RELE4))

void digout_init(void) {
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void digout_rele_update(gpio_num_t rele, int val) {
    val = val > 0;
    gpio_set_level(rele, val);
}

void digout_rele_clear_all(void) {
    gpio_set_level(RELE1, 0);
    gpio_set_level(RELE2, 0);
    gpio_set_level(RELE3, 0);
    gpio_set_level(RELE4, 0);
}

uint8_t digout_get(void) {
    uint8_t res=0;
    res|=gpio_get_level(RELE1);
    res|=(gpio_get_level(RELE2))<<1;
    res|=(gpio_get_level(RELE3))<<2;
    res|=(gpio_get_level(RELE4))<<3;
    return res;
}


