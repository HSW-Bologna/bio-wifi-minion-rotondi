#ifndef DIGIN_H_INCLUDED
#define DIGIN_H_INCLUDED


#include "hal/gpio_types.h"
#include <string.h>
#include <stdint.h>

typedef enum {
    DIGIN_1 = 0,
    DIGIN_2,
    DIGIN_3,
    DIGIN_4,
    NUM_DIGIN,
} digin_t;

void         digin_init(void);
int          digin_get(digin_t digin);
int          digin_take_reading(void);
unsigned int digin_get_inputs(void);

#endif