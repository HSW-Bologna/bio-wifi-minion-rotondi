#ifndef MODEL_H_INCLUDED
#define MODEL_H_INCLUDED

#include <stdint.h>

typedef struct {
    uint32_t id;
} model_t;

void model_init(model_t *model);

#endif