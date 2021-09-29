#include <assert.h>
#include "model.h"

void model_init(model_t *pmodel) {
    assert(pmodel != NULL);
    pmodel->id = 0;
}