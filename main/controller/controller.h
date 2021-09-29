#ifndef CONTROLLER_H_INCLUDED
#define CONTROLLER_H_INCLUDED

#include "model/model.h"


void controller_init(model_t *model);
void controller_manage_packet(model_t *pmodel);


#endif