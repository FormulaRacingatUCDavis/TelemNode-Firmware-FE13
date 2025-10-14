#ifndef TELEM_NODE_H
#define TELEM_NODE_H


#include "main.h"
#include "config.h"


// PUBLIC FUNCTION PROTOTYPES
void TelemNode_Init();
void TelemNode_Update();
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif
