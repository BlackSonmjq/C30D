#ifndef ROUTE_TEST_STEPPER_H
#define ROUTE_TEST_STEPPER_H
#include <stdint.h>
enum { STEPPER_WAITING_COMMAND=0,STEPPER_RETURNED=6,STEPPER_FAULT=7 };
typedef struct { uint8_t state,result; uint32_t previous_phase_ms; } StepperTask_Data;
uint8_t StepperTask_IsBusy(void);
uint8_t StepperTask_RequestMango(uint8_t id);
void StepperTask_GetData(StepperTask_Data *data);
#endif
