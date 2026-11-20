#include <stdio.h>
#include <string.h>
#include "vision_strike.h"

static unsigned checks;
#define REQUIRE(c) do { ++checks; if (!(c)) { printf("FAIL line %u: %s\n", __LINE__, #c); return 1; } } while (0)
static Camera_Receiver camera;
static uint32_t tick;
static uint8_t Check, ready = 1, stepper_busy, servo_busy, debug_busy, reserved, owned;
static unsigned direct_calls, queue_calls;
static uint8_t submitted, reject;
#define portTICK_PERIOD_MS 1U
static uint32_t xTaskGetTickCount(void) { return tick; }
void Camera_GetData(Camera_Data *data) { *data = camera.data; }
static uint8_t RouteIO_Ready(void) { return ready && !stepper_busy; }
static uint8_t StepperTask_IsBusy(void) { return stepper_busy; }
static uint8_t ServoTask_IsBusy(void) { return servo_busy; }
static uint8_t USART_ServoControlBusy(void) { return debug_busy; }
static uint8_t RouteIO_WorkReserved(void) { return reserved; }
static uint8_t RouteIO_ControlOwned(void) { return owned; }
static uint8_t RouteIO_QueueWorkFromISR(uint8_t id)
{
    ++queue_calls;
    if (reject || reserved || stepper_busy) return 0;
    submitted = id; reserved = 1;
    return 1;
}
static uint8_t StepperTask_RequestMango(uint8_t id)
{
    ++direct_calls;
    if (reject || stepper_busy) return 0;
    submitted = id; stepper_busy = 1;
    return 1;
}
#include "production_dispatch.inc"

static uint8_t observe(VisionStrike_State *state, Camera_Data *data, uint8_t id,
                       uint32_t now, uint8_t blocked)
{
    ++data->frames;
    data->pole = data->green = id != 0;
    data->region = id;
    return VisionStrike_Update(state, data, now, blocked);
}
static void frame(uint8_t id)
{
    uint8_t bytes[6] = {0x63, 0, 0, 0, 0, 0x66};
    unsigned i;
    bytes[1] = bytes[3] = id != 0; bytes[4] = id;
    for (i = 0; i < 6; ++i) Camera_Push(&camera, bytes[i]);
    tick += 50;
    VisionStrike_Process();
}
static void clear_target(void) { frame(0); frame(0); frame(0); }

int main(void)
{
    VisionStrike_State s = {0};
    Camera_Data d = {0};
    unsigned i, before;
    uint8_t *gate;
    REQUIRE(VisionStrike_Update(&s, &d, 0, 0) == 0);
    REQUIRE(observe(&s, &d, 1, 50, 0) == 0);
    for (i = 0; i < 10; ++i) REQUIRE(VisionStrike_Update(&s, &d, 50, 0) == 0);
    REQUIRE(s.confirmed == 1);
    REQUIRE(observe(&s, &d, 1, 100, 0) == 0);
    REQUIRE(observe(&s, &d, 2, 150, 0) == 0); /* Mixed frames need a three-vote majority. */
    REQUIRE(observe(&s, &d, 2, 200, 0) == 0);
    REQUIRE(observe(&s, &d, 2, 250, 0) == 2);
    VisionStrike_Accept(&s);
    for (i = 0; i < 30; ++i) REQUIRE(observe(&s, &d, (uint8_t)(i % 4 + 1), 300 + 50*i, 0) == 0);
    REQUIRE(s.latched);
    REQUIRE(VisionStrike_Update(&s, &d, 5000, 0) == 0);
    REQUIRE(s.latched); /* disconnect is not proof of fruit removal */
    REQUIRE(observe(&s, &d, 0, 5050, 0) == 0);
    REQUIRE(observe(&s, &d, 0, 5100, 0) == 0);
    REQUIRE(s.latched);
    REQUIRE(observe(&s, &d, 1, 5150, 0) == 0); /* target present resets absence */
    REQUIRE(observe(&s, &d, 0, 5200, 0) == 0);
    REQUIRE(observe(&s, &d, 0, 5250, 1) == 0); /* busy absence cannot rearm */
    REQUIRE(observe(&s, &d, 0, 5300, 0) == 0);
    REQUIRE(observe(&s, &d, 0, 5350, 0) == 0);
    REQUIRE(s.latched);
    REQUIRE(observe(&s, &d, 0, 5400, 0) == 0);
    REQUIRE(!s.latched);
    REQUIRE(observe(&s, &d, 4, 5450, 0) == 0);
    REQUIRE(observe(&s, &d, 4, 5750, 0) == 0); /* exact stale limit */
    REQUIRE(s.confirmed == 1);
    REQUIRE(observe(&s, &d, 4, 5800, 0) == 0);
    REQUIRE(VisionStrike_Update(&s, &d, 5801, 1) == 0); /* blocked cached snapshot */
    REQUIRE(observe(&s, &d, 4, 5850, 0) == 0);
    REQUIRE(s.confirmed == 1);
    REQUIRE(observe(&s, &d, 4, 5900, 0) == 0);
    REQUIRE(observe(&s, &d, 4, 5950, 0) == 4);
    memset(&s, 0, sizeof(s));
    REQUIRE(observe(&s, &d, 4, 0xffffffceU, 0) == 0);
    REQUIRE(observe(&s, &d, 4, 0, 0) == 0);
    REQUIRE(observe(&s, &d, 4, 50, 0) == 4); /* clock wrap */

    /* Majority tolerates one wrong number, but never fires the current wrong number. */
    { VisionStrike_State vote={0};Camera_Data observation={0};
      REQUIRE(observe(&vote,&observation,2,100,0)==0);
      REQUIRE(observe(&vote,&observation,2,150,0)==0);
      REQUIRE(observe(&vote,&observation,4,200,0)==0);
      REQUIRE(observe(&vote,&observation,2,250,0)==2);
      REQUIRE(observe(&vote,&observation,4,300,0)==0 && vote.candidate==2);
      REQUIRE(observe(&vote,&observation,4,350,0)==4 && vote.candidate==4);
      VisionStrike_Accept(&vote);
      REQUIRE(vote.count==0 && vote.confirmed==0);
      REQUIRE(observe(&vote,&observation,2,400,0)==0 && vote.latched);
    }
    /* An old vote expires even when intervening traffic keeps the link fresh. */
    { VisionStrike_State vote={0};Camera_Data observation={0};
      REQUIRE(observe(&vote,&observation,2,0,0)==0);
      REQUIRE(observe(&vote,&observation,2,100,0)==0);
      REQUIRE(observe(&vote,&observation,4,200,0)==0);
      REQUIRE(observe(&vote,&observation,2,300,0)==0 && vote.confirmed==2);
      REQUIRE(observe(&vote,&observation,2,400,0)==0);
      REQUIRE(observe(&vote,&observation,2,450,0)==2);
      observation.region=0;++observation.frames;
      REQUIRE(VisionStrike_Update(&vote,&observation,460,0)==0 && vote.count==0);
    }

    /* Actual dispatcher + actual six-byte parser, one complete job per fruit. */
    for (i = 1; i <= 4; ++i) {
        if(i==3) continue;
        stepper_busy = 0;
        clear_target();
        before = direct_calls;
        frame((uint8_t)i); frame((uint8_t)i);
        REQUIRE(direct_calls == before);
        frame((uint8_t)i);
        REQUIRE(direct_calls == before + 1 && submitted == i && stepper_busy);
        frame((uint8_t)i); frame((uint8_t)i);
        REQUIRE(direct_calls == before + 1);
        stepper_busy = 0;
        frame((uint8_t)i); frame((uint8_t)(i % 4 + 1));
        REQUIRE(direct_calls == before + 1);
        tick += 1000; VisionStrike_Process();
        frame((uint8_t)i); frame((uint8_t)i); frame((uint8_t)i);
        REQUIRE(direct_calls == before + 1);
    }
    clear_target(); owned = 1;
    before = direct_calls;
    frame(4); frame(4); frame(4);
    REQUIRE(queue_calls == 1 && reserved && submitted == 4 && direct_calls == before);
    clear_target(); /* reserved navigation job must keep latch even if target disappears */
    reserved = 0;
    frame(4); frame(4); frame(4);
    REQUIRE(queue_calls == 1);
    clear_target(); owned = 0;

    /* Startup/low voltage/disabled base via Ready, servo, debug, Check, reserved. */
    for (i = 0; i < 5; ++i) {
        gate = i == 0 ? &ready : i == 1 ? &servo_busy : i == 2 ? &debug_busy : i == 3 ? &Check : &reserved;
        *gate = i == 0 ? 0 : 1;
        before = direct_calls;
        frame(1); frame(1); frame(1); frame(1);
        REQUIRE(direct_calls == before);
        *gate = i == 0 ? 1 : 0;
        VisionStrike_Process(); /* cached observation still cannot fire */
        frame(1); frame(1);
        REQUIRE(direct_calls == before);
        frame(1);
        REQUIRE(direct_calls == before + 1);
        stepper_busy = 0; clear_target();
    }
    /* Submission rejection never latches; retry needs a new valid observation. */
    reject = 1; before = direct_calls;
    frame(2); frame(2); frame(2);
    REQUIRE(direct_calls == before + 1 && !stepper_busy);
    reject = 0; VisionStrike_Process();
    REQUIRE(direct_calls == before + 1);
    frame(2);
    REQUIRE(direct_calls == before + 2 && submitted == 2);
    stepper_busy = 0; clear_target();
    before = direct_calls;
    frame(4); frame(4);
    /* Invalid region rejected by parser; cached good frame must not qualify. */
    frame(5); frame(5); frame(5);
    REQUIRE(direct_calls == before);
    tick += 300; VisionStrike_Process();
    frame(4); frame(4);
    REQUIRE(direct_calls == before);
    frame(4);
    REQUIRE(direct_calls == before + 1 && submitted == 4);
    printf("PASS: vision strike qualifier and production dispatch (%u checks)\n", checks);
    return 0;
}
