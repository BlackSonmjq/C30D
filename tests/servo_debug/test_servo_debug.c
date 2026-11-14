#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "servo.h"
typedef uint8_t u8;
typedef uint32_t TickType_t;
#define USART1 ((void *)0)
#define pdMS_TO_TICKS(ms) (ms)
static uint32_t now_ms;
static unsigned scenario, outward_commands, home_commands, failures, mode_writes;
static unsigned position_queries;
static unsigned stalled_queries, stops;
static uint8_t fault;
static uint16_t current_position=543;
static char reply[64];
uint32_t mock_primask(void) { return 0; }
void mock_disable_irq(void) {}
void mock_set_primask(uint32_t mask) { (void)mask; }
static TickType_t xTaskGetTickCount(void) { return now_ms; }
static void any_printf(void *port,const char *format,...) { (void)port; (void)format; }
uint8_t Servo_IsFaulted(void) { return fault; }
Servo_Result Servo_Stop(uint8_t id) { (void)id; ++stops; return SERVO_OK; }
Servo_Result Servo_FaultStop(uint8_t id) { fault=1; return Servo_Stop(id); }
void Servo_GetDiagnostics(Servo_Diagnostics *data) { memset(data,0,sizeof(*data)); }
void Servo_ClearRx(void) { reply[0]=0; }
Servo_Result Servo_SendCommand(const char *command) {
    unsigned id=(unsigned)(command[3]-'0');
    snprintf(reply,sizeof(reply),"#%03uMOD%u!",id,id<=2?2:1);
    return SERVO_OK;
}
Servo_Result Servo_SetMode(uint8_t id,uint8_t mode) {
    if (id<1 || id>4 || mode!=(id<=2?2:1)) ++failures;
    ++mode_writes; strcpy(reply,"#OK!"); return SERVO_OK;
}
Servo_Result Servo_MovePulse(uint8_t id,uint16_t pulse,uint16_t ms) {
    (void)id;
    if (ms!=500) ++failures;
    if (pulse==1833) {
        ++outward_commands;
        if (scenario!=1 && !((scenario==0 || scenario==4 || scenario==6) && outward_commands==1)) current_position=pulse;
    } else {
        if (pulse!=500) ++failures;
        ++home_commands;
        current_position=543;
    }
    return SERVO_OK;
}
Servo_Result Servo_ReadPosition(uint8_t id) {
    ++position_queries;
    if (scenario==6 && outward_commands==1 && ++stalled_queries>1U) return SERVO_OK;
    if (scenario==4 && outward_commands==1 && position_queries%2U==0U) return SERVO_OK;
    if (scenario==5 && position_queries%2U==0U) return SERVO_OK;
    if (scenario==7 && outward_commands) current_position=2493;
    if (scenario!=2) snprintf(reply,sizeof(reply),"#%03uP%04u!",(unsigned)id,(unsigned)current_position);
    return SERVO_OK;
}
Servo_Result Servo_GetFrame(char *buffer,uint16_t capacity,uint16_t *length) {
    if (!reply[0]) return SERVO_NO_FRAME;
    *length=(uint16_t)strlen(reply);
    if (capacity<=*length) return SERVO_BUFFER_TOO_SMALL;
    strcpy(buffer,reply); reply[0]=0; return SERVO_OK;
}
#include "production_logic.inc"
#define CHECK(condition) do { if (!(condition)) { printf("FAIL line %u scenario %u\n",__LINE__,scenario); ++failures; } } while (0)
static void run(unsigned test_case,unsigned request) {
    fault=0; stops=0; scenario=test_case; outward_commands=home_commands=mode_writes=0;
    position_queries=stalled_queries=0; current_position=543; reply[0]=0;
    servo_debug_pending=(u8)request; servo_debug_busy=1;
    for (now_ms=0;now_ms<12000 && servo_debug_busy;now_ms+=50) ServoDebug_Process();
    CHECK(!servo_debug_busy);
}
int main(void) {
    unsigned i;
    for (i=0;i<=6;++i) {
        if (i==3 || i==5) continue;
        run(i,2);
        CHECK(stops==1 && fault);
        CHECK(home_commands==1);
        CHECK(outward_commands==(i==2 ? 0U : 1U));
        /* Further S2 after a fault must not emit another move. */
        servo_debug_pending=2; servo_debug_busy=1;
        ServoDebug_Process();
        CHECK(!servo_debug_busy && home_commands==1 && stops==1);
    }
    run(7,2); /* Feedback crosses the commanded travel envelope. */
    CHECK(stops==1 && fault && home_commands==1 && outward_commands==1);
    run(3,2); /* Normal motion needs no retransmission. */
    CHECK(outward_commands==1); CHECK(home_commands==2);
    run(5,2); /* Arrival remains valid across intermittent missing replies. */
    CHECK(outward_commands==1); CHECK(home_commands==2);
    CHECK(now_ms<5000);
    run(5,2); /* Repeat S2 after completion, with the same intermittent link. */
    CHECK(outward_commands==1); CHECK(home_commands==2);
    CHECK(now_ms<5000);
    run(3,5); /* All-servo home has no outward motion. */
    CHECK(outward_commands==0); CHECK(home_commands==4);
    run(3,6); CHECK(mode_writes==1 && !home_commands && !outward_commands);
    run(3,7); CHECK(mode_writes==1 && !home_commands && !outward_commands);
    run(3,8); CHECK(mode_writes==1 && !home_commands && !outward_commands);
    run(3,9); CHECK(mode_writes==1 && !home_commands && !outward_commands);
    if (!failures) printf("PASS: fault-stop, no retransmit or automatic return, latched S2, travel guard, normal motion, modes\n");
    return failures;
}
