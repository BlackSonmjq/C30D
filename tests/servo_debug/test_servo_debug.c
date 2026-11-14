#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "servo.h"
typedef uint8_t u8;
typedef uint32_t TickType_t;
#define USART1 ((void *)0)
#define pdMS_TO_TICKS(ms) (ms)
static uint32_t now_ms, first_out, return_time;
static unsigned scenario, outward_commands, home_commands, failures, mode_writes;
static unsigned position_queries;
static unsigned stalled_queries;
static uint32_t bus_baud=SERVO_BAUDRATE, device_baud[5];
static uint32_t known_baud[5];
static unsigned baud_writes;
static unsigned version_queries;
static char delayed_reply[64];
static uint16_t current_position=543;
static char reply[64];
uint32_t mock_primask(void) { return 0; }
void mock_disable_irq(void) {}
void mock_set_primask(uint32_t mask) { (void)mask; }
static TickType_t xTaskGetTickCount(void) { return now_ms; }
static void any_printf(void *port,const char *format,...) { (void)port; (void)format; }
void Servo_GetDiagnostics(Servo_Diagnostics *data) { memset(data,0,sizeof(*data)); }
void Servo_ClearRx(void) { reply[0]=0; }
Servo_Result Servo_SetBusBaud(uint32_t baud) { bus_baud=baud; reply[0]=0; return SERVO_OK; }
uint32_t Servo_GetBusBaud(void) { return bus_baud; }
Servo_Result Servo_SetDeviceBaud(uint8_t id,uint32_t baud) { known_baud[id]=baud; return SERVO_OK; }
Servo_Result Servo_SelectDevice(uint8_t id) { bus_baud=known_baud[id]; return SERVO_OK; }
Servo_Result Servo_StartRxProbe(void) {
    if (delayed_reply[0]) { strcpy(reply,delayed_reply); delayed_reply[0]=0; }
    return SERVO_OK;
}
Servo_Result Servo_SendCommand(const char *command) {
    unsigned id=(unsigned)(command[3]-'0');
    if (memcmp(command+4,"PVER",4)==0) ++version_queries;
    if (scenario>=8 && bus_baud!=device_baud[id]) return SERVO_OK;
    if (memcmp(command+4,"PVER",4)==0) {
        snprintf(reply,sizeof(reply),"#%03uPV0.97!",id);
        if (scenario==12) snprintf(reply,sizeof(reply),"#%03u@ ZL-ZServo_AD_CBM V2.1.16STG!",id);
        if (scenario==11) { strcpy(delayed_reply,reply); strcpy(reply,"#OK!"); }
        return SERVO_OK;
    }
    if (memcmp(command+4,"PBD0",4)==0) {
        if (scenario!=10) device_baud[id]=9600;
        ++baud_writes;
        snprintf(reply,sizeof(reply),"#%03uPBD9600!",id); return SERVO_OK;
    }
    snprintf(reply,sizeof(reply),"#%03uMOD%u!",id,id<=2?2:1);
    return SERVO_OK;
}
Servo_Result Servo_SetMode(uint8_t id,uint8_t mode) {
    if (id<1 || id>4 || mode!=(id<=2?2:1)) ++failures;
    ++mode_writes; strcpy(reply,"#OK!"); return SERVO_OK;
}
Servo_Result Servo_MovePulse(uint8_t id,uint16_t pulse,uint16_t ms) {
    Servo_SelectDevice(id);
    if (ms!=500) ++failures;
    if (pulse==1833) {
        ++outward_commands;
        if (outward_commands==1) first_out=now_ms;
        if (scenario!=1 && !((scenario==0 || scenario==4 || scenario==6) && outward_commands==1)) current_position=pulse;
    } else {
        if (pulse!=500) ++failures;
        ++home_commands;
        if (outward_commands) return_time=now_ms;
        current_position=543;
    }
    return SERVO_OK;
}
Servo_Result Servo_ReadPosition(uint8_t id) {
    Servo_SelectDevice(id);
    if (scenario>=8 && bus_baud!=device_baud[id]) return SERVO_OK;
    ++position_queries;
    if (scenario==6 && outward_commands==1 && ++stalled_queries>1U) return SERVO_OK;
    if (scenario==4 && outward_commands==1 && position_queries%2U==0U) return SERVO_OK;
    if (scenario==5 && position_queries%2U==0U) return SERVO_OK;
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
    scenario=test_case; outward_commands=home_commands=mode_writes=0;
    first_out=return_time=position_queries=stalled_queries=0; current_position=543; reply[0]=0;
    delayed_reply[0]=0;
    servo_debug_pending=(u8)request; servo_debug_busy=1;
    for (now_ms=0;now_ms<30000 && servo_debug_busy;now_ms+=50) ServoDebug_Process();
    CHECK(!servo_debug_busy);
}
int main(void) {
    unsigned id;
    CHECK(ServoDebug_Version("#002PV0.97!",11,2));
    CHECK(!ServoDebug_Version("#003PV0.97!",11,2));
    CHECK(!ServoDebug_Version("#002PVER!",9,2));
    CHECK(!ServoDebug_Version("#OK!",4,2));
    {
        const char *actual="#002@ ZL-ZServo_AD_CBM V2.1.16STG!";
        CHECK(ServoDebug_Version(actual,(uint16_t)strlen(actual),2));
        CHECK(!ServoDebug_Version(actual,(uint16_t)strlen(actual),1));
        CHECK(!ServoDebug_Version(actual,(uint16_t)strlen(actual)-1U,2));
        CHECK(!ServoDebug_Version("#002@ V2.1.16STG!",(uint16_t)strlen("#002@ V2.1.16STG!"),2));
    }
    for (id=1;id<=4;++id) device_baud[id]=known_baud[id]=115200;
    version_queries=baud_writes=0; run(8,12);
    CHECK(version_queries==4 && !baud_writes && !home_commands && !outward_commands);
    version_queries=0; run(12,12);
    CHECK(version_queries==4 && !baud_writes && !home_commands && !outward_commands);
    device_baud[2]=9600; known_baud[2]=115200;
    version_queries=0; run(11,14);
    CHECK(version_queries==2 && known_baud[2]==9600 && !baud_writes && !home_commands && !outward_commands);
    for (id=1;id<=4;++id) device_baud[id]=known_baud[id]=115200;
    baud_writes=0; run(8,11); /* Read-only scan never writes. */
    CHECK(!baud_writes && bus_baud==115200 && !home_commands && !outward_commands);
    run(10,10); /* Device accepts traffic but ignores PBD0. */
    CHECK(baud_writes==4 && bus_baud==115200);
    for (id=1;id<=4;++id) CHECK(device_baud[id]==115200 && known_baud[id]==115200);
    run(10,1); CHECK(outward_commands==1 && home_commands==2); /* S1 still works. */
    device_baud[2]=9600; baud_writes=0; run(8,11);
    CHECK(!baud_writes && known_baud[2]==9600 && known_baud[1]==115200);
    run(8,1); CHECK(outward_commands==1 && home_commands==2 && bus_baud==115200);
    run(8,2); CHECK(outward_commands==1 && home_commands==2 && bus_baud==9600);
    for (id=1;id<=4;++id) device_baud[id]=known_baud[id]=115200;
    baud_writes=0; run(8,10);
    CHECK(baud_writes==4 && !home_commands && !outward_commands && bus_baud==9600);
    for (id=1;id<=4;++id) CHECK(device_baud[id]==9600);
    baud_writes=0; run(8,10); CHECK(baud_writes==0); /* No repeated writes. */
    device_baud[2]=115200; device_baud[3]=38400; baud_writes=0; run(9,10);
    CHECK(baud_writes==1 && device_baud[2]==9600 && device_baud[3]==38400 && bus_baud==115200);
    device_baud[3]=115200; baud_writes=0; run(8,10);
    CHECK(baud_writes==1 && device_baud[3]==9600);
    run(6,2); /* Only ONE off-target reply, then silence until motion is resent. */
    CHECK(outward_commands==2); CHECK(home_commands==2);
    CHECK(return_time-first_out<3000);
    run(0,2); /* First motion command lost, valid readbacks remain at zero. */
    CHECK(outward_commands==2); CHECK(home_commands==2);
    CHECK(return_time-first_out<3000);
    run(4,2); /* Valid stalled replies alternate with missing queries. */
    CHECK(outward_commands==2); CHECK(home_commands==2);
    CHECK(return_time-first_out<3000);
    run(1,2); /* Permanent stall must have bounded retries and return. */
    CHECK(outward_commands==3); CHECK(home_commands==2);
    CHECK(return_time-first_out<=3100);
    run(2,2); /* No feedback must not be accepted as arrival. */
    CHECK(outward_commands==0); CHECK(home_commands==2);
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
    if (!failures) printf("PASS: PVER, baud discovery/recovery, feedback sequence, all-home and modes\n");
    return failures;
}
