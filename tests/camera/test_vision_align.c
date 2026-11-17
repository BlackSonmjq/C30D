#include <stdio.h>
#include <string.h>
#include "vision_align.h"

static unsigned checks;
#define REQUIRE(c) do { ++checks; if (!(c)) { printf("FAIL line %u: %s\n", __LINE__, #c); return 1; } } while (0)
static Camera_Receiver camera;
static VisionAlign_State state;
static uint32_t now;
static uint8_t manual_input;
static VisionAlign_Output update(uint8_t manual, uint8_t ready)
{ return VisionAlign_Update(&state, &camera.data, now, manual, ready); }
static VisionAlign_Output frame(uint8_t pole, int offset)
{
    unsigned i;
    uint8_t data[6] = {0x63,0,0,0,0,0x66};
    data[1] = pole; data[2] = (uint8_t)offset;
    for (i=0;i<6;++i) Camera_Push(&camera,data[i]);
    now += 50;
    return update(manual_input,1);
}
static void release_done(void) { unsigned k; for(k=0;k<20;++k) frame(1,0); }
static void arm(void) { VisionAlign_Cancel(&state); update(0,1); }
static void reset(void) { memset(&state,0,sizeof(state)); memset(&camera,0,sizeof(camera)); now=10000; manual_input=0; }
int main(void)
{
    unsigned i;
    VisionAlign_Output out;
    reset();
    out=update(0,1);
    REQUIRE(!out.owned && out.x_mps==0 && out.y_mps==0);
    REQUIRE(state.phase==VISION_ALIGN_WAIT);
    for(i=0;i<20;++i) { out=update(0,1); REQUIRE(!out.owned && out.y_mps==0); }
    REQUIRE(state.confirmed==0);
    out=frame(1,60); REQUIRE(out.y_mps==0);
    out=update(1,1); REQUIRE(out.owned && state.phase==VISION_ALIGN_ACQUIRE && out.y_mps==0);
    out=frame(1,60); REQUIRE(out.y_mps==0);
    out=frame(1,60); REQUIRE(state.phase==VISION_ALIGN_TRACK && out.x_mps==0 && out.y_mps==.08f);
    out=update(1,1); REQUIRE(state.phase==VISION_ALIGN_TRACK && out.owned && out.y_mps==.08f);
    out=frame(1,-128); REQUIRE(out.y_mps==-.08f);
    out=frame(1,15); REQUIRE(out.y_mps>.044999f && out.y_mps<.045001f);
    out=frame(1,-15); REQUIRE(out.y_mps>-.045001f && out.y_mps<-.044999f);
    out=frame(1,1); REQUIRE(out.y_mps==.03f);
    out=frame(1,-1); REQUIRE(out.y_mps==-.03f);
    out=frame(1,0); REQUIRE(out.y_mps==0 && state.phase==VISION_ALIGN_TRACK);
    out=update(0,1); REQUIRE(state.zero_frames==1 && state.phase==VISION_ALIGN_TRACK);
    out=frame(1,0); REQUIRE(state.phase==VISION_ALIGN_TRACK);
    out=frame(1,1); REQUIRE(state.zero_frames==0 && out.y_mps>0);
    frame(1,0); frame(1,0); out=frame(1,0);
    REQUIRE(out.owned && out.x_mps==0 && out.y_mps==0 && state.phase==VISION_ALIGN_DONE); release_done();
    for(i=0;i<10;++i) { out=frame(1,i%2?2:-2); REQUIRE(!out.owned && out.y_mps==0 && state.phase==VISION_ALIGN_DONE); }
    out=frame(1,4); REQUIRE(state.phase==VISION_ALIGN_DONE && out.y_mps==0);
    for(i=0;i<10;++i) { out=update(0,1); REQUIRE(state.restart_frames==1 && state.phase==VISION_ALIGN_DONE && out.y_mps==0); }
    out=frame(1,-4); REQUIRE(state.restart_frames==1 && state.phase==VISION_ALIGN_DONE);
    out=frame(1,4); REQUIRE(state.restart_frames==1 && state.phase==VISION_ALIGN_DONE);
    out=frame(1,0); REQUIRE(state.restart_frames==0 && state.phase==VISION_ALIGN_DONE);
    frame(1,10); out=frame(1,10); REQUIRE(state.phase==VISION_ALIGN_DONE && out.y_mps==0);
    out=frame(1,10); REQUIRE(state.phase==VISION_ALIGN_TRACK && out.y_mps>.029999f && out.y_mps<.030001f);
    frame(1,0); frame(1,0); out=frame(1,0); REQUIRE(state.phase==VISION_ALIGN_DONE && out.y_mps==0); release_done();
    frame(1,-15); frame(1,-15); out=frame(1,-15);
    REQUIRE(state.phase==VISION_ALIGN_TRACK && out.y_mps<-.044999f && out.y_mps>-.045001f);
    frame(1,0); frame(1,0); frame(1,0); release_done();
    frame(1,3); frame(1,3); out=frame(1,3);
    REQUIRE(state.phase==VISION_ALIGN_TRACK && out.y_mps==.03f);
    frame(1,0); frame(1,0); out=frame(1,0); REQUIRE(state.phase==VISION_ALIGN_DONE && out.y_mps==0); release_done();
    out=update(1,1); REQUIRE(!out.owned && state.phase==VISION_ALIGN_DONE);
    out=frame(1,100); REQUIRE(!out.owned && out.y_mps==0);

    /* No new target at arming, malformed input and cached B must never move. */
    reset(); arm(); now+=299; out=update(0,1);
    REQUIRE(!out.owned && state.phase==VISION_ALIGN_WAIT);
    out=frame(1,60); REQUIRE(out.owned && state.phase==VISION_ALIGN_ACQUIRE);
    now+=299; out=update(0,1);
    REQUIRE(out.owned && state.phase==VISION_ALIGN_ACQUIRE && out.y_mps==0);
    ++now; out=update(0,1);
    REQUIRE(out.owned && state.phase==VISION_ALIGN_LOST && out.y_mps==0);
    out=frame(1,60); REQUIRE(out.y_mps==0 && state.phase==VISION_ALIGN_ACQUIRE);
    arm(); frame(1,60); frame(1,60); frame(1,60);
    out=frame(0,0); REQUIRE(out.owned && state.phase==VISION_ALIGN_LOST && out.y_mps==0);
    out=frame(1,60); REQUIRE(state.phase==VISION_ALIGN_ACQUIRE && out.y_mps==0);
    frame(1,60); out=frame(1,60); REQUIRE(state.phase==VISION_ALIGN_TRACK && out.y_mps>0);
    arm(); frame(1,60); frame(1,60); frame(1,60);
    now+=300; out=update(0,1);
    REQUIRE(state.phase==VISION_ALIGN_LOST && out.y_mps==0);
    arm(); frame(1,60); frame(1,60); frame(1,60);
    now+=300; out=frame(1,60); /* New packet after a long scheduling gap cannot hide expiry. */
    REQUIRE(state.phase==VISION_ALIGN_LOST && out.y_mps==0);
    arm(); frame(1,60); frame(1,60); frame(1,60);
    out=update(0,0); REQUIRE(!out.owned && state.phase==VISION_ALIGN_IDLE);
    out=frame(1,60); REQUIRE(out.owned && state.phase==VISION_ALIGN_ACQUIRE);
    out=update(1,0); REQUIRE(!out.owned && state.phase==VISION_ALIGN_IDLE);

    reset(); arm();
    frame(1,100);
    for(i=0;i<199;++i) frame(1,100);
    REQUIRE(state.phase==VISION_ALIGN_TRACK);
    out=frame(1,100); REQUIRE(state.phase==VISION_ALIGN_TIMEOUT && out.y_mps==0);
    {
        uint32_t stopped=now;
        out=frame(1,0); REQUIRE(state.phase==VISION_ALIGN_TIMEOUT && out.y_mps==0);
        now=stopped+999; out=update(0,1); REQUIRE(state.phase==VISION_ALIGN_TIMEOUT && out.y_mps==0);
        ++now; out=update(0,1); REQUIRE(state.phase==VISION_ALIGN_TIMEOUT && out.y_mps==0);
        out=frame(1,60); REQUIRE(state.phase==VISION_ALIGN_ACQUIRE && out.y_mps==0);
        out=frame(1,60); REQUIRE(state.phase==VISION_ALIGN_ACQUIRE && out.y_mps==0);
        out=frame(1,60); REQUIRE(state.phase==VISION_ALIGN_TRACK && out.y_mps>0);
    }
    reset(); now=0xffffff00U; arm();
    frame(1,60); frame(1,60); out=frame(1,60);
    REQUIRE(state.phase==VISION_ALIGN_TRACK && out.y_mps>0);
    frame(1,0); frame(1,0); out=frame(1,0);
    REQUIRE(state.phase==VISION_ALIGN_DONE && out.y_mps==0);
    reset(); now=0xffffd800U; arm();
    frame(1,60); for(i=0;i<200;++i) frame(1,60);
    REQUIRE(state.phase==VISION_ALIGN_TIMEOUT);
    now+=999; out=update(0,1); REQUIRE(state.phase==VISION_ALIGN_TIMEOUT && out.y_mps==0);
    out=frame(1,60); REQUIRE(state.phase==VISION_ALIGN_ACQUIRE && out.y_mps==0);
    frame(1,60); out=frame(1,60); REQUIRE(state.phase==VISION_ALIGN_TRACK && out.y_mps>0);
    reset();manual_input=1;
    out=frame(1,60); REQUIRE(!out.owned && state.phase==VISION_ALIGN_WAIT);
    for(i=0;i<10;++i) {out=update(1,1); REQUIRE(!out.owned);}
    out=frame(0,0); REQUIRE(!out.owned && state.manual_confirmed==0);
    out=frame(1,60); REQUIRE(!out.owned);
    out=frame(1,60); REQUIRE(!out.owned);
    out=frame(1,60); REQUIRE(out.owned && state.phase==VISION_ALIGN_TRACK && out.y_mps==.08f);
    frame(1,0);frame(1,0);out=frame(1,0); REQUIRE(state.phase==VISION_ALIGN_DONE); release_done();
    for(i=0;i<5;++i) {out=frame(1,0); REQUIRE(!out.owned && state.phase==VISION_ALIGN_DONE);}
    frame(1,-5);frame(1,5);out=frame(1,-5); REQUIRE(!out.owned);
    frame(1,-5);out=frame(1,-5); REQUIRE(out.owned && state.phase==VISION_ALIGN_TRACK);
    /* Hold deadline and unsigned tick wrap, with held manual input. */
    reset();now=0xffffff00U;frame(1,60);frame(1,60);frame(1,60);
    frame(1,0);frame(1,0);frame(1,0);
    { uint32_t stopped=now;
      now=stopped+999U;out=update(1,1); REQUIRE(out.owned && out.y_mps==0);
      now=stopped+1000U;out=update(1,1); REQUIRE(!out.owned);
    }
    printf("PASS: vision DX alignment state machine (%u checks)\n",checks);
    return 0;
}
