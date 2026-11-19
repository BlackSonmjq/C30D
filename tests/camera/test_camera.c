#include <stdio.h>
#include <string.h>
#include "camera_protocol.h"
#include "route_debug.h"
#include "vision_align.h"
#include "my_chassis_task.h"
static int fails;
static RouteLidarFrame telemetry_lidar;
void RouteIO_ReadLidar(RouteLidarFrame *frame) { *frame=telemetry_lidar; }
static NavLidar telemetry_received;
static RouteNavigationStatus telemetry_nav;
static uint32_t telemetry_now;
static uint8_t telemetry_owned;
void RouteIO_ReadNavigationLidar(NavLidar *frame) { *frame=telemetry_received; }
void Route_ReadNavigationStatus(RouteNavigationStatus *status) { *status=telemetry_nav; }
uint32_t RouteIO_Millis(void) { return telemetry_now; }
uint8_t VisionAlign_ControlOwned(void) { return telemetry_owned; }
static Camera_Receiver telemetry_camera;
static VisionAlign_Diagnostics telemetry_align;
void VisionAlign_GetDiagnostics(VisionAlign_Diagnostics *data) { *data=telemetry_align; }
void Camera_GetData(Camera_Data *data) { *data=telemetry_camera.data; }
#define CHECK(x) do { if (!(x)) { printf("FAIL line %u\n",__LINE__); ++fails; } } while (0)
static void feed(Camera_Receiver *r,const unsigned char *p,unsigned n) { while(n--) Camera_Push(r,*p++); }
static float channel(const unsigned char *p,unsigned n) {
 float v; memcpy(&v,p+n*4,4); return v;
}
static void test_telemetry(void) {
 RouteController controller={0}; RouteInputs in={0}; RouteDebugFrame frozen;
 unsigned char before[236],after[236],visual[]={0x63,1,0xEC,1,3,0x66};
 RouteDebug_Reset();
 CHECK(ROUTE_VOFA_CHANNEL_COUNT==58 && ROUTE_VOFA_FRAME_BYTES==236);
 CHECK(RouteDebug_BuildPacket(before,sizeof(before))==236);
 CHECK(channel(before,0)==0 && channel(before,5)==0);
 CHECK(channel(before,45)==-1 && channel(before,57)==0);
 telemetry_align.ly=186;telemetry_align.rx=127;telemetry_align.control_flags=81;telemetry_align.rc_velocity_mmps=200;
 telemetry_align.phase=2;telemetry_align.vy_mps=-.14f;telemetry_align.wz_radps=.125f;
 telemetry_align.pick_phase=5;telemetry_align.hit_region=4;telemetry_align.hit_count=7;telemetry_align.locked_region=3;telemetry_align.lock_count=12;
 telemetry_align.yaw_target_deg=10;telemetry_align.yaw_deg=15;telemetry_align.yaw_error_deg=-5;
 telemetry_align.gyro_z_dps=-12.5f;telemetry_align.heading_i_radps=.1f;
 telemetry_align.heading_locked=1;telemetry_align.imu_fresh=1;telemetry_align.blocked=64;
 for(unsigned w=0;w<4;++w) {
  telemetry_align.wheel_target[w]=.125f*(w+1);
  telemetry_align.wheel_feedback[w]=-.0625f*(w+1);
  telemetry_align.wheel_pwm[w]=w%2 ? -16700 : 16700;
 }
 feed(&telemetry_camera,visual,6);
 /* No route capture is needed to publish fresh visual data. */
 CHECK(RouteDebug_BuildPacket(after,sizeof(after))==236);
 CHECK(channel(after,0)==1 && channel(after,1)==-20);
 CHECK(channel(after,2)==1 && channel(after,3)==3);
 CHECK(channel(after,38)==5 && channel(after,39)==4 && channel(after,40)==7);
 CHECK(channel(after,41)==3 && channel(after,42)==12);
 CHECK(channel(after,35)==6 && channel(after,36)==0 && channel(after,37)==0);
 CHECK(channel(after,4)==1 && channel(after,5)==2 && channel(after,6)==0);
 CHECK(channel(after,7)==0 && channel(after,8)==0);
 CHECK(channel(after,31)==186 && channel(after,32)==127 && channel(after,33)==81 && channel(after,34)==200);
 CHECK(channel(after,9)==-.14f && channel(after,10)==.125f);
 CHECK(channel(after,11)==10 && channel(after,12)==15 && channel(after,13)==-5);
 CHECK(channel(after,14)==-12.5f && channel(after,15)==.1f);
 CHECK(channel(after,16)==1 && channel(after,17)==1 && channel(after,18)==64);
 for(unsigned w=0;w<4;++w) {
  CHECK(channel(after,19+w)==.125f*(w+1));
  CHECK(channel(after,23+w)==-.0625f*(w+1));
  CHECK(channel(after,27+w)==(w%2 ? -16700 : 16700));
 }
 telemetry_lidar.mm[ROUTE_LIDAR_FRONT]=120;telemetry_lidar.mm[ROUTE_LIDAR_BEHIND]=60;telemetry_lidar.fresh=1;
 telemetry_lidar.mm[ROUTE_LIDAR_LEFT]=100;telemetry_lidar.mm[ROUTE_LIDAR_RIGHT]=2700;
 telemetry_received.sequence=1;telemetry_received.received_ms=200;telemetry_now=220;
 telemetry_nav.state=6;telemetry_nav.reason=10;telemetry_nav.target=5;telemetry_nav.confirmed=4;
 telemetry_nav.main_error_mm=-50;telemetry_nav.cross_error_mm=12;
 telemetry_nav.command_vx_mps=.125f;telemetry_nav.command_vy_mps=-.25f;
 telemetry_nav.reference_mask=13;telemetry_owned=1;
 CHECK(RouteDebug_BuildPacket(after,sizeof(after))==236);
 CHECK(channel(after,7)==60 && channel(after,6)==1 && channel(after,8)==120);
 CHECK(channel(after,43)==100 && channel(after,44)==2700 && channel(after,45)==20);
 CHECK(channel(after,46)==6 && channel(after,47)==10 && channel(after,48)==5 && channel(after,49)==4);
 CHECK(channel(after,50)==-50 && channel(after,51)==12);
 CHECK(channel(after,52)==.125f && channel(after,53)==-.25f);
 CHECK(channel(after,54)==13 && channel(after,55)==1);
 CHECK(channel(after,56)==220 && channel(after,57)==15);
 telemetry_lidar.mm[ROUTE_LIDAR_LEFT]=0;
 CHECK(RouteDebug_BuildPacket(after,sizeof(after))==236 && channel(after,57)==11);
 telemetry_now=600;telemetry_lidar.fresh=0;
 CHECK(RouteDebug_BuildPacket(after,sizeof(after))==236 && channel(after,45)==400 && channel(after,57)==0);
 CHECK(after[232]==0 && after[233]==0 && after[234]==0x80 && after[235]==0x7f);
 visual[2]=0x80;feed(&telemetry_camera,visual,6);
 CHECK(RouteDebug_BuildPacket(after,sizeof(after))==236);
 CHECK(channel(after,1)==-128 && channel(after,4)==2);
 controller.status.phase=ROUTE_FAULT;in.now_ms=1234;
 RouteDebug_Capture(&controller,&in,0,0);
 CHECK(RouteDebug_ReadFault(&frozen) && frozen.frozen);
 CHECK(RouteDebug_BuildPacket(before,sizeof(before))==236);
 visual[2]=0x7f;feed(&telemetry_camera,visual,6);
 CHECK(RouteDebug_BuildPacket(after,sizeof(after))==236);
 CHECK(channel(after,1)==127 && channel(after,4)==3);
 CHECK(memcmp(before,after,sizeof(before))!=0);
 CHECK(RouteDebug_ReadFault(&frozen) && frozen.tick_ms==1234);
 CHECK(RouteDebug_BuildPacket(after,sizeof(after)-1)==0);
 CHECK(RouteDebug_BuildPacket(0,sizeof(after))==0);
 if(!fails) printf("PASS: 58-channel merged navigation and wheels/IMU/live vision/rear range/alignment/physical job completion telemetry\n");
}
int main(void) {
 Camera_Receiver r={0}; unsigned i;
 unsigned char good[]={0x63,1,0xEC,1,3,0x66};
 unsigned char special[]={0x63,1,0x63,1,4,0x66,0x63,1,0x66,0,0,0x66};
 unsigned char bad[]={0x63,0,0,1,2,0x66};
 unsigned char empty[]={0x63,0,0,1,0,0x66};
 feed(&r,good,3); CHECK(r.data.frames==0); feed(&r,good+3,3);
 CHECK(r.data.frames==1 && r.data.offset==-20 && r.data.region==3);
 feed(&r,special,sizeof(special)); CHECK(r.data.frames==3 && r.data.offset==102);
 feed(&r,bad,6); CHECK(r.data.frames==3 && r.data.bad>=1);
 feed(&r,empty,6); CHECK(r.data.frames==4 && r.data.pole==0 && r.data.green==1 && r.data.region==0);
 unsigned char unknown[]={0x63,1,2,1,0,0x66};
 feed(&r,unknown,6); CHECK(r.data.frames==5 && r.data.pole==1 && r.data.green==1 && r.data.region==0);
 for(i=1;i<6;++i) { memset(&r,0,sizeof(r)); feed(&r,good,i); feed(&r,good,6); CHECK(r.data.frames==1 && r.data.offset==-20); }
 memset(&r,0,sizeof(r)); for(i=0;i<1000;++i) feed(&r,good,6); CHECK(r.data.frames==1000);
 good[2]=0x80; feed(&r,good,6); CHECK(r.data.offset==-128);
 good[2]=0x7f; feed(&r,good,6); CHECK(r.data.offset==127);
 if(!fails) printf("PASS: camera framing, signed data, fragmentation, resync, validation\n");
 test_telemetry();
 return fails;
}
