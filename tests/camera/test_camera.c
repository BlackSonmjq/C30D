#include <stdio.h>
#include <string.h>
#include "camera_protocol.h"
static int fails;
#define CHECK(x) do { if (!(x)) { printf("FAIL line %u\n",__LINE__); ++fails; } } while (0)
static void feed(Camera_Receiver *r,const unsigned char *p,unsigned n) { while(n--) Camera_Push(r,*p++); }
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
 for(i=1;i<6;++i) { memset(&r,0,sizeof(r)); feed(&r,good,i); feed(&r,good,6); CHECK(r.data.frames==1 && r.data.offset==-20); }
 memset(&r,0,sizeof(r)); for(i=0;i<1000;++i) feed(&r,good,6); CHECK(r.data.frames==1000);
 good[2]=0x80; feed(&r,good,6); CHECK(r.data.offset==-128);
 good[2]=0x7f; feed(&r,good,6); CHECK(r.data.offset==127);
 if(!fails) printf("PASS: camera framing, signed data, fragmentation, resync, validation\n");
 return fails;
}
