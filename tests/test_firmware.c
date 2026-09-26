#include "fw_start.h"
#include "fw_text.h"
#include "fw_store.h"
#include "fw_update.h"
#include "fw_protocol.h"
#include "fw_layout.h"
#include "nimes.h"
#include "wall_control.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned char memory[0x100000], backup[0x100000];
static int budget=-1, operations, failed;
static int mutate(void)
{
    if (failed) return -1;
    if (budget==0) { failed=1; return -1; }
    if (budget>0) --budget;
    ++operations; return 0;
}
static void power(int cut) { budget=cut; failed=0; operations=0; }
static size_t index_of(uint32_t addr, size_t n)
{
    assert(addr>=FW_BOOT_BASE && addr-FW_BOOT_BASE+n<=sizeof memory);
    return addr-FW_BOOT_BASE;
}
static int read_flash(void *ctx, uint32_t a, void *p, size_t n)
{ (void)ctx; memcpy(p,memory+index_of(a,n),n); return 0; }
static int erase_flash(void *ctx, uint32_t a, size_t n)
{
    (void)ctx;
    const uint32_t sectors[]={0x08004000,0x08008000,0x0800c000,0x08010000,
      0x08020000,0x08040000,0x08060000,0x08080000,0x080a0000,0x080c0000,0x080e0000,0x08100000};
    int first=-1,last=-1;
    for (unsigned i=0;i<sizeof sectors/sizeof *sectors;++i) {
        if (a==sectors[i]) first=(int)i;
        if (a+n==sectors[i]) last=(int)i;
    }
    assert(first>=0 && last>first);
    for (int i=first;i<last;++i) {
        size_t size=sectors[i+1]-sectors[i], off=index_of(sectors[i],size);
        if (mutate()) { memset(memory+off,0xff,size/2); return -1; } /* Torn erase */
        memset(memory+off,0xff,size);
    }
    return 0;
}
static int program_flash(void *ctx, uint32_t a, const void *p, size_t n)
{
    (void)ctx; assert(a>=FW_STORE_A && !(a&3) && !(n&3));
    size_t off=index_of(a,n); const unsigned char *src=p;
    for (size_t i=0;i<n;i+=4) {
        if (mutate()) { /* Some bits of the failing word may already be programmed. */
            memory[off+i] &= src[i]; return -1;
        }
        for (unsigned j=0;j<4;++j) {
            assert((memory[off+i+j] & src[i+j])==src[i+j]);
            memory[off+i+j] &= src[i+j];
        }
    }
    return 0;
}
static const fw_flash_t flash={0,read_flash,erase_flash,program_flash};
static void store_test(void)
{
    uint32_t a[128],b[128],out[128];
    for (unsigned i=0;i<128;++i) { a[i]=i; b[i]=i*7+5; }
    memset(memory,0xff,sizeof memory); power(-1);
    assert(fw_store_load(&flash,1,out,sizeof out));
    assert(!fw_store_save(&flash,1,a,sizeof a));
    memcpy(backup,memory,sizeof memory);
    power(-1); assert(!fw_store_save(&flash,1,b,sizeof b)); int writes=operations;
    for (int cut=0;cut<=writes;++cut) {
        memcpy(memory,backup,sizeof memory); power(cut);
        (void)fw_store_save(&flash,1,b,sizeof b); power(-1);
        assert(!fw_store_load(&flash,1,out,sizeof out));
        assert(!memcmp(a,out,sizeof a) || !memcmp(b,out,sizeof b));
    }
    power(-1); assert(!fw_store_save(&flash,1,b,sizeof b)); assert(operations==0);
    memory[index_of(FW_STORE_B+40,1)]^=1;
    assert(!fw_store_load(&flash,1,out,sizeof out)); assert(!memcmp(out,a,sizeof a));
    assert(fw_store_load(&flash,2,out,sizeof out));
    assert(fw_store_save(&flash,1,b,sizeof b-1));
    printf("store: %d torn-write/erase cut points, CRC fallback, unchanged save\n",writes+1);
}
static unsigned char image_data[4096];
static void stage(fw_update_t *u)
{
    assert(!fw_update_begin(&flash,u,sizeof image_data,fw_crc32(image_data,sizeof image_data)));
    for (unsigned off=0;off<sizeof image_data;off+=1024)
        assert(!fw_update_write(&flash,u,off,image_data+off,1024));
}
static void update_test(void)
{
    for (unsigned i=0;i<sizeof image_data;++i) image_data[i]=(unsigned char)(i*13);
    uint32_t vectors[]={FW_SRAM_END,FW_APP_BASE+101}; memcpy(image_data,vectors,8);
    memset(memory,0xff,sizeof memory); power(-1); fw_update_t u;
    assert(fw_application_valid(&flash));
    assert(fw_update_begin(&flash,&u,FW_IMAGE_MAX+4,0));
    assert(fw_update_begin(&flash,&u,7,0));
    stage(&u);
    assert(!fw_update_write(&flash,&u,0,image_data,1024)); /* Idempotent retry */
    assert(fw_update_write(&flash,&u,2,image_data,4));
    assert(fw_update_write(&flash,&u,sizeof image_data,image_data,4));
    assert(!fw_update_seal(&flash,&u));
    memcpy(backup,memory,sizeof memory);
    power(-1); assert(!fw_update_install(&flash)); int writes=operations;
    for (int cut=0;cut<=writes;++cut) {
        memcpy(memory,backup,sizeof memory); power(cut);
        (void)fw_update_install(&flash); power(-1); /* New boot */
        assert(!fw_update_install(&flash)); assert(!fw_application_valid(&flash));
        assert(!memcmp(memory+index_of(FW_APP_BASE,sizeof image_data),image_data,sizeof image_data));
        assert(!memcmp(memory+index_of(FW_STAGE_BASE,sizeof image_data),image_data,sizeof image_data));
    }
    /* A corrupt upload cannot invalidate the installed application. */
    stage(&u); memory[index_of(FW_STAGE_BASE+20,1)]^=1;
    assert(fw_update_seal(&flash,&u)); assert(!fw_application_valid(&flash));
    stage(&u); memcpy(backup,memory,sizeof memory);
    power(-1); assert(!fw_update_seal(&flash,&u)); int seals=operations;
    for (int cut=0;cut<=seals;++cut) {
        memcpy(memory,backup,sizeof memory); power(cut); u.active=1;
        (void)fw_update_seal(&flash,&u); power(-1);
        /* Before installation starts, the old application's bytes are untouched. */
        assert(!memcmp(memory+index_of(FW_APP_BASE,sizeof image_data),image_data,sizeof image_data));
        int result=fw_update_install(&flash);
        if (!result) assert(!fw_application_valid(&flash));
        else assert(fw_application_valid(&flash)); /* Remain in recovery service. */
    }
    printf("update: %d install and %d manifest power cuts; bad CRC rejected\n",writes+1,seals+1);
}
static void protocol_test(void)
{
    fw_packet_t p={0}; fw_reply_t r; fw_update_t u={0};
    p.word[0]=FW_PACKET_MAGIC; p.word[1]=FW_BEGIN; p.word[2]=99;
    p.word[3]=sizeof image_data; p.word[4]=fw_crc32(image_data,sizeof image_data); p.word[6]=1;
    power(-1); fw_protocol(&flash,&u,&p,&r); assert(r.word[2] && !operations);
    p.word[7]=fw_crc32(&p,28);
    fw_protocol(&flash,&u,&p,&r); assert(!r.word[2] && r.word[1]==99 && operations==1);
    assert(r.word[7]==fw_crc32(&r,28));
    p.word[1]=FW_DATA; p.word[3]=4; p.word[5]=1024; memcpy(p.payload,image_data,1024);
    p.word[7]=fw_crc32_more(fw_crc32(&p,28),p.payload,1024);
    power(-1); fw_protocol(&flash,&u,&p,&r); assert(r.word[2] && !operations);
    p.word[5]=1025; fw_protocol(&flash,&u,&p,&r); assert(r.word[2] && !operations);
    puts("protocol: malformed CRC, length and out-of-order requests rejected");
}
static unsigned rng=12345;
static unsigned random32(void) { rng^=rng<<13; rng^=rng>>17; rng^=rng<<5; return rng; }
static int bfs(const nm_map_t *m,int start,int goal)
{
    int queue[NM_CELLS],distance[NM_CELLS],rd=0,wr=0;
    for(int i=0;i<NM_CELLS;++i) distance[i]=-1;
    queue[wr++]=start; distance[start]=0;
    while(rd<wr) {
        int c=queue[rd++];
        for(unsigned d=0;d<4;++d) {
            int n=nm_neighbour(c,d);
            if(n>=0 && distance[n]<0 && (m->cell[c].known & 1u<<d) && !(m->cell[c].walls & 1u<<d)) {
                distance[n]=distance[c]+1; queue[wr++]=n;
            }
        }
    }
    return distance[goal];
}
static void route_check(const nm_map_t *m,nm_pose_t p,const uint8_t *goals,const nm_route_t *r)
{
    int c=p.y*NM_SIDE+p.x; uint32_t cost=0; unsigned h=p.heading;
    for(unsigned i=0;i<r->length;++i) {
        unsigned d=r->direction[i],turn=(d+4-h)%4;
        if(turn==3) turn=1;
        assert(m->cell[c].known & 1u<<d); assert(!(m->cell[c].walls & 1u<<d));
        c=nm_neighbour(c,d); assert(c>=0); cost+=1024+turn; h=d;
    }
    assert(goals[c] && cost==r->cost);
}
static void maze_test(void)
{
    nm_map_t m; nm_init(&m); assert(nm_valid(&m));
    assert(nm_edge(&m,0,NM_SOUTH,0));
    assert(!nm_edge(&m,0,NM_NORTH,0)); assert(nm_edge(&m,9,NM_SOUTH,1));
    nm_map_t copy=m;
    assert(nm_observe(&m,(nm_pose_t){0,0,NM_NORTH},1)); assert(!memcmp(&copy,&m,sizeof m));
    uint8_t goals[NM_CELLS]; assert(!nm_goal(&m,goals));
    /* Noncentral goal room at x=6,y=1. */
    int c=15;
    assert(!nm_edge(&m,c,NM_NORTH,0)); assert(!nm_edge(&m,c,NM_EAST,0));
    assert(!nm_edge(&m,c+1,NM_NORTH,0)); assert(!nm_edge(&m,c+9,NM_EAST,0));
    assert(nm_goal(&m,goals)==1 && goals[15] && goals[25] && !goals[40]);
    nm_route_t route;
    assert(nm_route(&m,(nm_pose_t){0,0,0},goals,0,&route));
    assert(!nm_route(&m,(nm_pose_t){0,0,0},goals,1,&route));
    for(int trial=0;trial<500;++trial) {
        nm_init(&m);
        for(int cell=0;cell<NM_CELLS;++cell) for(unsigned d=0;d<2;++d)
            if(nm_neighbour(cell,d)>=0) assert(!nm_edge(&m,cell,d,(random32()%100)<35));
        assert(nm_valid(&m)); memset(goals,0,sizeof goals);
        int start=(int)(random32()%NM_CELLS),goal=(int)(random32()%NM_CELLS); goals[goal]=1;
        nm_pose_t p={start%9,start/9,random32()%4};
        int shortest=bfs(&m,start,goal),result=nm_route(&m,p,goals,0,&route);
        assert((shortest<0)==(result<0));
        if(!result) { assert(route.length==shortest); route_check(&m,p,goals,&route); }
    }
    nm_init(&m);
    for(int cell=0;cell<NM_CELLS;++cell) for(unsigned d=0;d<2;++d)
        if(nm_neighbour(cell,d)>=0) assert(!nm_edge(&m,cell,d,0));
    memset(goals,0,sizeof goals); goals[80]=1;
    assert(!nm_route(&m,(nm_pose_t){0,0,NM_NORTH},goals,0,&route));
    assert(route.length==16 && route.cost==16*1024+1); /* Turn tie-break. */
    assert(nm_refine(&m,(nm_pose_t){4,4,0},(nm_pose_t){0,0,0},goals,&route)==1);
    puts("maze: 500 random maps vs BFS, reciprocal walls, noncentral 2x2 goal, turn tie-break");
}
static void exploration_test(void)
{
    int completed=0;
    for(int attempt=0;completed<40 && attempt<500;++attempt) {
        uint8_t opened[NM_CELLS][4]={{0}},visited[NM_CELLS]={0},goals[NM_CELLS];
        int stack[NM_CELLS],depth=0;
        const int corners[]={0,8,80,72}; int origin=corners[completed%4];
        stack[depth++]=origin; visited[origin]=1;
        while(depth) {
            int c=stack[depth-1], options[4],count=0;
            for(unsigned d=0;d<4;++d) {
                int n=nm_neighbour(c,d);
                if(n>=0 && !visited[n]) options[count++]=(int)d;
            }
            if(!count) { --depth; continue; }
            unsigned d=(unsigned)options[random32()%(unsigned)count]; int n=nm_neighbour(c,d);
            opened[c][d]=opened[n][(d+2)%4]=1; visited[n]=1; stack[depth++]=n;
        }
        int room=1+(int)(random32()%6)+(1+(int)(random32()%6))*NM_SIDE;
        const int cells[]={room,room,room+1,room+NM_SIDE};
        const unsigned dirs[]={NM_EAST,NM_NORTH,NM_NORTH,NM_EAST};
        for(unsigned i=0;i<4;++i) {
            int n=nm_neighbour(cells[i],dirs[i]);
            opened[cells[i]][dirs[i]]=opened[n][(dirs[i]+2)%4]=1;
        }
        nm_map_t truth,known; nm_init(&truth); nm_init(&known);
        for(int c=0;c<NM_CELLS;++c) for(unsigned d=0;d<2;++d)
            if(nm_neighbour(c,d)>=0) assert(!nm_edge(&truth,c,d,!opened[c][d]));
        if(nm_goal(&truth,goals)!=1) continue;
        unsigned degree=0; for(unsigned d=0;d<4;++d) degree+=opened[origin][d];
        if(degree!=1) continue;
        nm_pose_t start={origin%9,origin/9,0},pose=start;
        int certified=0;
        for(int step=0;step<1000;++step) {
            int c=pose.y*NM_SIDE+pose.x;
            unsigned headings[]={pose.heading,(pose.heading+3)%4,(pose.heading+1)%4};
            uint8_t walls=0;
            for(unsigned i=0;i<3;++i) walls |= !!(truth.cell[c].walls & 1u<<headings[i])<<i;
            assert(!nm_observe(&known,pose,walls)); assert(nm_valid(&known));
            nm_route_t route;
            if(nm_goal(&known,goals)==1) {
                int r=nm_refine(&known,pose,start,goals,&route); assert(r>=0);
                if(r==1) { certified=1; break; }
            } else assert(!nm_frontier(&known,pose,&route));
            if(!route.length) {
                unsigned d; for(d=0;d<4;++d) if(!(known.cell[c].known & 1u<<d)) break;
                assert(d<4); pose.heading=d; continue;
            }
            unsigned d=route.direction[0]; assert(!(truth.cell[c].walls & 1u<<d));
            int n=nm_neighbour(c,d); assert(n>=0);
            pose=(nm_pose_t){n%9,n/9,d};
        }
        assert(certified);
        nm_route_t actual,reference;
        assert(!nm_route(&known,start,goals,0,&actual));
        assert(!nm_route(&truth,start,goals,0,&reference));
        assert(actual.cost==reference.cost);
        ++completed;
    }
    assert(completed==40);
    puts("exploration: 40 generated mazes, all four start corners, goal found and optimal route proven");
}
static void wall_test(void)
{
    wall_control_t a,b; wall_control_reset(&a); wall_control_reset(&b);
    int old=0;
    for(int i=0;i<200;++i) {
        uint8_t s=i%2 ? 0x0f : 0x3c; /* Left near / right near symmetric. */
        int x=wall_control_step(&a,s),y=wall_control_step(&b,i%2 ? 0x3c : 0x0f);
        assert(abs(x)<=120 && abs(x-old)<=8 && x==-y); old=x;
    }
    for(int i=0;i<100;++i) wall_control_step(&a,0x0f);
    assert(a.output>0);
    for(int i=0;i<100;++i) wall_control_step(&a,0x3f);
    assert(a.output==0);
    puts("control: chatter slew limit, symmetry, doorway decay");
}
static void interaction_test(void)
{
    fw_start_gate_t g={0};
    for(unsigned t=0;t<1000;t+=10) assert(!fw_start_gate(&g,t,0,1));
    assert(g.state==FW_WAIT_HAND); /* Never start without a hand. */
    for(unsigned t=1000;t<=1200;t+=10) assert(!fw_start_gate(&g,t,1,1));
    assert(g.state==FW_WAIT_RELEASE);
    for(unsigned t=1210;t<1310;t+=10) assert(!fw_start_gate(&g,t,0,1));
    assert(fw_start_gate(&g,1310,0,1));
    g=(fw_start_gate_t){0};
    fw_start_gate(&g,UINT32_MAX-100,1,1);
    fw_start_gate(&g,99,1,1); assert(g.state==FW_WAIT_RELEASE);
    fw_start_gate(&g,110,0,1); fw_start_gate(&g,220,0,0);
    assert(!fw_start_gate(&g,230,0,1)); assert(fw_start_gate(&g,330,0,1));
    char line[20]; const char *p="REGLAGES ET CARTE CHARGES";
    p=fw_text_line(p,line,19); assert(!strcmp(line,"REGLAGES ET CARTE"));
    p=fw_text_line(p,line,19); assert(!strcmp(line,"CHARGES") && !*p);
    p=fw_text_line("ABCDEFGHIJKLMNOPQRSTUV",line,19);
    assert(strlen(line)==19 && !strcmp(p,"TUV"));
    p=fw_text_line("A\nB",line,19); assert(!strcmp(line,"A") && !strcmp(p,"B"));
    puts("interaction: hand present/released, stale scans, timer wrap and OLED text wrapping");
}
int main(void)
{
    assert(fw_crc32("123456789",9)==0xcbf43926);
    assert(fw_crc32_more(fw_crc32("1234",4),"56789",5)==0xcbf43926);
    interaction_test(); store_test(); update_test(); protocol_test(); maze_test(); exploration_test(); wall_test();
    puts("All firmware core tests passed."); return 0;
}
