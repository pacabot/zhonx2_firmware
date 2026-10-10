#include "fw_hm18.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
/* Simulated HM-10 family module: answers only at its own speed, with or
 * without CR LF, applies the speed at AT+RESET. */
typedef struct {
    unsigned baud,pending_baud;int crlf_only,refuse_baud,present,noti;
    char name[16];unsigned resets,sets;
} module_t;
static char reply[48];static uint32_t reply_at;
static void answer(module_t *m,const char *cmd,unsigned baud,uint32_t now)
{
    reply[0]=0;
    if(!m->present || baud!=m->baud)return;
    char c[24];snprintf(c,sizeof c,"%s",cmd);
    size_t n=strlen(c);int crlf=n>=2 && !strcmp(c+n-2,"\r\n");
    if(crlf)c[n-2]=0;
    if(m->crlf_only && !crlf)return;
    if(!strcmp(c,"AT"))strcpy(reply,"OK");
    else if(!strcmp(c,"AT+BAUD4")) {
        if(m->refuse_baud)strcpy(reply,"ERROR");else {strcpy(reply,"OK+Set:4");m->pending_baud=115200;++m->sets;}
    }
    else if(!strcmp(c,"AT+RESET")) {strcpy(reply,"OK+RESET");++m->resets;if(m->pending_baud)m->baud=m->pending_baud;}
    else if(!strcmp(c,"AT+NAME?"))snprintf(reply,sizeof reply,"OK+NAME:%s",m->name);
    else if(!strncmp(c,"AT+NAME",7)) {snprintf(m->name,sizeof m->name,"%s",c+7);snprintf(reply,sizeof reply,"OK+Set:%s",m->name);++m->sets;}
    else if(!strcmp(c,"AT+NOTI?"))snprintf(reply,sizeof reply,"OK+Get:%d",m->noti);
    else if(!strcmp(c,"AT+NOTI1")) {m->noti=1;strcpy(reply,"OK+Set:1");++m->sets;}
    else if(!strcmp(c,"AT+ADDR?"))strcpy(reply,"OK+ADDR:A4C138123456");
    if(m->crlf_only && reply[0])strcat(reply,"\r\n");
    reply_at=now+15;
}
/* Runs the driver side for at most limit_ms; returns the time it took to be ready. */
static uint32_t run(fw_hm18_t *h,module_t *m,uint32_t limit_ms)
{
    uint32_t now=0;reply[0]=0;fw_hm18_start(h,now);
    for(;now<limit_ms;++now) {
        if(reply[0] && now>=reply_at) {for(const char *p=reply;*p;++p)fw_hm18_rx(h,(uint8_t)*p,now);reply[0]=0;}
        const char *send;
        if(fw_hm18_step(h,now,&send))answer(m,send,h->baud,now);
        if(fw_hm18_ready(h))return now;
    }
    return now;
}
int main(void)
{
    fw_hm18_t h;
    /* Factory module: 9600, default name, no notifications. */
    module_t factory={.baud=9600,.present=1,.name="HMSoft"};
    uint32_t t=run(&h,&factory,20000);
    assert(fw_hm18_ready(&h) && h.baud==115200 && factory.baud==115200);
    assert(!strcmp(factory.name,FW_HM18_NAME) && factory.noti==1 && factory.resets==2);
    assert(!strcmp(h.name,"HMSoft") && !strcmp(h.address,"A4C138123456"));
    printf("hm18: factory module configured in %u ms\n",(unsigned)t);
    /* Second boot: already configured, nothing written, ready within a second. */
    factory.resets=factory.sets=0;
    t=run(&h,&factory,20000);
    assert(fw_hm18_ready(&h) && t<1000 && !factory.resets && !factory.sets && h.baud==115200);
    /* A module that only understands CR LF commands. */
    module_t crlf={.baud=9600,.crlf_only=1,.present=1,.name="HMSoft"};
    run(&h,&crlf,30000);
    assert(fw_hm18_ready(&h) && h.baud==115200 && h.crlf && crlf.noti==1 && !strcmp(crlf.name,FW_HM18_NAME));
    /* A module that refuses the speed change keeps working at 9600. */
    module_t slow={.baud=9600,.refuse_baud=1,.present=1,.name=FW_HM18_NAME,.noti=1};
    run(&h,&slow,20000);
    assert(fw_hm18_ready(&h) && h.baud==9600 && !slow.resets);
    /* No module: absent after the whole search, searched again later. */
    module_t none={.baud=9600};
    run(&h,&none,9000);
    assert(h.state==FW_HM18_ABSENT && !fw_hm18_ready(&h));
    unsigned before=h.probes;none.present=1;
    for(uint32_t now=9000;now<40000 && !fw_hm18_ready(&h);++now) {
        if(reply[0] && now>=reply_at) {for(const char *p=reply;*p;++p)fw_hm18_rx(&h,(uint8_t)*p,now);reply[0]=0;}
        const char *send;if(fw_hm18_step(&h,now,&send))answer(&none,send,h.baud,now);
    }
    assert(fw_hm18_ready(&h) && h.probes>before);
    /* Connection notifications inside the data stream, never inside zeros. */
    fw_hm18_watch_t w;memset(&w,0,sizeof w);
    const char *stream="xxOK+CONNyyOK+LOST";int seen[2]={0,0};
    for(const char *p=stream;*p;++p) {int n=fw_hm18_watch(&w,(uint8_t)*p);if(n>0)++seen[0];if(n<0)++seen[1];}
    assert(seen[0]==1 && seen[1]==1);
    const uint8_t binary[]={'O','K','+',0,'C','O','N','N'};int any=0;
    for(unsigned i=0;i<sizeof binary;++i)any|=fw_hm18_watch(&w,binary[i]);
    assert(!any);
    puts("hm18: factory, configured, CR LF only, refused speed and absent modules; CONN/LOST notifications");
}
