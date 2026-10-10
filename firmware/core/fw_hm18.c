#include "fw_hm18.h"
#include <stdio.h>
#include <string.h>
static const unsigned bauds[]={115200,9600,57600,38400,19200};
#define BAUDS (sizeof bauds/sizeof *bauds)
static const char *const names[]={"probe","baud","reset","name?","name","noti?","noti","addr?","final","ready","absent"};
const char *fw_hm18_state_name(const fw_hm18_t *m) {return m->state<=FW_HM18_ABSENT?names[m->state]:"?";}
int fw_hm18_ready(const fw_hm18_t *m) {return m->state==FW_HM18_READY;}
static void go(fw_hm18_t *m,unsigned state) {m->state=(uint8_t)state;m->pending=0;}
static void pause_until(fw_hm18_t *m,uint32_t now,unsigned ms) {m->wait_until=now+ms;if(!m->wait_until)m->wait_until=1;}
void fw_hm18_start(fw_hm18_t *m,uint32_t now)
{
    memset(m,0,sizeof *m);
    m->baud=bauds[0];go(m,FW_HM18_PROBE);pause_until(m,now,50);
}
void fw_hm18_rx(fw_hm18_t *m,uint8_t byte,uint32_t now)
{
    if(!m->pending || !byte)return;
    if(m->reply_len+1u<sizeof m->reply) {m->reply[m->reply_len++]=(char)byte;m->reply[m->reply_len]=0;}
    m->last_rx_ms=now;
}
static int ok(const fw_hm18_t *m) {return strstr(m->reply,"OK")!=0;}
/* Text after the last ':' without the line ending, e.g. "OK+NAME:HMSoft". */
static void value(const fw_hm18_t *m,char *out,unsigned size)
{
    const char *colon=strrchr(m->reply,':');
    unsigned n=0;
    if(colon)for(const char *p=colon+1;*p && *p!='\r' && *p!='\n' && n+1<size;++p)out[n++]=*p;
    out[n]=0;
}
static void next_probe(fw_hm18_t *m,uint32_t now)
{
    if(!m->crlf) m->crlf=1;
    else {
        m->crlf=0;
        if(++m->baud_index>=BAUDS) {
            m->baud_index=0;m->baud=bauds[0];go(m,FW_HM18_ABSENT);pause_until(m,now,FW_HM18_RETRY_MS);return;
        }
    }
    m->baud=bauds[m->baud_index];go(m,FW_HM18_PROBE);pause_until(m,now,30);
}
static void handle(fw_hm18_t *m,int complete,uint32_t now)
{
    int good=complete && ok(m);
    switch(m->state) {
    case FW_HM18_PROBE:
        if(!good) {next_probe(m,now);return;}
        go(m,m->baud!=FW_HM18_BAUD && !m->baud_set_tried?FW_HM18_SET_BAUD:FW_HM18_NAME_GET);
        return;
    case FW_HM18_SET_BAUD:
        m->baud_set_tried=1;
        if(good) {m->after_baud=1;go(m,FW_HM18_RESET);}
        else go(m,FW_HM18_NAME_GET);
        return;
    case FW_HM18_RESET:
        /* The module restarts whatever it answered; it may not answer at all. */
        pause_until(m,now,FW_HM18_RESET_MS);
        if(m->after_baud) {
            /* Probe the new speed first; the usual search follows if it did not take. */
            m->after_baud=0;m->baud_index=0;m->baud=bauds[0];go(m,FW_HM18_PROBE);
        } else go(m,FW_HM18_FINAL);
        return;
    case FW_HM18_NAME_GET:
        if(good)value(m,m->name,sizeof m->name);
        go(m,good && strcmp(m->name,FW_HM18_NAME)?FW_HM18_NAME_SET:FW_HM18_NOTI_GET);
        return;
    case FW_HM18_NAME_SET:
        if(good)m->changed=1;
        go(m,FW_HM18_NOTI_GET);return;
    case FW_HM18_NOTI_GET: {
        char v[8];value(m,v,sizeof v);
        go(m,good && strcmp(v,"1")?FW_HM18_NOTI_SET:FW_HM18_ADDR_GET);
        return;
    }
    case FW_HM18_NOTI_SET:
        if(good)m->changed=1;
        go(m,FW_HM18_ADDR_GET);return;
    case FW_HM18_ADDR_GET:
        if(good)value(m,m->address,sizeof m->address);
        if(m->changed) {m->changed=0;go(m,FW_HM18_RESET);}
        else go(m,FW_HM18_READY);
        return;
    case FW_HM18_FINAL:
        if(good)go(m,FW_HM18_READY);
        else {m->baud_index=0;m->crlf=0;m->baud=bauds[0];go(m,FW_HM18_PROBE);}
        return;
    default:
        return;
    }
}
static const char *command(fw_hm18_t *m)
{
    const char *c;
    switch(m->state) {
    case FW_HM18_SET_BAUD: c="AT+BAUD4";break;
    case FW_HM18_RESET: c="AT+RESET";break;
    case FW_HM18_NAME_GET: c="AT+NAME?";break;
    case FW_HM18_NAME_SET: c="AT+NAME" FW_HM18_NAME;break;
    case FW_HM18_NOTI_GET: c="AT+NOTI?";break;
    case FW_HM18_NOTI_SET: c="AT+NOTI1";break;
    case FW_HM18_ADDR_GET: c="AT+ADDR?";break;
    default: c="AT";break;
    }
    snprintf(m->command,sizeof m->command,"%s%s",c,m->crlf?"\r\n":"");
    return m->command;
}
int fw_hm18_step(fw_hm18_t *m,uint32_t now,const char **send)
{
    if(m->state==FW_HM18_READY)return 0;
    if(m->wait_until) {
        if((int32_t)(now-m->wait_until)<0)return 0;
        m->wait_until=0;
        if(m->state==FW_HM18_ABSENT) {m->crlf=0;m->baud_index=0;m->baud=bauds[0];go(m,FW_HM18_PROBE);}
    }
    if(m->state==FW_HM18_ABSENT)return 0;
    if(!m->pending) {
        m->reply_len=0;m->reply[0]=0;m->pending=1;m->sent_ms=now;m->last_rx_ms=now;++m->probes;
        *send=command(m);return 1;
    }
    int complete=m->reply_len>0 && (uint32_t)(now-m->last_rx_ms)>=FW_HM18_SILENCE_MS;
    if(!complete && (uint32_t)(now-m->sent_ms)<FW_HM18_REPLY_MS)return 0;
    m->pending=0;handle(m,complete,now);
    return 0;
}
int fw_hm18_watch(fw_hm18_watch_t *w,uint8_t byte)
{
    /* The last seven bytes; a zero byte (binary data) never matches. */
    memmove(w->tail,w->tail+1,6);
    w->tail[6]=(char)byte;w->tail[7]=0;
    int notice=!strcmp(w->tail,"OK+CONN")?1:!strcmp(w->tail,"OK+LOST")?-1:0;
    if(notice)w->tail[6]=0;
    return notice;
}
