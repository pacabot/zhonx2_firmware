/* Ariane Link on ZHONX II, see fw_link.h and docs/ARIANE.md.
 * Port of ZHONX III's link.c: the pure modules (frames, rings, light
 * sequencer, parameters, console line) are shared verbatim in firmware/link/;
 * this file holds what depends on the robot: the PendSV scheduling, the HM-18
 * module, the commands mapped onto fw_motion / fw_app, the telemetry sources
 * (step odometry, binary IR sensors) and the light signal (LEDs and OLED). */
#include "fw_link.h"
#include "fw_app.h"
#include "fw_ariane.h"
#include "fw_battery.h"
#include "fw_bt.h"
#include "fw_buttons.h"
#include "fw_display.h"
#include "fw_layout.h"
#include "fw_motion.h"
#include "fw_sound.h"
#include "fw_start.h"
#include "middleware/link/ariane_link_schema.h"
#include "middleware/link/link_frame.h"
#include "middleware/link/link_led.h"
#include "middleware/link/link_line.h"
#include "middleware/link/link_params.h"
#include "middleware/link/link_port.h"
#include "middleware/link/link_ring.h"
#include "stm32f4xx.h"
#include "config/config.h"
#include "app/app_def.h"
#include "hal/hal_led.h"
#include "hal/hal_os.h"
#include "hal/hal_sensor.h"
#include "hal/hal_step_motor.h"
#include "oled/ssd1306.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Telemetry ring: 1024 records (38 912 bytes) in the CCM, 51 s at the default
 * 20 Hz. BLE carries a few kB/s at best: 100 Hz would already need 4.4 kB/s. */
#define RING_RECORDS 1024u
#define EVENT_ENTRIES 64u
#define REPLY_FIFO_SIZE 2048u
#define LOG_FIFO_SIZE 1024u
#define LOG_TEXT_MAX 200u
#define DUMP_CHUNK 128u
#define TELEMETRY_RATE_DEFAULT 20
#define TELEMETRY_RATE_MAX 100
#define SESSION_TIMEOUT_MS 60000u
#define DROPPED_REPORT_MS 1000u
#define BATCH_WAIT_MS 100u
#define REBOOT_DELAY_MS 300u
#define FW_NAME "Zhonx2 ariane"
#define FRAME_MAX (LINK_FRAME_OVERHEAD + LINK_FRAME_MAX_PAYLOAD)
#define EVENT_FRAME_MAX (LINK_FRAME_OVERHEAD + LINK_EVENT_HEADER_SIZE + LINK_EVENT_ARGS_MAX)
#define BATCH_FRAME_MAX (LINK_FRAME_OVERHEAD + LINK_TELEMETRY_BATCH_SIZE + LINK_BATCH_RECORDS * LINK_TELEMETRY_RECORD_SIZE)
#define DUMP_FRAME_MAX (LINK_FRAME_OVERHEAD + LINK_DUMP_HEADER_SIZE + DUMP_CHUNK)
#define TICKS_PER_MM (2.0 * (double)STEPS_PER_MM)

enum { JOB_IDLE, JOB_PENDING, JOB_RUNNING };
int fw_link_run1_speed;
typedef struct { uint8_t cmd, job, flags, len; uint8_t args[LINK_CMD_MOVE_SIZE]; } job_t;

static link_telemetry_record_t telemetry_slots[RING_RECORDS] __attribute__((section(".ccm_noinit")));
static link_event_t event_slots[EVENT_ENTRIES];
static link_tring_t tring;
static link_ering_t ering;
static uint8_t reply_data[REPLY_FIFO_SIZE], log_data[LOG_FIFO_SIZE];
static link_fifo_t reply_fifo, log_fifo;  /* frames as [len lo][len hi][frame], SEQ patched when sent */
static uint8_t tx_frame[2 + FRAME_MAX];
static uint8_t scratch[2 + LINK_FRAME_OVERHEAD + LINK_LOG_HEADER_SIZE + LOG_TEXT_MAX];
static link_parser_t parser;
static char console_text[96];
static link_line_t console_line;
static fw_hm18_watch_t watch;
static link_led_t led;
static link_params_t params;
extern const link_param_t fw_link_param_table[];
extern const unsigned fw_link_param_table_count;

static struct {
    uint8_t initialised, log_enabled, tx_seq;
    volatile uint8_t session, rate_hz, menu_idle, led_foreground;
    volatile uint16_t sample_period;
    uint32_t last_rx_ms, last_dropped_ms, last_batch_ms, reboot_at, build_id;
    int last_fault;
    fw_link_status_t status;
} link;
static struct {
    volatile uint8_t state;
    job_t pending, current;
    uint8_t led_cmd_active, led_cmd_cmd, led_cmd_job;
} job;
static struct { uint8_t active; uint32_t addr, left; } dump;
static struct { uint8_t active, kind; } move;
/* WALL_FOLLOW flag: set from the first move on, as ZHONX III keeps its follow type. */
static volatile uint8_t moved;
/* Telemetry producer, SysTick side; the pose comes from the foreground. */
static struct {
    uint16_t tick, battery_tick;
    int32_t last_um[2];
    double last_yaw;
    volatile int8_t cell_x, cell_y;
    volatile uint8_t heading, motion_override;
    volatile uint32_t track_um;
} telem;

/* Entry edges crossed recently, in map coordinates (foreground only). */
#define ENTRIES 8
static struct {uint8_t x,y,valid;uint32_t t_us;} entries[ENTRIES];
static unsigned entry_next;
/* Last cell given a CELL: one CELL per arrival, never again after a turn in place. */
static struct {uint8_t x,y,valid;} last_cell;

/* ---- clocks ---------------------------------------------------------------------- */
uint32_t link_time_ms(void) {return hal_os_get_systicks();}
uint32_t link_time_us(void)
{
    uint32_t ms,value,pending;
    do {
        ms=hal_os_get_systicks();value=SysTick->VAL;
        pending=SCB->ICSR&SCB_ICSR_PENDSTSET_Msk;
    } while(ms!=hal_os_get_systicks());
    uint32_t load=SysTick->LOAD+1;
    /* Above SysTick priority the counter may have wrapped before its handler ran. */
    if(pending && value>load/2)++ms;
    return ms*1000u+(load-1-value)*1000u/load;
}
/* CRC32 (IEEE, as zlib) of the application image: what application.ota.bin holds. */
static uint32_t image_crc32(void)
{
    static const uint32_t table[16]={
        0x00000000u,0x1DB71064u,0x3B6E20C8u,0x26D930ACu,0x76DC4190u,0x6B6B51F4u,0x4DB26158u,0x5005713Cu,
        0xEDB88320u,0xF00F9344u,0xD6D6A3E8u,0xCB61B38Cu,0x9B64C2B0u,0x86D3D2D4u,0xA00AE278u,0xBDBDF21Cu};
    extern uint32_t _sidata,_sdata,_edata;
    const uint8_t *p=(const uint8_t *)FW_APP_BASE;
    const uint8_t *end=(const uint8_t *)((uint32_t)&_sidata+((uint32_t)&_edata-(uint32_t)&_sdata));
    uint32_t crc=0xFFFFFFFFu;
    while(p<end) {crc^=*p++;crc=table[crc&15]^(crc>>4);crc=table[crc&15]^(crc>>4);}
    return ~crc;
}

/* ---- frames out (PendSV only) ------------------------------------------------------ */
static void send_frame(uint8_t *frame,size_t length)
{
    frame[2]=link.tx_seq++;
    uint16_t crc=link_crc16(frame+1,length-3,0xFFFF);
    frame[length-2]=(uint8_t)crc;frame[length-1]=(uint8_t)(crc>>8);
    fw_bt_send(frame,(unsigned)length);
}
static int queue_frame(link_fifo_t *fifo,uint8_t type,const void *payload,uint16_t length)
{
    size_t n=link_frame_encode(scratch+2,sizeof scratch-2,type,0,payload,length);
    if(!n)return 0;
    scratch[0]=(uint8_t)n;scratch[1]=(uint8_t)(n>>8);
    return link_fifo_put(fifo,scratch,(unsigned)n+2);
}
static void reply(uint8_t type,const void *payload,uint16_t length)
{
    if(!queue_frame(&reply_fifo,type,payload,length))++link.status.replies_dropped;
}
static void cmd_ack(uint8_t cmd,uint8_t id,uint8_t status,uint8_t detail)
{
    link_cmd_ack_t ack={cmd,id,status,detail};
    reply(LINK_FRAME_TYPE_CMD_ACK,&ack,LINK_CMD_ACK_SIZE);
}
static void push_event(uint8_t event,const void *args,uint8_t length)
{
    if(link.initialised)link_ering_push(&ering,event,link_time_us(),args,length);
}
static void pump_fifo(link_fifo_t *fifo)
{
    for(;;) {
        uint8_t header[2];
        if(link_fifo_peek(fifo,header,2)<2)return;
        unsigned length=header[0]|(unsigned)header[1]<<8;
        if(fw_bt_tx_free()<length)return;
        link_fifo_peek(fifo,tx_frame,length+2);link_fifo_drop(fifo,length+2);
        send_frame(tx_frame+2,length);
    }
}
static void pump_dump(void)
{
    if(!dump.active || fw_bt_tx_free()<DUMP_FRAME_MAX)return;
    uint8_t payload[LINK_DUMP_HEADER_SIZE+DUMP_CHUNK];
    uint32_t n=dump.left<DUMP_CHUNK?dump.left:DUMP_CHUNK;
    link_dump_header_t header={dump.addr,(uint16_t)n};
    memcpy(payload,&header,LINK_DUMP_HEADER_SIZE);
    memcpy(payload+LINK_DUMP_HEADER_SIZE,(const void *)dump.addr,n);
    size_t length=link_frame_encode(tx_frame,sizeof tx_frame,LINK_FRAME_TYPE_DUMP_DATA,0,payload,(uint16_t)(LINK_DUMP_HEADER_SIZE+n));
    send_frame(tx_frame,length);
    dump.addr+=n;dump.left-=n;
    if(!dump.left) {link_ev_dump_done_t done={0};dump.active=0;push_event(LINK_EVENT_DUMP_DONE,&done,LINK_EV_DUMP_DONE_SIZE);}
}
static void report_dropped(uint32_t now)
{
    if(!link.session || now-link.last_dropped_ms<DROPPED_REPORT_MS)return;
    uint32_t state=link_lock();
    link_ev_dropped_t dropped={(uint16_t)tring.dropped_unreported,(uint8_t)ering.dropped_unreported};
    tring.dropped_unreported=ering.dropped_unreported=0;
    link_unlock(state);
    if(!dropped.records && !dropped.events)return;
    link.last_dropped_ms=now;
    push_event(LINK_EVENT_DROPPED,&dropped,LINK_EV_DROPPED_SIZE);
}
static void pump_tx(uint32_t now)
{
    pump_fifo(&reply_fifo);
    if(!link.session)return;
    while(link_ering_pending(&ering) && fw_bt_tx_free()>=EVENT_FRAME_MAX) {
        size_t n=link_build_event(&ering,tx_frame,sizeof tx_frame,0);
        if(!n)break;
        send_frame(tx_frame,n);
    }
    /* Ten records per frame halve the framing overhead: wait for a full batch
     * at most BATCH_WAIT_MS. */
    unsigned pending=link_tring_pending(&tring);
    if(pending && (pending>=LINK_BATCH_RECORDS || now-link.last_batch_ms>=BATCH_WAIT_MS) &&
       fw_bt_tx_free()>=BATCH_FRAME_MAX) {
        size_t n=link_build_telemetry(&tring,tx_frame,sizeof tx_frame,0);
        if(n) {send_frame(tx_frame,n);link.last_batch_ms=now;}
    }
    pump_dump();
    pump_fifo(&log_fifo);
}

/* ---- logs and console ------------------------------------------------------------- */
static void log_line(const char *text,size_t length)
{
    uint8_t payload[LINK_LOG_HEADER_SIZE+LOG_TEXT_MAX];
    while(length) {
        size_t n=length<LOG_TEXT_MAX?length:LOG_TEXT_MAX;
        link_log_header_t header={link_time_us(),LINK_LOG_LEVEL_DEBUG};
        memcpy(payload,&header,LINK_LOG_HEADER_SIZE);memcpy(payload+LINK_LOG_HEADER_SIZE,text,n);
        if(!queue_frame(&log_fifo,LINK_FRAME_TYPE_LOG,payload,(uint16_t)(LINK_LOG_HEADER_SIZE+n)))++link.status.logs_dropped;
        text+=n;length-=n;
    }
}
/* One console answer line: a LOG frame in a session, raw text to a terminal otherwise. */
static void say(const char *text)
{
    if(link.session) {if(link.log_enabled)log_line(text,strlen(text));}
    else {fw_bt_send(text,(unsigned)strlen(text));fw_bt_send("\r\n",2);}
}
static void console_run(const char *line)
{
    char s[128];
    if(!strcmp(line,"help")) {say("help status bt version beep");return;}
    if(!strcmp(line,"version")) {
        snprintf(s,sizeof s,"%s build %08lX schema %08lX",FW_NAME,(unsigned long)link.build_id,(unsigned long)LINK_SCHEMA_CRC);
        say(s);return;
    }
    if(!strcmp(line,"status")) {
        /* Same lines as ZHONX III's status command (cmd_diag.c). */
        fw_bt_status_t b;fw_bt_status(&b);
        snprintf(s,sizeof s,"Uptime: %lu ms, Bluetooth: %s",(unsigned long)hal_os_get_systicks(),b.ready?"present":"absent");say(s);
        snprintf(s,sizeof s,"Ariane Link: %s, build %08lX, frames %lu, CRC errors %lu, overruns %lu, restarts %lu",
                 link.session?"session":"idle",(unsigned long)link.build_id,(unsigned long)link.status.frames_rx,
                 (unsigned long)parser.crc_errors,(unsigned long)b.rx_errors,0ul);say(s);
        snprintf(s,sizeof s,"Ariane Link dropped: telemetry %lu, events %lu, logs %lu, replies %lu",
                 (unsigned long)tring.dropped_total,(unsigned long)ering.dropped_total,
                 (unsigned long)link.status.logs_dropped,(unsigned long)link.status.replies_dropped);say(s);
        snprintf(s,sizeof s,"Bluetooth module recoveries: 0, transmit chunks abandoned: %lu, bytes dropped: %lu",
                 (unsigned long)b.tx_stalls,(unsigned long)b.tx_dropped);say(s);
        return;
    }
    if(!strcmp(line,"bt")) {
        fw_bt_status_t b;fw_bt_status(&b);
        snprintf(s,sizeof s,"HM-18 %s %u bd name %s addr %s budget %u B/s rx %lu tx %lu",
                 b.state,b.baud,b.name,b.address[0]?b.address:"?",b.tx_rate,(unsigned long)b.rx_bytes,(unsigned long)b.tx_bytes);
        say(s);return;
    }
    if(!strcmp(line,"beep")) {fw_sound_effect(FW_SOUND_VICTORY);say("beep");return;}
    if(line[0]) {snprintf(s,sizeof s,"Unknown command: %.40s (help)",line);say(s);}
}
static void console_byte(uint8_t byte)
{
    /* Terminal applications end lines with LF alone, CR alone or CR LF. */
    if(byte==LINK_LINE_LF && console_line.length)byte=LINK_LINE_CR;
    size_t n=link_line_feed(&console_line,byte);
    if(!n)return;
    console_text[n-1]=0;
    console_run(console_text);
}

/* ---- connection -------------------------------------------------------------------- */
static void session_end(void) {link.session=0;}
static void connection(int up)
{
    if(up==fw_bt_connected())return;
    fw_bt_set_connected(up);
    link_line_init(&console_line,console_text,sizeof console_text);
    if(!up) {
        session_end();
        link_fifo_init(&reply_fifo,reply_data,REPLY_FIFO_SIZE);
        link_fifo_init(&log_fifo,log_data,LOG_FIFO_SIZE);
        dump.active=0;
    } else link.last_rx_ms=hal_os_get_systicks();
    fw_sound_effect(up?FW_SOUND_CONNECT:FW_SOUND_DISCONNECT);
}

/* ---- commands --------------------------------------------------------------------- */
static int busy(void) {return job.state!=JOB_IDLE || !link.menu_idle || dump.active;}
static void hello(void)
{
    link_hello_ack_t ack;memset(&ack,0,sizeof ack);
    link.session=1;link.log_enabled=1;
    fw_bt_quiet(0);
    if(!link.rate_hz)fw_link_set_telemetry_rate(TELEMETRY_RATE_DEFAULT);
    /* Whatever the previous session left unacknowledged goes again. */
    link_tring_flush(&tring);link_ering_flush(&ering);
    const fw_rotation_data_t *r=fw_app_rotation();
    uint32_t quarter=fw_rotation_valid(r)?fw_rotation_quarter(r,120,0):0;
    telem.track_um=quarter?(uint32_t)lround(quarter*4.0/M_PI):(uint32_t)(WHEELS_DISTANCE*1000.0);
    ack.proto_version=LINK_PROTO_VERSION;ack.robot=LINK_ROBOT_ZHONX2;ack.telemetry_hz=link.rate_hz;
    ack.schema_crc=LINK_SCHEMA_CRC;ack.build_id=link.build_id;
    strncpy(ack.fw_name,FW_NAME,sizeof ack.fw_name-1);
    ack.ring_records=RING_RECORDS;ack.max_payload=LINK_MAX_PAYLOAD;
    reply(LINK_FRAME_TYPE_HELLO_ACK,&ack,LINK_HELLO_ACK_SIZE);
}
static int dump_range_ok(uint32_t addr,uint32_t length)
{
    static const struct {uint32_t base,size;} regions[]={{0x20000000u,128u*1024u},{0x10000000u,64u*1024u},{0x08000000u,1024u*1024u}};
    if(!length)return 0;
    for(unsigned i=0;i<sizeof regions/sizeof *regions;++i)
        if(addr>=regions[i].base && addr-regions[i].base<regions[i].size && length<=regions[i].size-(addr-regions[i].base))return 1;
    return 0;
}
static void queue_job(const link_cmd_header_t *header,const uint8_t *args,unsigned length,uint8_t flags)
{
    job.pending.cmd=header->cmd;job.pending.job=header->job;job.pending.flags=flags;
    job.pending.len=(uint8_t)(length<sizeof job.pending.args?length:sizeof job.pending.args);
    memcpy(job.pending.args,args,job.pending.len);
    job.state=JOB_PENDING;
    cmd_ack(header->cmd,header->job,LINK_CMD_STATUS_STARTED,0);
}
static void command(const uint8_t *payload,unsigned length)
{
    if(length<LINK_CMD_HEADER_SIZE)return;
    link_cmd_header_t header;memcpy(&header,payload,LINK_CMD_HEADER_SIZE);
    const uint8_t *args=payload+LINK_CMD_HEADER_SIZE;unsigned alen=length-LINK_CMD_HEADER_SIZE;
    uint8_t c=header.cmd,id=header.job;
    switch(c) {
    case LINK_COMMAND_STOP: {
        link_ev_stop_t stop={FW_LINK_STOP_REMOTE};
        fw_link_stop_flag=1;
        push_event(LINK_EVENT_STOP,&stop,LINK_EV_STOP_SIZE);
        cmd_ack(c,id,LINK_CMD_STATUS_DONE,0);return;
    }
    case LINK_COMMAND_LED_SYNC:
        if(alen<LINK_CMD_LED_SYNC_SIZE || args[0]>1)break;
        if(busy()) {
            /* Inline: LEDs only, the foreground owns the display. */
            if(args[0]==0)link_led_start_sequence(&led,hal_os_get_systicks());
            else link_led_blip(&led,hal_os_get_systicks());
            job.led_cmd_active=1;job.led_cmd_cmd=c;job.led_cmd_job=id;
            cmd_ack(c,id,LINK_CMD_STATUS_STARTED,0);
        } else queue_job(&header,args,alen,0);
        return;
    case LINK_COMMAND_MOVE: case LINK_COMMAND_TURN: case LINK_COMMAND_EXPLORE: case LINK_COMMAND_RUN:
    case LINK_COMMAND_PARAM_SAVE: {
        unsigned need=c==LINK_COMMAND_MOVE?LINK_CMD_MOVE_SIZE:c==LINK_COMMAND_TURN?LINK_CMD_TURN_SIZE:
                      c==LINK_COMMAND_EXPLORE?LINK_CMD_EXPLORE_SIZE:c==LINK_COMMAND_RUN?LINK_CMD_RUN_SIZE:0;
        uint8_t flags=0;
        if(alen<need)break;
        if(c==LINK_COMMAND_MOVE) {
            link_cmd_move_t m;memcpy(&m,args,LINK_CMD_MOVE_SIZE);
            if(m.kind>LINK_MOVE_KIND_START_CELL || (m.kind==LINK_MOVE_KIND_CELLS && !m.cells) || m.cells>16)break;
            flags=m.flags;
        } else if(c==LINK_COMMAND_TURN) {
            link_cmd_turn_t t;memcpy(&t,args,LINK_CMD_TURN_SIZE);
            if(t.kind>LINK_TURN_KIND_UTURN)break;
            flags=t.flags;
        } else if(c==LINK_COMMAND_EXPLORE)flags=args[2];
        else if(c==LINK_COMMAND_RUN) {if(args[0]!=1 && args[0]!=2)break;flags=args[3];}
        if(busy()) {cmd_ack(c,id,LINK_CMD_STATUS_BUSY,0);return;}
        queue_job(&header,args,alen,flags);return;
    }
    case LINK_COMMAND_CALIB: case LINK_COMMAND_HALT: case LINK_COMMAND_BOOTLOADER:
        /* Calibrations need the robot placed by hand at each step (menu);
         * no power switch; the bootloader speaks SPI3, not BLE. */
        cmd_ack(c,id,LINK_CMD_STATUS_UNSUPPORTED,0);return;
    case LINK_COMMAND_REBOOT:
        if(busy()) {cmd_ack(c,id,LINK_CMD_STATUS_BUSY,0);return;}
        /* As ZHONX III: STARTED, then the power goes, no CMD_DONE. */
        cmd_ack(c,id,LINK_CMD_STATUS_STARTED,0);
        link.reboot_at=hal_os_get_systicks()+REBOOT_DELAY_MS;if(!link.reboot_at)link.reboot_at=1;
        return;
    case LINK_COMMAND_TELEM_CFG: {
        if(alen<LINK_CMD_TELEM_CFG_SIZE)break;
        link_cmd_telem_cfg_t cfg;memcpy(&cfg,args,LINK_CMD_TELEM_CFG_SIZE);
        fw_link_set_telemetry_rate(cfg.rate_hz);link.log_enabled=cfg.enable_log!=0;
        uint8_t ev[LINK_CMD_TELEM_CFG_SIZE]={link.rate_hz,link.log_enabled};
        push_event(LINK_EVENT_TELEM_CFG,ev,sizeof ev);
        cmd_ack(c,id,LINK_CMD_STATUS_DONE,0);return;
    }
    case LINK_COMMAND_FLUSH:
        link_tring_flush(&tring);link_ering_flush(&ering);
        cmd_ack(c,id,LINK_CMD_STATUS_DONE,0);return;
    case LINK_COMMAND_PARAM_LIST: {
        if(busy()) {cmd_ack(c,id,LINK_CMD_STATUS_BUSY,0);return;}
        cmd_ack(c,id,LINK_CMD_STATUS_DONE,(uint8_t)link_params_count(&params));
        link_param_info_t info;
        for(unsigned i=0;link_params_info(&params,i,&info);++i)reply(LINK_FRAME_TYPE_PARAM_INFO,&info,LINK_PARAM_INFO_SIZE);
        return;
    }
    case LINK_COMMAND_PARAM_GET: case LINK_COMMAND_PARAM_SET: {
        unsigned need=c==LINK_COMMAND_PARAM_GET?LINK_CMD_PARAM_ID_SIZE:LINK_CMD_PARAM_SET_SIZE;
        if(alen<need)break;
        if(busy()) {cmd_ack(c,id,LINK_CMD_STATUS_BUSY,0);return;}
        link_cmd_param_set_t set;memset(&set,0,sizeof set);memcpy(&set,args,need);
        link_param_value_t value;
        if(c==LINK_COMMAND_PARAM_GET)link_params_get(&params,set.id,&value);
        else link_params_set(&params,set.id,set.value,&value);
        cmd_ack(c,id,value.status,0);
        reply(LINK_FRAME_TYPE_PARAM_VALUE,&value,LINK_PARAM_VALUE_SIZE);return;
    }
    case LINK_COMMAND_DUMP: {
        if(alen<LINK_CMD_DUMP_SIZE)break;
        link_cmd_dump_t d;memcpy(&d,args,LINK_CMD_DUMP_SIZE);
        if(busy()) {cmd_ack(c,id,LINK_CMD_STATUS_BUSY,0);return;}
        if(!dump_range_ok(d.addr,d.len))break;
        dump.addr=d.addr;dump.left=d.len;dump.active=1;
        cmd_ack(c,id,LINK_CMD_STATUS_STARTED,0);return;
    }
    case LINK_COMMAND_CONSOLE: {
        if(busy()) {cmd_ack(c,id,LINK_CMD_STATUS_BUSY,0);return;}
        char line[sizeof console_text];
        unsigned n=alen<sizeof line-1?alen:sizeof line-1;
        memcpy(line,args,n);line[n]=0;
        cmd_ack(c,id,LINK_CMD_STATUS_DONE,0);
        console_run(line);return;
    }
    default: break;
    }
    cmd_ack(c,id,LINK_CMD_STATUS_INVALID,0);
}
static void frame_received(const link_frame_t *frame,uint32_t t_rx_us)
{
    ++link.status.frames_rx;link.last_rx_ms=hal_os_get_systicks();
    switch(frame->type) {
    case LINK_FRAME_TYPE_HELLO: hello();break;
    case LINK_FRAME_TYPE_PING: {
        link_pong_t pong;memset(&pong,0,sizeof pong);
        if(frame->len>=LINK_PING_SIZE)memcpy(&pong.token,frame->payload,sizeof pong.token);
        pong.t_robot_us=t_rx_us;
        reply(LINK_FRAME_TYPE_PONG,&pong,LINK_PONG_SIZE);break;
    }
    case LINK_FRAME_TYPE_ACK: {
        if(frame->len<LINK_STREAM_ACK_SIZE)break;
        link_stream_ack_t ack;memcpy(&ack,frame->payload,LINK_STREAM_ACK_SIZE);
        link_tring_ack(&tring,ack.telemetry_seq);link_ering_ack(&ering,ack.event_seq);break;
    }
    case LINK_FRAME_TYPE_RESEND: {
        if(frame->len<LINK_RESEND_SIZE)break;
        link_resend_t r;memcpy(&r,frame->payload,LINK_RESEND_SIZE);
        if(r.stream==LINK_STREAM_TELEMETRY)link_tring_resend(&tring,r.from_seq,r.to_seq);
        else if(r.stream==LINK_STREAM_EVENTS)link_ering_resend(&ering,r.from_seq,r.to_seq);
        break;
    }
    case LINK_FRAME_TYPE_CMD: command(frame->payload,frame->len);break;
    default: break;
    }
}
static void pump_rx(void)
{
    uint8_t byte;
    while(fw_bt_read(&byte)) {
        /* Notifications are matched on every byte: one cut inside a frame
         * must still end the connection. */
        int notice=fw_hm18_watch(&watch,byte);
        if(notice) {connection(notice>0);link_line_init(&console_line,console_text,sizeof console_text);}
        link_frame_t frame;
        int result=link_parser_feed(&parser,byte,&frame);
        if(result==LINK_FEED_FRAME) {connection(1);frame_received(&frame,link_time_us());}
        else if(result==LINK_FEED_STRAY && !notice) {
            link.last_rx_ms=hal_os_get_systicks();
            console_byte(byte);
        }
    }
    link.status.crc_errors=parser.crc_errors;
}

/* ---- light signal ------------------------------------------------------------------- */
void link_led_port_set(int on,int with_oled)
{
    hal_led_set_state(app_context.led,HAL_LED_COLOR_RED,(unsigned char)!!on);
    hal_led_set_state(app_context.led,HAL_LED_COLOR_ORANGE,(unsigned char)!!on);
    /* The display belongs to the foreground: only a foreground sequence uses it. */
    if(with_oled && link.led_foreground && !__get_IPSR())ssd1306DisplayAllOn(on);
}
void link_led_port_event(uint8_t index,uint8_t state,uint32_t t_us)
{
    link_ev_led_sync_t sync={index,state};
    link_ering_push(&ering,LINK_EVENT_LED_SYNC,t_us,&sync,LINK_EV_LED_SYNC_SIZE);
}
static void poll_led(uint32_t now)
{
    if(link.led_foreground)return;
    link_led_poll(&led,now);
    if(job.led_cmd_active && !link_led_sequence_active(&led) && !led.blip_on && !led.single_blip) {
        link_ev_cmd_done_t done={job.led_cmd_cmd,job.led_cmd_job,LINK_CMD_STATUS_DONE};
        job.led_cmd_active=0;push_event(LINK_EVENT_CMD_DONE,&done,LINK_EV_CMD_DONE_SIZE);
    }
}
/* Foreground: runs a light sequence (or one blip) with the display. */
static void foreground_light(int sequence)
{
    link.led_foreground=1;
    uint32_t now=hal_os_get_systicks();
    if(sequence)link_led_start_sequence(&led,now);else link_led_blip(&led,now);
    do {link_led_poll(&led,hal_os_get_systicks());__WFI();}
    while((link_led_sequence_active(&led) || led.blip_on || led.single_blip) && !fw_link_stop_flag);
    if(link_led_is_on(&led)) {link_led_port_set(0,1);led.on=0;}
    link.led_foreground=0;
}

/* ---- PendSV --------------------------------------------------------------------------- */
void PendSV_Handler(void)
{
    if(!link.initialised)return;
    uint32_t now=hal_os_get_systicks();
    fw_bt_poll(now);
    pump_rx();
    poll_led(now);
    int fault=fw_motion_fault();
    if(fault!=link.last_fault) {
        link.last_fault=fault;
        if(fault) {
            /* As ZHONX III's emergencyStop(): STOP 2, then FAULT with the motion fault code. */
            link_ev_stop_t stop={FW_LINK_STOP_EMERGENCY};link_ev_fault_t f={1,(uint16_t)fault};
            push_event(LINK_EVENT_STOP,&stop,LINK_EV_STOP_SIZE);
            push_event(LINK_EVENT_FAULT,&f,LINK_EV_FAULT_SIZE);
        }
    }
    report_dropped(now);
    if(link.session && now-link.last_rx_ms>SESSION_TIMEOUT_MS)session_end();
    pump_tx(now);
    /* The acknowledgement left long before: a few ms at the transmit budget. */
    if(link.reboot_at && (int32_t)(now-link.reboot_at)>=0)NVIC_SystemReset();
}

/* ---- telemetry (SysTick) ----------------------------------------------------------- */
int fw_link_telemetry_rate(void) {return link.rate_hz;}
void fw_link_set_telemetry_rate(int rate)
{
    if(rate<=0) {link.sample_period=0;link.rate_hz=0;return;}
    if(rate>TELEMETRY_RATE_MAX)rate=TELEMETRY_RATE_MAX;
    uint16_t period=(uint16_t)(1000/rate);
    link.rate_hz=(uint8_t)(1000/period);link.sample_period=period;
}
int fw_link_blip_period(void) {return (int)led.blip_period_ms;}
void fw_link_set_blip_period(int period) {link_led_set_blips(&led,led.blips_enabled,(uint32_t)period,hal_os_get_systicks());}
static int16_t clamp16(double v) {return (int16_t)(v>32767?32767:v<-32768?-32768:v);}
static uint8_t motion_state(void)
{
    if(telem.motion_override)return telem.motion_override;
    if(fw_motion_fault())return LINK_MOTION_STATE_FAULT;
    if(fw_link_stop_flag && fw_motion_busy())return LINK_MOTION_STATE_STOPPING;
    switch(fw_motion_kind()) {
    case FW_MOTION_STRAIGHT: return LINK_MOTION_STATE_STRAIGHT;
    case FW_MOTION_TURN: return LINK_MOTION_STATE_TURN_IN_PLACE;
    case FW_MOTION_CURVE: return LINK_MOTION_STATE_CURVE;
    case FW_MOTION_CALIB: return LINK_MOTION_STATE_CALIB;
    default: return LINK_MOTION_STATE_IDLE;
    }
}
static unsigned battery_mv(void)
{
    fw_battery_status_t b=fw_battery_status();
    return b.voltage_valid?b.pack_mv:0;
}
void fw_link_tick(uint32_t now)
{
    (void)now;
    if(!link.initialised)return;
    SCB->ICSR=SCB_ICSR_PENDSVSET_Msk;
    if(!link.session)return;
    if(++telem.battery_tick>=1000) {
        link_ev_battery_t battery={(uint16_t)battery_mv()};
        telem.battery_tick=0;push_event(LINK_EVENT_BATTERY,&battery,LINK_EV_BATTERY_SIZE);
    }
    if(!link.sample_period || ++telem.tick<link.sample_period)return;
    telem.tick=0;
    link_telemetry_record_t rec;memset(&rec,0,sizeof rec);
    rec.t_us=link_time_us();
    rec.cell_x=telem.cell_x;rec.cell_y=telem.cell_y;rec.heading=telem.heading;rec.motion=motion_state();
    /* Wheel 0 is the right one. Steps are commanded, not measured: a lost
     * step or a slip stays invisible here. */
    int32_t um[2];
    for(unsigned i=0;i<2;++i)um[i]=(int32_t)llround((double)hal_step_motor_odometer[i]*1000.0/TICKS_PER_MM);
    rec.enc_right_um=um[0];rec.enc_left_um=um[1];
    double rate=link.rate_hz;
    rec.speed_mm_s=clamp16(((um[0]-telem.last_um[0])+(um[1]-telem.last_um[1]))*rate/2000.0);
    /* No gyroscope: heading from the wheel steps, clockwise positive as ZHONX III's gyro_cdeg. */
    double yaw=telem.track_um?(double)(um[1]-um[0])/telem.track_um:0.0;
    rec.gyro_cdeg=(int32_t)llround(yaw*18000.0/M_PI);
    rec.omega_ddps=clamp16((yaw-telem.last_yaw)*rate*1800.0/M_PI);
    telem.last_um[0]=um[0];telem.last_um[1]=um[1];telem.last_yaw=yaw;
    /* Binary IR sensors: no distance, only the detected walls (tel_*_mm = 0). */
    hal_sensor_snapshot scan;
    if(hal_sensor_snapshot_read(&scan)) {
        uint8_t s=scan.filtered;
        if(!(s&SENSOR_F10_POS))rec.walls|=LINK_WALL_BITS_FL|LINK_WALL_BITS_FR;
        if(!(s&SENSOR_L10_POS))rec.walls|=LINK_WALL_BITS_DL;
        if(!(s&SENSOR_R10_POS))rec.walls|=LINK_WALL_BITS_DR;
    }
    if(link_led_is_on(&led))rec.flags|=LINK_TELEMETRY_FLAGS_LED_ON;
    if(fw_motion_busy())rec.flags|=LINK_TELEMETRY_FLAGS_MOTION_BUSY;
    if(fw_motion_fault())rec.flags|=LINK_TELEMETRY_FLAGS_FAULT;
    if(moved)rec.flags|=LINK_TELEMETRY_FLAGS_WALL_FOLLOW;
    rec.bat_mv=(uint16_t)battery_mv();
    rec.target_mm=fw_motion_busy()?clamp16(fw_motion_remaining()/TICKS_PER_MM):0;
    link_tring_push(&tring,&rec);
}

/* ---- hooks ----------------------------------------------------------------------------- */
int fw_link_session(void) {return link.session;}
int fw_link_connected(void) {return fw_bt_connected();}
void fw_link_quiet(int quiet) {fw_bt_quiet(quiet && !link.session);}
void fw_link_menu_idle(int idle) {link.menu_idle=(uint8_t)!!idle;}
int fw_link_job_pending(void) {return job.state==JOB_PENDING;}
static int remote(void) {return job.state==JOB_RUNNING;}
static void start_event(void)
{
    link_ev_start_t start={remote()?job.current.cmd:0};
    last_cell.valid=0;memset(entries,0,sizeof entries);
    push_event(LINK_EVENT_START,&start,LINK_EV_START_SIZE);
}
int fw_link_start_gate(void)
{
    if(remote()) {
        uint8_t flags=job.current.flags;
        if(flags&LINK_CMD_FLAGS_LED_SYNC_BEFORE_START) {
            telem.motion_override=LINK_MOTION_STATE_LED_SYNC;
            foreground_light(1);
            telem.motion_override=0;
            if(fw_link_stop_flag)return -1;
        }
        if(flags&LINK_CMD_FLAGS_LED_BLIPS_DURING_MOTION) {
            link_led_reset_index(&led);link_led_set_blips(&led,1,led.blip_period_ms,hal_os_get_systicks());
        }
        if(!(flags&LINK_CMD_FLAGS_WAIT_HAND)) {start_event();return 1;}
    }
    telem.motion_override=LINK_MOTION_STATE_WAIT_HAND;
    push_event(LINK_EVENT_HAND_WAIT,0,0);
    return 0;
}
void fw_link_hand_go(void)
{
    telem.motion_override=0;
    push_event(LINK_EVENT_HAND_GO,0,0);
    start_event();
}
void fw_link_entered(nm_pose_t cell)
{
    entries[entry_next].x=cell.x;entries[entry_next].y=cell.y;entries[entry_next].valid=1;
    entries[entry_next].t_us=link_time_us();entry_next=(entry_next+1)%ENTRIES;
}
void fw_link_rebase(int dx,int dy)
{
    for(unsigned i=0;i<ENTRIES;++i) {entries[i].x=(uint8_t)(entries[i].x+dx);entries[i].y=(uint8_t)(entries[i].y+dy);}
    last_cell.x=(uint8_t)(last_cell.x+dx);last_cell.y=(uint8_t)(last_cell.y+dy);
}
static uint32_t entry_time(nm_pose_t p)
{
    for(unsigned k=1;k<=ENTRIES;++k) {
        unsigned i=(entry_next+ENTRIES-k)%ENTRIES;
        if(entries[i].valid && entries[i].x==p.x && entries[i].y==p.y) {entries[i].valid=0;return entries[i].t_us;}
    }
    return link_time_us();
}
void fw_link_cell(const nm_map_t *map,nm_pose_t p,uint8_t seen)
{
    if(last_cell.valid && last_cell.x==p.x && last_cell.y==p.y)return;
    last_cell.x=p.x;last_cell.y=p.y;last_cell.valid=1;
    link_ev_cell_t cell;
    fw_ariane_frame(map,p.x,p.y,p.heading,&cell.x,&cell.y,&cell.heading);
    cell.walls=fw_ariane_walls(map,p.heading,seen);
    telem.cell_x=cell.x;telem.cell_y=cell.y;telem.heading=cell.heading;
    if(link.initialised)link_ering_push(&ering,LINK_EVENT_CELL,entry_time(p),&cell,LINK_EV_CELL_SIZE);
}
void fw_link_decision(const nm_map_t *map,unsigned from_heading,nm_pose_t target)
{
    link_ev_decision_t d;uint8_t h;
    fw_ariane_frame(map,target.x,target.y,target.heading,&d.x,&d.y,&h);
    d.direction=(uint8_t)((target.heading+4-from_heading)&3);
    push_event(LINK_EVENT_DECISION,&d,LINK_EV_DECISION_SIZE);
}
void fw_link_move_begin(uint8_t kind,int16_t param,uint16_t speed)
{
    if(move.active)fw_link_move_end(0);
    link_ev_move_begin_t b={kind,param,speed};
    move.active=1;move.kind=kind;moved=1;
    push_event(LINK_EVENT_MOVE_BEGIN,&b,LINK_EV_MOVE_BEGIN_SIZE);
}
/* ZHONX III parameters: degrees, clockwise positive (CW90 90, CCW90 -90, 180 and U-turn 180). */
void fw_link_turn_begin(int degrees,int curve,unsigned speed)
{
    uint8_t kind=curve==FW_LINK_UTURN?LINK_TURN_KIND_UTURN:
        curve?(degrees>0?LINK_TURN_KIND_CURVE_CW90:LINK_TURN_KIND_CURVE_CCW90):
        degrees==180 || degrees==-180?LINK_TURN_KIND_IN_PLACE_180:
        degrees>0?LINK_TURN_KIND_IN_PLACE_CW90:LINK_TURN_KIND_IN_PLACE_CCW90;
    int16_t param=(int16_t)(kind==LINK_TURN_KIND_IN_PLACE_180 || kind==LINK_TURN_KIND_UTURN?180:degrees);
    fw_link_move_begin((uint8_t)(FW_LINK_MOVE_TURN|kind),param,(uint16_t)speed);
}
void fw_link_move_end(int stopped)
{
    if(!move.active)return;
    link_ev_move_end_t e={move.kind,(uint8_t)(stopped!=0)};
    move.active=0;push_event(LINK_EVENT_MOVE_END,&e,LINK_EV_MOVE_END_SIZE);
}
void fw_link_run_done(uint32_t ms,int failed)
{
    link_ev_run_done_t d={ms,(uint8_t)(failed!=0)};
    push_event(LINK_EVENT_RUN_DONE,&d,LINK_EV_RUN_DONE_SIZE);
}
void fw_link_status(fw_link_status_t *s)
{
    *s=link.status;s->session=link.session;s->telemetry_hz=link.rate_hz;s->build_id=link.build_id;
    s->telemetry_dropped=tring.dropped_total;s->events_dropped=ering.dropped_total;
}

/* ---- jobs (foreground) ----------------------------------------------------------------- */
static void job_screen(const char *what)
{
    ssd1306ClearScreen();
    fw_display_text(0,0,"ARIANE");fw_display_text(0,22,what);fw_display_text(0,44,"ESC: STOP");
    ssd1306Refresh();
}
static unsigned speed_or(uint16_t requested,int fallback,unsigned maximum)
{
    unsigned speed=requested?requested:(unsigned)fallback;
    return speed<20?20:speed>maximum?maximum:speed;
}
/* Hand wave on the F10 sensor, as wait_hand() in fw_app.c. */
static int hand_wave(void)
{
    fw_start_gate_t gate={0};
    for(;;) {
        if(fw_cancel_pressed())return -1;
        uint32_t now=hal_os_get_systicks();hal_sensor_snapshot scan;
        int fresh=hal_sensor_snapshot_read(&scan) && now-scan.timestamp<=50;
        if(fw_start_gate(&gate,now,fresh && !(scan.filtered&SENSOR_F10_POS),fresh))return 0;
        __WFI();
    }
}
static uint8_t wait_motion(void)
{
    while(fw_motion_busy()) {
        if(fw_cancel_pressed())fw_motion_stop();
        __WFI();
    }
    int stopped=fw_link_stop_flag || fw_cancel_button();
    fw_link_move_end(stopped);
    if(stopped)return LINK_CMD_STATUS_REFUSED;
    return fw_motion_fault()?LINK_CMD_STATUS_ERROR:LINK_CMD_STATUS_DONE;
}
static uint8_t job_gate(void)
{
    int gate=fw_link_start_gate();
    if(gate<0)return LINK_CMD_STATUS_REFUSED;
    if(!gate) {if(hand_wave())return LINK_CMD_STATUS_REFUSED;fw_link_hand_go();}
    return 0;
}
static uint8_t job_move(const job_t *j)
{
    link_cmd_move_t m;memcpy(&m,j->args,LINK_CMD_MOVE_SIZE);
    unsigned speed=speed_or(m.speed_mm_s,fw_search_speed,FW_RUN_MAX_SPEED);
    job_screen("MOVE");
    uint8_t status=job_gate();if(status)return status;
    fw_motion_init();
    /* START_CELL leaves the start cell: ZHONX II starts at its centre, so one cell. */
    unsigned cells=m.kind==LINK_MOVE_KIND_START_CELL?1u:m.cells;
    if(fw_motion_straight_to(cells,speed,1))return LINK_CMD_STATUS_ERROR;
    fw_link_move_begin(m.kind,(int16_t)cells,(uint16_t)speed);
    return wait_motion();
}
static uint8_t job_turn(const job_t *j)
{
    link_cmd_turn_t t;memcpy(&t,j->args,LINK_CMD_TURN_SIZE);
    /* In place, the wheels follow the travel speed (fw_motion_turn_speed). */
    unsigned travel=speed_or(t.speed_mm_s,fw_search_speed,FW_RUN_MAX_SPEED);
    unsigned speed=travel<FW_CURVE_MAX_SPEED?travel:FW_CURVE_MAX_SPEED;
    job_screen("TURN");
    uint8_t status=job_gate();if(status)return status;
    fw_motion_init();
    int result,degrees=90,shape=FW_LINK_IN_PLACE;
    switch(t.kind) {
    case LINK_TURN_KIND_IN_PLACE_CW90: result=fw_motion_turn(90,travel);break;
    case LINK_TURN_KIND_IN_PLACE_CCW90: result=fw_motion_turn(degrees=-90,travel);break;
    case LINK_TURN_KIND_IN_PLACE_180: result=fw_motion_turn(degrees=180,travel);break;
    case LINK_TURN_KIND_CURVE_CW90: result=fw_motion_curve(90,speed,1);shape=FW_LINK_CURVE;break;
    case LINK_TURN_KIND_CURVE_CCW90: result=fw_motion_curve(degrees=-90,speed,1);shape=FW_LINK_CURVE;break;
    /* No curved U-turn on ZHONX II: the same half turn, in place. */
    default: result=fw_motion_turn(degrees=180,travel);shape=FW_LINK_UTURN;break;
    }
    if(result)return LINK_CMD_STATUS_ERROR;
    fw_link_turn_begin(degrees,shape,shape==FW_LINK_CURVE?speed:fw_motion_turn_speed(travel));
    return wait_motion();
}
static uint8_t job_explore(const job_t *j)
{
    link_cmd_explore_t e;memcpy(&e,j->args,LINK_CMD_EXPLORE_SIZE);
    fw_app_set_search_override(e.speed_mm_s?speed_or(e.speed_mm_s,fw_search_speed,FW_RUN_MAX_SPEED):0);
    int result=fw_app_discover();
    fw_app_set_search_override(0);
    if(fw_link_stop_flag)return LINK_CMD_STATUS_REFUSED;
    /* As ZHONX III: DONE only for a proven maze (shortest route certified). */
    return !result && fw_app_ready()?LINK_CMD_STATUS_DONE:LINK_CMD_STATUS_ERROR;
}
static uint8_t job_run(const job_t *j)
{
    link_cmd_run_t r;memcpy(&r,j->args,LINK_CMD_RUN_SIZE);
    if(!fw_app_ready())return LINK_CMD_STATUS_ERROR;
    /* Speed 0: the run1 / run2 settings, as ZHONX III (parameters 13 and 16);
     * run1 follows the exploration speed until Ariane sets it. */
    int fallback=r.run_type!=1?fw_run_speed:fw_link_run1_speed?fw_link_run1_speed:fw_search_speed;
    int result=fw_app_trial(r.run_type,speed_or(r.speed_mm_s,fallback,FW_RUN_MAX_SPEED));
    if(fw_link_stop_flag)return LINK_CMD_STATUS_REFUSED;
    return result?LINK_CMD_STATUS_ERROR:LINK_CMD_STATUS_DONE;
}
static uint8_t job_light(const job_t *j)
{
    telem.motion_override=LINK_MOTION_STATE_LED_SYNC;
    foreground_light(j->args[0]==0);
    telem.motion_override=0;
    return LINK_CMD_STATUS_DONE;
}
static uint8_t job_save(void)
{
    if(fw_app_settings_save())return LINK_CMD_STATUS_ERROR;
    push_event(LINK_EVENT_PARAMS_SAVED,0,0);
    return LINK_CMD_STATUS_DONE;
}
void fw_link_job_run(void)
{
    if(job.state!=JOB_PENDING)return;
    job.current=job.pending;fw_link_stop_flag=0;job.state=JOB_RUNNING;
    uint8_t status;
    switch(job.current.cmd) {
    case LINK_COMMAND_LED_SYNC: status=job_light(&job.current);break;
    case LINK_COMMAND_MOVE: status=job_move(&job.current);break;
    case LINK_COMMAND_TURN: status=job_turn(&job.current);break;
    case LINK_COMMAND_EXPLORE: status=job_explore(&job.current);break;
    case LINK_COMMAND_RUN: status=job_run(&job.current);break;
    case LINK_COMMAND_PARAM_SAVE: status=job_save();break;
    default: status=LINK_CMD_STATUS_INVALID;break;
    }
    fw_motion_stop();
    if(led.blips_enabled)link_led_set_blips(&led,0,led.blip_period_ms,hal_os_get_systicks());
    fw_link_move_end(fw_link_stop_flag);
    telem.motion_override=0;
    link_ev_cmd_done_t done={job.current.cmd,job.current.job,status};
    push_event(LINK_EVENT_CMD_DONE,&done,LINK_EV_CMD_DONE_SIZE);
    job.state=JOB_IDLE;
}

/* ---- init -------------------------------------------------------------------------------- */
void fw_link_init(void)
{
    memset(&link,0,sizeof link);memset(&job,0,sizeof job);memset(&dump,0,sizeof dump);
    memset(&move,0,sizeof move);memset(&telem,0,sizeof telem);memset(&watch,0,sizeof watch);
    telem.heading=LINK_HEADING_NORTH;telem.track_um=(uint32_t)(WHEELS_DISTANCE*1000.0);
    link.build_id=image_crc32();
    link_tring_init(&tring,telemetry_slots,RING_RECORDS);
    link_ering_init(&ering,event_slots,EVENT_ENTRIES);
    link_fifo_init(&reply_fifo,reply_data,REPLY_FIFO_SIZE);
    link_fifo_init(&log_fifo,log_data,LOG_FIFO_SIZE);
    link_parser_init(&parser);
    link_line_init(&console_line,console_text,sizeof console_text);
    link_led_init(&led);
    link_params_init(&params,fw_link_param_table,fw_link_param_table_count);
    fw_link_set_telemetry_rate(TELEMETRY_RATE_DEFAULT);
    fw_bt_init(hal_os_get_systicks());
    NVIC_SetPriority(PendSV_IRQn,15);
    /* First event after boot: the application resets its trackers on it. */
    link.initialised=1;
    push_event(LINK_EVENT_BOOT,0,0);
}
