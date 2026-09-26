#include "fw_cal_extra.h"
#include <string.h>
#include <stdio.h>
#define F5 4u
#define F10 8u
#define REPEATS 3u
#define EDGES 64u
static const unsigned rotation_speeds[FW_CAL_SPEEDS]={40,80,120};
static const unsigned corner_speeds[FW_CAL_SPEEDS]={40,120,220};
static uint32_t absolute(int32_t n) { return (uint32_t)(n<0?-(int64_t)n:n); }
static uint32_t difference(uint32_t a,uint32_t b) { return a>b?a-b:b-a; }
static uint32_t minimum(uint32_t a,uint32_t b) { return a<b?a:b; }
static uint32_t maximum(uint32_t a,uint32_t b) { return a>b?a:b; }
int fw_rotation_valid(const fw_rotation_data_t *d)
{
    if (!d || d->valid!=1 || !fw_cal_geometry_valid(&d->geometry)) return 0;
    for (unsigned i=0;i<FW_CAL_SPEEDS;++i) {
        const fw_rotation_point_t *p=&d->point[i];
        if (p->speed!=rotation_speeds[i]) return 0;
        for (unsigned dir=0;dir<2;++dir)
            if (p->quarter_um[dir]<47000 || p->quarter_um[dir]>87000 ||
                p->spread_um[dir]>8000 || p->quarter_error_mdeg[dir]>2000 ||
                (p->quarter_checks[dir] && p->quarter_checks[dir]<4)) return 0;
    }
    return 1;
}
int fw_corner_valid(const fw_corner_data_t *d)
{
    if (!d || d->valid!=1 || d->side>1 || !fw_cal_geometry_valid(&d->geometry) ||
        d->post_um<d->geometry.inner_um/2+50000 || d->post_um>250000) return 0;
    for (unsigned facing=0;facing<2;++facing) for (unsigned i=0;i<FW_CAL_SPEEDS;++i) {
        const fw_corner_point_t *p=&d->point[facing][i];
        if (p->speed!=corner_speeds[i] || !(p->mask&2) || (p->mask&~3u)) return 0;
        for (unsigned s=0;s<2;++s) if (p->mask&(1u<<s)) {
            if (absolute(p->raw_open_um[s])>60000 || absolute(p->raw_close_um[s])>60000 ||
                absolute(p->open_um[s])>60000 || absolute(p->close_um[s])>60000 || p->spread_um[s]>5000) return 0;
        }
    }
    return 1;
}
uint32_t fw_rotation_quarter(const fw_rotation_data_t *d,unsigned speed,unsigned dir)
{
    if (!fw_rotation_valid(d) || dir>1 || speed<40 || speed>120) return 0;
    for (unsigned i=1;i<FW_CAL_SPEEDS;++i) if (speed<=d->point[i].speed) {
        const fw_rotation_point_t *a=&d->point[i-1], *b=&d->point[i];
        int64_t delta=(int64_t)b->quarter_um[dir]-a->quarter_um[dir];
        return (uint32_t)(a->quarter_um[dir]+delta*(speed-a->speed)/(b->speed-a->speed));
    }
    return 0;
}
int fw_corner_offset(const fw_corner_data_t *d,unsigned facing,unsigned speed,unsigned sensor,int opening,int32_t *out)
{
    if (!out || !fw_corner_valid(d) || facing>1 || sensor>1 || speed<40 || speed>220) return -1;
    for (unsigned i=1;i<FW_CAL_SPEEDS;++i) if (speed<=d->point[facing][i].speed) {
        const fw_corner_point_t *a=&d->point[facing][i-1],*b=&d->point[facing][i];
        if (!(a->mask & b->mask & (1u<<sensor))) return -1;
        int32_t x=opening?a->open_um[sensor]:a->close_um[sensor];
        int32_t y=opening?b->open_um[sensor]:b->close_um[sensor];
        *out=x+(int32_t)((int64_t)(y-x)*(speed-a->speed)/(b->speed-a->speed)); return 0;
    }
    return -1;
}
typedef struct {
    uint32_t position[EDGES],candidate;
    uint8_t far[EDGES],state,count,used;
} optical_t;
typedef struct { optical_t sensor[2]; uint32_t offset; int overflow; } rotation_scan_t;
static void rotation_observe(void *context,int32_t travel,uint8_t raw,uint8_t filtered)
{
    (void)filtered; rotation_scan_t *scan=context;
    uint32_t position=scan->offset+absolute(travel);
    for (unsigned i=0;i<2;++i) {
        optical_t *s=&scan->sensor[i]; unsigned far=!!(raw&(i?F10:F5));
        if (far==s->state) { s->count=0; continue; }
        if (!s->count) s->candidate=position;
        if (++s->count<3) continue;
        s->state=(uint8_t)far; s->count=0;
        if (s->used==EDGES) { scan->overflow=1; continue; }
        s->position[s->used]=s->candidate; s->far[s->used++]=(uint8_t)far;
    }
}
/* The open rear of a three-wall cell gives the longest clear interval per turn.
 * Its midpoint repeats at 360 degrees for the SAME sensor and SAME direction;
 * mounting angle, fixed threshold and hysteresis therefore cancel. */
static int rotation_fit(const rotation_scan_t *scan,uint32_t *period,uint32_t *spread,
                        uint32_t *quarter_error,uint32_t *checks)
{
    if (scan->overflow) return FW_CAL_RANGE;
    const optical_t *s=&scan->sensor[1]; uint32_t longest=0,anchor[EDGES],count=0;
    for (unsigned i=0;i+1<s->used;++i) if (s->far[i] && !s->far[i+1])
        longest=maximum(longest,s->position[i+1]-s->position[i]);
    if (!longest) return FW_CAL_RANGE;
    for (unsigned i=0;i+1<s->used;++i) if (s->far[i] && !s->far[i+1] &&
        s->position[i+1]-s->position[i]>=longest*3/4)
        anchor[count++]=(s->position[i]+s->position[i+1])/2;
    if (count<3) return FW_CAL_RANGE;
    uint32_t low=UINT32_MAX,high=0,sum=0;
    for (unsigned i=1;i<count;++i) {
        uint32_t p=anchor[i]-anchor[i-1];
        if (p<188000 || p>348000) return FW_CAL_UNSTABLE;
        low=minimum(low,p); high=maximum(high,p); sum+=p;
    }
    *period=sum/(count-1); *spread=high-low;
    if (*spread>*period/50+1000) return FW_CAL_UNSTABLE;
    /* When F5 resolves all three separate wall lobes, their centres give
     * an independent 90-degree check. No invented quarter reference otherwise. */
    s=&scan->sensor[0]; count=0; *checks=0; *quarter_error=0;
    for (unsigned i=0;i+1<s->used;++i) if (!s->far[i] && s->far[i+1])
        anchor[count++]=(s->position[i]+s->position[i+1])/2;
    uint32_t q=*period/4;
    for (unsigned i=1;i<count;++i) {
        uint32_t gap=anchor[i]-anchor[i-1];
        if (gap>q*6/10 && gap<q*14/10) {
            ++*checks; *quarter_error=maximum(*quarter_error,
                (uint32_t)((uint64_t)difference(gap,q)*360000/ *period));
        }
    }
    if (*checks<4) { *checks=0; *quarter_error=0; }
    return *quarter_error>2000?FW_CAL_UNSTABLE:0;
}
static int collect_rotation(const fw_cal_extra_io_t *io,unsigned speed,unsigned dir,
                            uint32_t distance,unsigned chunks,rotation_scan_t *scan)
{
    memset(scan,0,sizeof *scan); uint8_t raw;
    if (io->base.read(io->base.context,0,&raw)) return FW_CAL_MOTION;
    scan->sensor[0].state=!!(raw&F5); scan->sensor[1].state=!!(raw&F10);
    for (unsigned i=0;i<chunks;++i) {
        scan->offset=i*distance;
        if (io->spin(io->base.context,dir?-(int32_t)distance:(int32_t)distance,
                     speed,rotation_observe,scan)) return FW_CAL_MOTION;
    }
    return 0;
}
static int close_revolution(const fw_cal_extra_io_t *io,unsigned speed,unsigned dir,uint32_t used,uint32_t period)
{
    uint32_t residual=used%period;
    if (!residual) return 0;
    int32_t correction=(int32_t)(period-residual);
    return io->spin(io->base.context,dir?-correction:correction,speed,0,0)?FW_CAL_MOTION:0;
}
int fw_rotation_run(const fw_cal_extra_io_t *io,const fw_cal_geometry_t *g,fw_rotation_data_t *out)
{
    if (!io || !io->spin || !out || !fw_cal_geometry_valid(g)) return FW_CAL_GEOMETRY;
    fw_rotation_data_t result={0}; result.geometry=*g;
    for (unsigned v=0;v<FW_CAL_SPEEDS;++v) for (unsigned dir=0;dir<2;++dir) {
        uint8_t raw;
        if (io->base.read(io->base.context,F10|1u|32u,&raw)) return FW_CAL_MOTION;
        if (raw&(F10|1u|32u)) return FW_CAL_WALLS;
        int r=fw_cal_reference(&io->base,g,0); if (r) return r;
        unsigned speed=rotation_speeds[v];
        char label[24]; snprintf(label,sizeof label,"%s %u MM/S",dir?"CCW":"CW",speed);
        if (io->base.status) io->base.status(io->base.context,label,0);
        rotation_scan_t scan;
        /* Five nominal revolutions; broad track bounds are validated optically. */
        uint32_t used=1311615,period=0,spread=0,error=0,checks=0;
        r=collect_rotation(io,speed,dir,used,1,&scan); if (r) return r;
        r=rotation_fit(&scan,&period,&spread,&error,&checks); if (r) return r;
        r=close_revolution(io,speed,dir,used,period); if (r) return r;
        uint32_t quarter=period/4; int converged=0;
        /* Validate/refine using the actual start/stop profile of 90-degree turns. */
        for (unsigned pass=0;pass<3;++pass) {
            r=collect_rotation(io,speed,dir,quarter,16,&scan); if (r) return r;
            r=rotation_fit(&scan,&period,&spread,&error,&checks); if (r) return r;
            r=close_revolution(io,speed,dir,quarter*16,period); if (r) return r;
            uint32_t measured=period/4;
            if (difference(quarter,measured)<=quarter/200+100) converged=1;
            quarter=measured;
            if (converged) break;
        }
        if (!converged) return FW_CAL_UNSTABLE;
        fw_rotation_point_t *p=&result.point[v]; p->speed=speed;
        p->quarter_um[dir]=quarter; p->spread_um[dir]=spread;
        p->quarter_error_mdeg[dir]=error; p->quarter_checks[dir]=checks;
    }
    int r=fw_cal_reference(&io->base,g,0); if (r) return r;
    result.valid=1;
    if (!fw_rotation_valid(&result)) return FW_CAL_UNSTABLE;
    *out=result; return 0;
}
typedef struct {
    uint32_t origin,mask,bit[2],candidate[2],raw_edge[2],filtered_edge[2];
    uint8_t previous[2],count[2],raw_seen[2],filter_previous[2],filter_seen[2];
    int opening,bad,facing_out;
} corner_scan_t;
static void corner_observe(void *context,int32_t travel,uint8_t raw,uint8_t filtered)
{
    corner_scan_t *s=context;
    uint32_t position=(uint32_t)((int32_t)s->origin+(s->facing_out?travel:-travel));
    for (unsigned i=0;i<2;++i) if (s->mask&(1u<<i)) {
        unsigned far=!!(raw&s->bit[i]), filter_far=!!(filtered&s->bit[i]);
        if (far==s->previous[i]) s->count[i]=0;
        else {
            if (!s->count[i]) s->candidate[i]=position;
            if (++s->count[i]>=3) {
                if (s->raw_seen[i] || far!=(unsigned)s->opening) s->bad=1;
                s->raw_seen[i]=1; s->raw_edge[i]=s->candidate[i];
                s->previous[i]=(uint8_t)far; s->count[i]=0;
            }
        }
        if (filter_far!=s->filter_previous[i]) {
            if (s->filter_seen[i] || filter_far!=(unsigned)s->opening) s->bad=1;
            s->filter_seen[i]=1; s->filtered_edge[i]=position; s->filter_previous[i]=(uint8_t)filter_far;
        }
    }
}
int fw_corner_run(const fw_cal_io_t *io,const fw_cal_geometry_t *g,unsigned side,uint32_t post,fw_corner_data_t *out)
{
    if (!io || !out || side>1 || !fw_cal_geometry_valid(g) ||
        post<g->inner_um/2+50000 || post>250000) return FW_CAL_GEOMETRY;
    uint32_t bit[2]={side?2u:16u,side?1u:32u},centre=g->inner_um/2,end=post+100000;
    fw_corner_data_t result={.side=side,.post_um=post,.geometry=*g};
    uint8_t raw;
    if (io->read(io->context,F10|1u|32u,&raw)) return FW_CAL_MOTION;
    if (raw&(F10|bit[1])) return FW_CAL_WALLS;
    if (!(raw&(side?32u:1u))) return FW_CAL_WALLS; /* Mirror two-wall fixtures. */
    for (unsigned facing=0;facing<2;++facing) for (unsigned v=0;v<FW_CAL_SPEEDS;++v) {
        unsigned sensor_side=facing?1-side:side;
        bit[0]=sensor_side?2u:16u; bit[1]=sensor_side?1u:32u;
        fw_corner_point_t *p=&result.point[facing][v]; p->speed=corner_speeds[v];
        uint32_t low[2][4],high[2][4]={{0}};
        for (unsigned s=0;s<2;++s) for (unsigned k=0;k<4;++k) low[s][k]=UINT32_MAX;
        for (unsigned repeat=0;repeat<REPEATS;++repeat) {
            int r=fw_cal_reference(io,g,side); if (r) return r;
            if (facing && io->turn(io->context,180)) return FW_CAL_MOTION;
            if (io->read(io->context,(uint8_t)bit[1],&raw)) return FW_CAL_MOTION;
            if (raw&bit[1]) return FW_CAL_WALLS;
            uint32_t mask=(raw&bit[0])?2u:3u;
            if (repeat && mask!=p->mask) return FW_CAL_UNSTABLE;
            p->mask=mask;
            for (unsigned direction=0;direction<2;++direction) {
                if (io->read(io->context,(uint8_t)(bit[1]|((mask&1)?bit[0]:0)),&raw)) return FW_CAL_MOTION;
                corner_scan_t scan={.origin=direction?end:centre,.mask=mask,
                    .bit={bit[0],bit[1]},.opening=!direction,.facing_out=(int)facing};
                for (unsigned s=0;s<2;++s) {
                    scan.previous[s]=scan.filter_previous[s]=!!(raw&bit[s]);
                    if ((mask&(1u<<s)) && scan.previous[s]!=(direction?1:0)) return FW_CAL_RANGE;
                }
                char label[24];
                snprintf(label,sizeof label,"%c %s %s %u",sensor_side?'R':'L',direction?"CLOSE":"OPEN",
                         (direction!=facing)?"FWD":"BACK",(unsigned)p->speed);
                if (io->status) io->status(io->context,label,0);
                int32_t travel=(int32_t)(end-centre);
                if (!direction) travel=-travel;
                if (facing) travel=-travel;
                if (io->move(io->context,travel,p->speed,corner_observe,&scan)) return FW_CAL_MOTION;
                if (scan.bad) return FW_CAL_UNSTABLE;
                /* Require cruise at the trigger: the calibration speed must be real. */
                uint32_t ramp=(p->speed*p->speed-40u*40u)*1000u/1600u+2000u;
                for (unsigned s=0;s<2;++s) if (mask&(1u<<s)) {
                    if (!scan.raw_seen[s] || !scan.filter_seen[s]) return FW_CAL_RANGE;
                    uint32_t values[]={scan.raw_edge[s],scan.filtered_edge[s]};
                    for (unsigned k=0;k<2;++k) {
                        if (values[k]<centre+ramp || values[k]>end-ramp) return FW_CAL_RANGE;
                        low[s][direction*2+k]=minimum(low[s][direction*2+k],values[k]);
                        high[s][direction*2+k]=maximum(high[s][direction*2+k],values[k]);
                    }
                    if (direction) {
                        p->raw_close_um[s]+=(int32_t)values[0]-(int32_t)post;
                        p->close_um[s]+=(int32_t)values[1]-(int32_t)post;
                    } else {
                        p->raw_open_um[s]+=(int32_t)values[0]-(int32_t)post;
                        p->open_um[s]+=(int32_t)values[1]-(int32_t)post;
                    }
                }
            }
            if (facing && io->turn(io->context,-180)) return FW_CAL_MOTION;
        }
        for (unsigned s=0;s<2;++s) if (p->mask&(1u<<s)) {
            p->raw_open_um[s]/=3; p->raw_close_um[s]/=3; p->open_um[s]/=3; p->close_um[s]/=3;
            for (unsigned k=0;k<4;++k) p->spread_um[s]=maximum(p->spread_um[s],high[s][k]-low[s][k]);
        }
    }
    int r=fw_cal_reference(io,g,side); if (r) return r;
    result.valid=1;
    if (!fw_corner_valid(&result)) return FW_CAL_UNSTABLE;
    *out=result; return 0;
}
