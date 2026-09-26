#include "fw_calibration.h"
#include <string.h>
#define F5 0x04u
#define F10 0x08u
#define L5 0x10u
#define L10 0x20u
#define R5 0x02u
#define R10 0x01u
#define REPEATS 3u
#define STEP 1000u
#define OVERTRAVEL 5000u
#define MARGIN 2000u
static uint32_t min(uint32_t a,uint32_t b) { return a<b?a:b; }
static uint32_t max(uint32_t a,uint32_t b) { return a>b?a:b; }
static uint32_t clearance(const fw_cal_geometry_t *g)
{
    /* Robot centred on its axle; bound the swept rectangle including wheels. */
    uint64_t square=(uint64_t)g->nose_um*g->nose_um+(uint64_t)g->width_um*g->width_um/4;
    uint32_t r=0;
    while ((uint64_t)r*r<square) r+=1000;
    return r+MARGIN;
}
int fw_cal_geometry_valid(const fw_cal_geometry_t *g)
{
    if (!g || g->nose_um<10000 || g->nose_um>75000 || g->width_um<40000 ||
        g->width_um>140000 || g->inner_um<140000 || g->inner_um>190000 ||
        g->pitch_um<=g->inner_um || g->pitch_um>210000) return 0;
    return clearance(g)+STEP<g->inner_um/2;
}
int fw_cal_valid(const fw_cal_data_t *d)
{
    if (!d || d->valid!=1 || d->repetitions!=REPEATS || !fw_cal_geometry_valid(&d->geometry)) return 0;
    for (unsigned i=0;i<2;++i) {
        if (d->front[i].on_um<d->geometry.nose_um ||
            d->front[i].off_um<d->front[i].on_um ||
            d->front[i].off_um>d->geometry.inner_um || d->front[i].spread_um>2000) return 0;
        if (d->side[i].near_um<clearance(&d->geometry) ||
            d->side[i].far_um<=d->side[i].near_um ||
            d->side[i].far_um-d->side[i].near_um>STEP+d->side[i].spread_um ||
            d->side[i].far_um>d->geometry.inner_um-clearance(&d->geometry) ||
            d->side[i].spread_um>2000) return 0;
    }
    return 1;
}
typedef struct {
    uint32_t origin, edge[2], candidate[2];
    uint8_t previous[2], count[2], seen[2];
    int outward, bad;
} sweep_t;
static void observe(void *context,int32_t travelled,uint8_t raw,uint8_t filtered)
{
    (void)filtered;
    sweep_t *s=context;
    uint32_t distance=(uint32_t)((int32_t)s->origin-travelled);
    for (unsigned i=0;i<2;++i) {
        unsigned bit=i?F10:F5, far=!!(raw&bit);
        if (far==s->previous[i]) { s->count[i]=0; continue; }
        if (!s->count[i]) s->candidate[i]=distance;
        if (++s->count[i]<3) continue;
        s->previous[i]=far; s->count[i]=0;
        if (s->seen[i] || far!=(unsigned)s->outward) s->bad=1;
        s->seen[i]=1; s->edge[i]=s->candidate[i];
    }
}
static void status(const fw_cal_io_t *io,const char *text,uint32_t distance)
{ if (io->status) io->status(io->context,text,(int32_t)distance); }
static int move(const fw_cal_io_t *io,int32_t um)
{ return !um?0:io->move(io->context,um,20,0,0); }
static int seat(const fw_cal_io_t *io,const fw_cal_geometry_t *g,uint32_t distance)
{
    uint8_t raw;
    status(io,"SLOW WALL CONTACT",distance);
    if (move(io,(int32_t)(distance-g->nose_um+OVERTRAVEL)) || io->read(io->context,&raw))
        return FW_CAL_MOTION;
    return (raw&(F5|F10))?FW_CAL_WALLS:0;
}
/* Return to the original heading after every lateral measurement. */
int fw_cal_reference(const fw_cal_io_t *io,const fw_cal_geometry_t *g,unsigned side)
{
    if (!io || !fw_cal_geometry_valid(g) || side>1) return FW_CAL_GEOMETRY;
    int angle=side?90:-90;
    if (io->turn(io->context,angle)) return FW_CAL_MOTION;
    int r=seat(io,g,g->inner_um-g->nose_um); if (r) return r;
    if (move(io,-(int32_t)(g->inner_um/2-g->nose_um)) || io->turn(io->context,-angle))
        return FW_CAL_MOTION;
    r=seat(io,g,g->inner_um-g->nose_um); if (r) return r;
    return move(io,-(int32_t)(g->inner_um/2-g->nose_um))?FW_CAL_MOTION:0;
}
static int side_point(const fw_cal_io_t *io,const fw_cal_geometry_t *g,
                      unsigned side,uint32_t *position,uint32_t distance,int *detected,int *axis_known)
{
    int result=seat(io,g,*position); uint8_t raw;
    if (result) return result;
    if (move(io,-(int32_t)(distance-g->nose_um)) ||
        io->turn(io->context,side?-90:90)) return FW_CAL_MOTION;
    /* Seat on the perpendicular wall to remove yaw and establish the other axis. */
    result=seat(io,g,*axis_known?g->inner_um/2:g->inner_um-g->nose_um);
    if (result) return result;
    if (move(io,-(int32_t)(g->inner_um/2-g->nose_um)) || io->read(io->context,&raw))
        return FW_CAL_MOTION;
    *axis_known=1;
    if (raw&(side?R10:L10)) return FW_CAL_WALLS;
    status(io,side?"RIGHT 5CM SAMPLE":"LEFT 5CM SAMPLE",distance);
    *detected=!(raw&(side?R5:L5));
    if (io->turn(io->context,side?90:-90)) return FW_CAL_MOTION;
    *position=distance;
    return 0;
}
int fw_cal_run(const fw_cal_io_t *io,const fw_cal_geometry_t *g,fw_cal_data_t *out)
{
    fw_cal_data_t d={0}; uint8_t raw;
    if (!io || !io->move || !io->turn || !io->read || !out || !fw_cal_geometry_valid(g))
        return FW_CAL_GEOMETRY;
    /* Do not publish a partial result; caller keeps the previous calibration. */
    if (io->read(io->context,&raw)) return FW_CAL_MOTION;
    if (raw&(F10|L10|R10)) return FW_CAL_WALLS;
    uint32_t centre=g->inner_um/2, position=centre;
    /* First contact on each axis covers the cell's free travel. Subsequent
     * contacts use the established coordinate plus only 5 mm of overtravel. */
    int result=seat(io,g,g->inner_um-g->nose_um);
    if (result) return result;
    d.geometry=*g; d.repetitions=REPEATS;
    uint32_t low[2][2]={{UINT32_MAX,UINT32_MAX},{UINT32_MAX,UINT32_MAX}}, high[2][2]={{0}};
    for (unsigned repeat=0;repeat<REPEATS;++repeat) {
        for (unsigned direction=0;direction<2;++direction) {
            int outward=!direction;
            if (io->read(io->context,&raw)) return FW_CAL_MOTION;
            if ((raw&(F5|F10))!=(outward?0:(F5|F10))) return FW_CAL_RANGE;
            sweep_t sweep={.origin=outward?g->nose_um:g->inner_um,.outward=outward};
            sweep.previous[0]=!!(raw&F5); sweep.previous[1]=!!(raw&F10);
            status(io,outward?"REVERSE F5 / F10":"FORWARD F5 / F10",sweep.origin);
            int32_t travel=(int32_t)(g->inner_um-g->nose_um);
            if (io->move(io->context,outward?-travel:travel,10,observe,&sweep)) return FW_CAL_MOTION;
            if (sweep.bad) return FW_CAL_UNSTABLE;
            if (!sweep.seen[0] || !sweep.seen[1]) return FW_CAL_RANGE;
            for (unsigned i=0;i<2;++i) {
                if (outward) d.front[i].off_um+=sweep.edge[i]; else d.front[i].on_um+=sweep.edge[i];
                low[i][direction]=min(low[i][direction],sweep.edge[i]);
                high[i][direction]=max(high[i][direction],sweep.edge[i]);
            }
        }
        result=seat(io,g,g->nose_um); if (result) return result;
    }
    for (unsigned i=0;i<2;++i) {
        d.front[i].on_um/=REPEATS; d.front[i].off_um/=REPEATS;
        d.front[i].spread_um=max(high[i][0]-low[i][0],high[i][1]-low[i][1]);
        if (d.front[i].off_um<d.front[i].on_um || d.front[i].spread_um>2000)
            return FW_CAL_UNSTABLE;
    }
    if (move(io,-(int32_t)(centre-g->nose_um))) return FW_CAL_MOTION;
    /* Search each lateral threshold around the centre in 1 mm increments.
     * Re-seating and rotations reset optical history: these are static brackets. */
    uint32_t radius=clearance(g), span=min(15000,centre-radius);
    int axis_known=0;
    for (unsigned side=0;side<2;++side) {
        uint32_t lo=UINT32_MAX,hi=0;
        for (unsigned repeat=0;repeat<REPEATS;++repeat) {
            uint32_t distance=centre; int first,reading;
            result=side_point(io,g,side,&position,distance,&first,&axis_known); if (result) return result;
            for (;;) {
                uint32_t previous=distance;
                distance=first?distance+STEP:distance-STEP;
                if (distance<centre-span || distance>centre+span) return FW_CAL_RANGE;
                result=side_point(io,g,side,&position,distance,&reading,&axis_known); if (result) return result;
                if (reading!=first) {
                    uint32_t near=min(previous,distance);
                    lo=min(lo,near); hi=max(hi,near); break;
                }
            }
        }
        d.side[side].near_um=lo; d.side[side].far_um=hi+STEP;
        d.side[side].spread_um=hi-lo;
        if (hi-lo>2000) return FW_CAL_UNSTABLE;
    }
    result=seat(io,g,position); if (result) return result;
    if (move(io,-(int32_t)(centre-g->nose_um))) return FW_CAL_MOTION;
    d.valid=1;
    if (!fw_cal_valid(&d)) return FW_CAL_UNSTABLE;
    *out=d; return FW_CAL_OK;
}
