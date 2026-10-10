#include "fw_cal_follow.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define F5 0x04u
#define F10 0x08u
#define L5 0x10u
#define R5 0x02u
#define L10 0x20u
#define R10 0x01u
/* A pass needs crossings spread over its length for the curvature. */
#define MIN_CROSSINGS 4u
#define MIN_SPAN_MM 300.0
unsigned fw_follow_pass_speed(unsigned pass) { return pass<FW_FOLLOW_PASSES/2?220u:600u; }
int fw_follow_valid(const fw_follow_data_t *d)
{
    return d && d->valid==1 && fw_cal_geometry_valid(&d->geometry) &&
        d->cells>=FW_FOLLOW_MIN_CELLS && d->cells<=FW_FOLLOW_MAX_CELLS &&
        d->yaw_bias_mrad_m>=-FW_FOLLOW_BIAS_LIMIT && d->yaw_bias_mrad_m<=FW_FOLLOW_BIAS_LIMIT &&
        d->spread_mrad_m<=FW_FOLLOW_SPREAD_LIMIT && d->passes>=FW_FOLLOW_MIN_FITTED &&
        d->passes<=FW_FOLLOW_PASSES && d->crossings>=d->passes*MIN_CROSSINGS;
}
/* Least squares of the lateral position at each crossing: y = y0 + h0 s +
 * I(s) + (bias + d bend) s^2/2 = threshold + direction * hysteresis, I the
 * step-yaw path. Start position and heading of each pass are eliminated; the
 * bias and the hysteresis of both sensors are common to all passes, since the
 * follower crosses in both directions. A corridor that is not straight bends
 * every pass by the same curvature, of opposite sign on the way back (d = +1
 * out, -1 back): fitted when both directions have passes, it no longer reads
 * as passes that disagree (3 mm of sag over 70 cm: spread 45 mrad/m and a
 * third of the bias lost). Distances mm, headings mrad, lateral um. */
#define PASS 3u   /* bias, left hysteresis, right hysteresis */
#define GLOBAL 4u /* + corridor bend */
typedef struct {
    double threshold[2],a[5][5],b[5],yaw,path,travel,first,last;
    unsigned count;uint8_t previous,started;
} pass_fit_t;
typedef struct {double s[PASS][PASS],r[PASS];int direction,fast;} reduced_t;
static void observe(void *context,const fw_follow_sample_t *x)
{
    pass_fit_t *p=context;
    double s=x->travel_um*1e-3,yaw=x->yaw_urad*1e-3,lag=x->lag_um*1e-3;
    if(p->started)p->path+=(p->yaw+yaw)*.5*(s-p->travel);
    p->yaw=yaw;p->travel=s;
    if(p->started)for(unsigned sensor=0;sensor<2;++sensor) {
        unsigned bit=sensor?R5:L5,wall=sensor?R10:L10;
        /* A threshold crossing beside a confirmed wall; never a door edge. */
        if(((x->sensors|p->previous)&wall) || !((x->sensors^p->previous)&bit))continue;
        /* Moving right: L5 turns far, R5 turns near. The reading is lag_um
         * late: the crossing happened before. */
        double direction=(!!(x->sensors&bit))==!sensor?1:-1,at=s-lag;
        double phi[5]={1,at,at*at*.5,sensor?0:-direction,sensor?-direction:0};
        double z=p->threshold[sensor]-(p->path-yaw*lag);
        for(unsigned i=0;i<5;++i) {
            p->b[i]+=phi[i]*z;
            for(unsigned j=0;j<5;++j)p->a[i][j]+=phi[i]*phi[j];
        }
        if(!p->count)p->first=at;
        p->last=at;++p->count;
    }
    p->previous=x->sensors;p->started=1;
}
/* Information on the common unknowns left after removing start position and
 * heading (Schur complement of the 2x2 nuisance block). */
static int eliminate(const pass_fit_t *p,reduced_t *out)
{
    if(p->count<MIN_CROSSINGS || p->last-p->first<MIN_SPAN_MM)return -1;
    double det=p->a[0][0]*p->a[1][1]-p->a[0][1]*p->a[1][0];
    if(det<=1e-9*p->a[0][0]*p->a[1][1])return -1;
    double inv[2][2]={{p->a[1][1]/det,-p->a[0][1]/det},{-p->a[1][0]/det,p->a[0][0]/det}};
    for(unsigned i=0;i<PASS;++i) {
        double w[2];
        for(unsigned k=0;k<2;++k)w[k]=inv[k][0]*p->a[0][2+i]+inv[k][1]*p->a[1][2+i];
        for(unsigned j=0;j<PASS;++j)out->s[i][j]=p->a[2+i][2+j]-w[0]*p->a[0][2+j]-w[1]*p->a[1][2+j];
        out->r[i]=p->b[2+i]-w[0]*p->b[0]-w[1]*p->b[1];
    }
    return out->s[0][0]>0?0:-1;
}
/* The first n unknowns of a, b. */
static int solve(unsigned n,double a[GLOBAL][GLOBAL],double b[GLOBAL],double x[GLOBAL])
{
    double m[GLOBAL][GLOBAL+1];
    for(unsigned i=0;i<n;++i) {for(unsigned j=0;j<n;++j)m[i][j]=a[i][j];m[i][n]=b[i];}
    for(unsigned c=0;c<n;++c) {
        unsigned pivot=c;
        for(unsigned r=c+1;r<n;++r)if(fabs(m[r][c])>fabs(m[pivot][c]))pivot=r;
        if(fabs(m[pivot][c])<1e-12)return -1;
        for(unsigned j=0;j<=n;++j) {double t=m[c][j];m[c][j]=m[pivot][j];m[pivot][j]=t;}
        for(unsigned r=0;r<n;++r)if(r!=c) {
            double f=m[r][c]/m[c][c];
            for(unsigned j=c;j<=n;++j)m[r][j]-=f*m[c][j];
        }
    }
    for(unsigned i=0;i<n;++i)x[i]=m[i][n]/m[i][i];
    for(unsigned i=n;i<GLOBAL;++i)x[i]=0;
    return 0;
}
static fw_follow_diag_t last;
const fw_follow_diag_t *fw_follow_last(void) {return &last;}
int fw_follow_run(const fw_follow_io_t *io,const fw_cal_data_t *walls,unsigned cells,fw_follow_data_t *out)
{
    memset(&last,0,sizeof last);last.result=FW_CAL_GEOMETRY;
    if(!io || !io->pass || !io->u_turn || !out || !fw_cal_valid(walls) ||
       cells<FW_FOLLOW_MIN_CELLS || cells>FW_FOLLOW_MAX_CELLS)return FW_CAL_GEOMETRY;
    last.result=FW_CAL_MOTION;
    uint8_t raw;
    if(io->base.read(io->base.context,L10|R10|F5|F10,&raw))return FW_CAL_MOTION;
    /* Walls on both sides, the corridor open ahead. */
    if((raw&(L10|R10)) || (raw&(F5|F10))!=(F5|F10))return last.result=FW_CAL_WALLS;
    static reduced_t pass[FW_FOLLOW_PASSES];
    double information[GLOBAL][GLOBAL]={{0}},residual[GLOBAL]={0};unsigned fitted=0,crossings=0;
    for(unsigned i=0;i<FW_FOLLOW_PASSES;++i) {
        unsigned speed=fw_follow_pass_speed(i);char label[32];
        snprintf(label,sizeof label,"FOLLOW %u MM/S PASS %u/%u",speed,i+1,FW_FOLLOW_PASSES);
        if(io->base.status)io->base.status(io->base.context,label,0);
        pass_fit_t p;memset(&p,0,sizeof p);
        /* Lateral from the left wall of the robot, mid-bracket thresholds. */
        p.threshold[0]=(walls->side[0].near_um+walls->side[0].far_um)*.5;
        p.threshold[1]=walls->geometry.inner_um-(walls->side[1].near_um+walls->side[1].far_um)*.5;
        /* An even pass count ends in the start cell with the start heading. */
        if(io->pass(io->base.context,cells,speed,observe,&p) || io->u_turn(io->base.context,speed))return FW_CAL_MOTION;
        if(eliminate(&p,&pass[fitted]))continue;
        /* An even pass count ends in the start cell: even passes go out. */
        reduced_t *q=&pass[fitted];q->direction=i&1?-1:1;q->fast=speed>220;
        for(unsigned a=0;a<PASS;++a) {
            residual[a]+=q->r[a];
            for(unsigned b=0;b<PASS;++b)information[a][b]+=q->s[a][b];
            information[a][3]+=q->direction*q->s[a][0];information[3][a]+=q->direction*q->s[0][a];
        }
        information[3][3]+=q->s[0][0];residual[3]+=q->direction*q->r[0];
        if(q->direction>0)++last.out;else ++last.back;
        crossings+=p.count;++fitted;
    }
    last.crossings=crossings;last.result=FW_CAL_RANGE;
    double x[GLOBAL];
    unsigned unknowns=last.out>=2 && last.back>=2?GLOBAL:PASS;
    if(fitted<FW_FOLLOW_MIN_FITTED || solve(unknowns,information,residual,x))return FW_CAL_RANGE;
    /* Spread: each pass alone, with the common hysteresis and bend, weighted
     * by its information on the curvature as the fit is. A pass with few
     * crossings close together (often at 600 mm/s) gives a loose curvature:
     * unweighted, +/-1.5 mm of threshold noise read as 60-96 mrad/m of
     * disagreement while the bias was right within 3 (weighted: 8). */
    double weight=0,sum=0,square=0,speed_weight[2]={0},speed_sum[2]={0};
    for(unsigned i=0;i<fitted;++i) {
        const reduced_t *q=&pass[i];double w=q->s[0][0];
        double b=((q->r[0]-q->s[0][1]*x[1]-q->s[0][2]*x[2])/w-q->direction*x[3])*1000;
        weight+=w;sum+=w*b;square+=w*b*b;speed_weight[q->fast]+=w;speed_sum[q->fast]+=w*b;
    }
    double mean=sum/weight,variance=square/weight-mean*mean;
    fw_follow_data_t result={.valid=1,.cells=(uint32_t)cells,.geometry=walls->geometry,
        .yaw_bias_mrad_m=(int32_t)lround(x[0]*1000),
        .spread_mrad_m=(uint32_t)lround(variance>0?sqrt(variance):0),
        .passes=fitted,.crossings=crossings};
    double length=(cells-1)*(walls->geometry.pitch_um*1e-3);
    last.yaw_bias_mrad_m=result.yaw_bias_mrad_m;last.spread_mrad_m=result.spread_mrad_m;
    last.bend_um=(int32_t)lround(x[3]*length*length/8);
    for(unsigned f=0;f<2;++f)
        last.speed_mrad_m[f]=speed_weight[f]>0?(int32_t)lround(speed_sum[f]/speed_weight[f]):0;
    /* Passes that still disagree mean slip, or a corridor that is not even
     * close to an arc (a step between two wall pieces). */
    if(!fw_follow_valid(&result))return last.result=FW_CAL_UNSTABLE;
    last.result=0;*out=result;return 0;
}
