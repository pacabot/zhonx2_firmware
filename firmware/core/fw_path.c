#include "fw_path.h"
#include <math.h>
#include <string.h>
#define PI 3.14159265359f
#define SAMPLES 64
static float arc_s[SAMPLES+1],arc_x[SAMPLES+1],arc_y[SAMPLES+1];
static float arc_heading[SAMPLES+1],arc_curvature[SAMPLES+1];
static float arc_length,max_curvature;
static void curve_table(void)
{
    if(arc_length)return;
    /* Quintic Bezier: collinear first/last three control points give zero
     * curvature at both joins. Unlike a circular arc, wheel ratios do not jump. */
    const float x[]={0,0,0,.4f,.7f,1},y[]={-1,-.7f,-.4f,0,0,0};
    for(unsigned i=0;i<=SAMPLES;++i) {
        float t=(float)i/SAMPLES,u=1-t,a[6],b[6];memcpy(a,x,sizeof a);memcpy(b,y,sizeof b);
        float dx=0,dy=0,ddx=0,ddy=0;
        for(unsigned n=5;n>0;--n) {
            if(n==2) {ddx=20*(a[2]-2*a[1]+a[0]);ddy=20*(b[2]-2*b[1]+b[0]);}
            if(n==1) {dx=5*(a[1]-a[0]);dy=5*(b[1]-b[0]);}
            for(unsigned j=0;j<n;++j) {a[j]=u*a[j]+t*a[j+1];b[j]=u*b[j]+t*b[j+1];}
        }
        arc_x[i]=a[0];arc_y[i]=b[0];arc_heading[i]=atan2f(dx,dy);
        float v2=dx*dx+dy*dy;
        arc_curvature[i]=(dy*ddx-dx*ddy)/(v2*sqrtf(v2));
        if(arc_curvature[i]>max_curvature)max_curvature=arc_curvature[i];
        if(i)arc_s[i]=arc_s[i-1]+hypotf(arc_x[i]-arc_x[i-1],arc_y[i]-arc_y[i-1]);
    }
    arc_length=arc_s[SAMPLES];
}
unsigned fw_run_acceleration(unsigned speed)
{
    if(speed<20)speed=20;
    if(speed>1000)speed=1000;
    return 600+2*speed; /* 840 at 120 mm/s, 1800 at 600, 2600 at 1000. */
}
static int fits(const fw_cal_geometry_t *g,float radius)
{
    float body=fmaxf(g->width_um*.5f,(float)g->nose_um),inside=g->inner_um*.5f-3000;
    for(unsigned i=0;i<=SAMPLES;++i)for(int a=-1;a<=1;a+=2)for(int b=-1;b<=1;b+=2) {
        float h=arc_heading[i],x=arc_x[i]*radius+body*(a*cosf(h)+b*sinf(h));
        float y=arc_y[i]*radius+body*(-a*sinf(h)+b*cosf(h));
        if(x< -inside || y>inside || (x>inside && y< -inside))return 0;
    }
    return 1;
}
static void offset(float *x,float *y,unsigned heading,float right,float forward)
{
    static const int dx[]={0,1,0,-1},dy[]={1,0,-1,0};
    *x+=forward*dx[heading]+right*dx[(heading+1)%4];
    *y+=forward*dy[heading]+right*dy[(heading+1)%4];
}
int fw_path_plan(fw_path_t *p,const nm_map_t *m,nm_pose_t pose,const nm_route_t *route,
                 const fw_cal_geometry_t *g,unsigned speed)
{
    memset(p,0,sizeof *p);
    if(!nm_valid(m) || !fw_cal_geometry_valid(g) || !route->length || route->length>NM_CELLS ||
       speed<20 || speed>1000 || route->direction[0]!=pose.heading)return -1;
    curve_table();p->radius=g->pitch_um*.5f;p->acceleration=(float)fw_run_acceleration(speed);
    nm_pose_t end=pose;
    for(unsigned i=0;i<route->length;++i) {
        unsigned d=route->direction[i];int c=end.y*NM_SIDE+end.x,n=nm_next(m,c,d);
        if(d>3 || n<0 || !(m->cell[c].known&(1u<<d)) || (m->cell[c].walls&(1u<<d)))return -1;
        if(i && d!=route->direction[i-1] && ((d+4-route->direction[i-1])%2)==0)return -1;
        end=(nm_pose_t){n%NM_SIDE,n/NM_SIDE,d};
    }
    float x=pose.x*(float)g->pitch_um,y=pose.y*(float)g->pitch_um;
    unsigned at=0;
    while(at<route->length) {
        unsigned h=route->direction[at],next=at+1;
        while(next<route->length && route->direction[next]==h)++next;
        float length=(next-at)*(float)g->pitch_um-(at?p->radius:0)-(next<route->length?p->radius:0);
        if(length>0) {
            p->segment[p->count++]=(fw_path_segment_t){p->length,length,x,y,(float)speed,0,h,0,at?(uint32_t)p->radius:0};
            p->length+=length;offset(&x,&y,h,0,length);
        }
        if(next<route->length) {
            if(!fits(g,p->radius))return -1;
            int dir=(route->direction[next]+4-h)%4==1?1:-1;
            float cap=sqrtf(p->acceleration*.5f*(p->radius*.001f)/max_curvature);
            if(cap>300)cap=300;
            if(cap>speed)cap=(float)speed;
            length=arc_length*p->radius;
            p->segment[p->count++]=(fw_path_segment_t){p->length,length,x,y,cap,0,h,dir,0};
            p->length+=length;offset(&x,&y,h,dir*p->radius,p->radius);
        }
        at=next;
    }
    float next_speed=40;
    for(unsigned i=p->count;i--;) {
        fw_path_segment_t *s=&p->segment[i];s->exit_speed=fminf(next_speed,s->limit);
        next_speed=fminf(s->limit,sqrtf(s->exit_speed*s->exit_speed+2*p->acceleration*s->length*.001f));
    }
    p->end=end;return 0;
}
void fw_path_point(const fw_path_t *p,unsigned index,float distance,fw_path_point_t *out)
{
    const fw_path_segment_t *s=&p->segment[index];
    float d=fmaxf(0,fminf(s->length,distance));
    *out=(fw_path_point_t){s->x,s->y,s->heading*PI/2,0};
    if(!s->turn) {offset(&out->x,&out->y,s->heading,0,d);return;}
    float u=d/p->radius;unsigned i=1;
    while(i<SAMPLES && arc_s[i]<u)++i;
    float f=(u-arc_s[i-1])/(arc_s[i]-arc_s[i-1]);
    float x=arc_x[i-1]+f*(arc_x[i]-arc_x[i-1]),y=arc_y[i-1]+f*(arc_y[i]-arc_y[i-1]);
    offset(&out->x,&out->y,s->heading,s->turn*x*p->radius,(y+1)*p->radius);
    out->heading+=s->turn*(arc_heading[i-1]+f*(arc_heading[i]-arc_heading[i-1]));
    out->curvature=s->turn*(arc_curvature[i-1]+f*(arc_curvature[i]-arc_curvature[i-1]))/p->radius;
}
