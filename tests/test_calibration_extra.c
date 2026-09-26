#include "fw_cal_extra.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define PI 3.14159265358979323846
static const fw_cal_geometry_t geometry={47000,94000,167000,179000};
typedef struct {
    double x,y,angle;
    unsigned moves,turns,spins,frames;
    int fixture,stuck,cancel_spin,noise,blind;
    uint8_t raw,filtered,confidence[6];
} robot_t;
static double nearest_heading(double angle)
{ return round(angle/(PI/2))*(PI/2); }
static double ray(robot_t *r,double bearing,double forward,double lateral,unsigned bit)
{
    double a=r->angle,b=a+bearing,dx=sin(b),dy=-cos(b);
    double x=r->x+sin(a)*forward+cos(a)*lateral;
    double y=r->y-cos(a)*forward+sin(a)*lateral;
    double limit=r->fixture<0?167000:179000+((r->raw&bit)?-1000:1000);
    double best=1e9,t;
    if(dy<-.00001) { t=-y/dy;double xx=x+t*dx;if(t>=0 && xx>=0 && xx<=167000) best=t; }
    if(dx<-.00001 && r->fixture!=1) {
        t=-x/dx;double yy=y+t*dy;if(t>=0 && yy>=0 && yy<=limit && t<best) best=t;
    }
    if(dx>.00001 && r->fixture!=0) {
        t=(167000-x)/dx;double yy=y+t*dy;if(t>=0 && yy>=0 && yy<=limit && t<best) best=t;
    }
    return best;
}
static void sensors(robot_t *r)
{
    const unsigned bits[]={4,8,16,32,2,1};
    const double angles[]={0,0,-PI/2,-PI/2,PI/2,PI/2};
    const double forward[]={30000,34000,25000,28000,18000,22000};
    const double lateral[]={3000,-2000,-47000,-47000,47000,47000};
    const double on[]={65000,115000,50000,105000,52000,105000};
    const double off[]={68000,118000,53000,108000,55000,108000};
    for(unsigned i=0;i<6;++i) {
        double distance=ray(r,angles[i],forward[i],lateral[i],bits[i]);
        if(distance<=on[i]) r->raw &= ~bits[i];
        if(distance>off[i]) r->raw |= bits[i];
        if(r->blind && i<2 && distance<20000) r->raw |= bits[i];
    }
    if(r->stuck) r->raw &= ~8u;
    uint8_t sample=r->raw;
    if(r->noise && ++r->frames%89==0) sample ^= 8;
    for(unsigned i=0;i<6;++i) {
        if(sample&(1u<<i)) { if(r->confidence[i]<3) ++r->confidence[i]; }
        else if(r->confidence[i]) --r->confidence[i];
        if(r->confidence[i]==3) r->filtered |= 1u<<i;
        if(!r->confidence[i]) r->filtered &= ~(1u<<i);
    }
}
static int read_robot(void *context,uint8_t stable_mask,uint8_t *raw)
{
    (void)stable_mask; robot_t *r=context;
    for(unsigned i=0;i<5;++i) sensors(r);
    *raw=r->raw; return 0;
}
static int move_robot(void *context,int32_t um,unsigned speed,fw_cal_observer observer,void *data)
{
    robot_t *r=context; ++r->moves;
    assert(um && um>=-300000 && um<=300000);
    /* Contact self-squares the robot. Reference turns are exact in this model;
     * optical spin profiles below intentionally use a wrong nominal track. */
    assert(fabs(remainder(r->angle-nearest_heading(r->angle),2*PI))<.18);
    double heading=nearest_heading(r->angle);
    int32_t total=um<0?-um:um,sign=um<0?-1:1,travel=0;
    while(travel<total) {
        int32_t step=(int32_t)speed*10;if(step>total-travel) step=total-travel;
        travel+=step;r->x+=sin(heading)*sign*step;r->y-=cos(heading)*sign*step;
        int contact=0;
        if(r->y<47000) { r->y=47000;contact=1; }
        if(r->x<47000 && r->fixture!=1) { r->x=47000;contact=1; }
        if(r->x>120000 && r->fixture!=0) { r->x=120000;contact=1; }
        if(contact) r->angle=heading;
        sensors(r);
        if(observer) observer(data,sign*travel,r->raw,r->filtered);
    }
    for(unsigned i=0;i<5;++i) { sensors(r);if(observer) observer(data,um,r->raw,r->filtered); }
    return 0;
}
static int turn_robot(void *context,int degrees)
{
    robot_t *r=context; ++r->turns;
    assert(degrees==90 || degrees==-90 || degrees==180 || degrees==-180);
    assert(r->x>69000 && r->x<98000 && r->y>69000);
    r->angle+=degrees*PI/180.;
    for(unsigned i=0;i<5;++i) sensors(r);
    return 0;
}
static double actual_track(unsigned speed,unsigned direction,int stopped)
{ return (direction?82000:84000)+speed*15+(stopped?500:0); }
static int spin_robot(void *context,int32_t um,unsigned speed,fw_cal_observer observer,void *data)
{
    robot_t *r=context;
    if((int)++r->spins==r->cancel_spin) return -1;
    assert(speed==40 || speed==80 || speed==120);
    int32_t total=um<0?-um:um,sign=um<0?-1:1,travel=0;
    double track=actual_track(speed,um<0,total<100000);
    while(travel<total) {
        int32_t step=(int32_t)speed*10;if(step>total-travel) step=total-travel;
        travel+=step;r->angle+=sign*2.*step/track;
        sensors(r);
        if(observer) observer(data,sign*travel,r->raw,r->filtered);
    }
    for(unsigned i=0;i<5;++i) { sensors(r);if(observer) observer(data,um,r->raw,r->filtered); }
    return 0;
}
static robot_t initial(int fixture)
{
    robot_t r={.x=83500,.y=83500,.fixture=fixture,.raw=0x3f,.filtered=0x3f};
    memset(r.confidence,3,sizeof r.confidence); for(unsigned i=0;i<5;++i) sensors(&r); return r;
}
int main(void)
{
    robot_t r=initial(-1);
    fw_cal_extra_io_t io={{&r,move_robot,turn_robot,read_robot,0},spin_robot};
    fw_rotation_data_t rotation={0};
    int result=fw_rotation_run(&io,&geometry,&rotation);
    if(result) fprintf(stderr,"rotation result %d at spin %u angle %.3f\n",result,r.spins,remainder(r.angle,2*PI));
    assert(!result && fw_rotation_valid(&rotation));
    for(unsigned v=0;v<FW_CAL_SPEEDS;++v) for(unsigned dir=0;dir<2;++dir) {
        double target=actual_track(rotation.point[v].speed,dir,1)*PI/4;
        assert(fabs(rotation.point[v].quarter_um[dir]-target)<700);
        assert(rotation.point[v].quarter_checks[dir]>=4);
        printf("rotation: %u mm/s %s quarter %.2f mm (true %.2f), optical error %u mdeg\n",
          rotation.point[v].speed,dir?"CCW":"CW",rotation.point[v].quarter_um[dir]/1000.,target/1000.,rotation.point[v].quarter_error_mdeg[dir]);
    }
    assert(r.x==83500 && r.y==83500);
    assert(fw_rotation_quarter(&rotation,60,0)>rotation.point[0].quarter_um[0]);
    assert(!fw_rotation_quarter(&rotation,121,0));
    fw_rotation_data_t previous=rotation;
    r=initial(-1); r.blind=1;
    assert(!fw_rotation_run(&io,&geometry,&rotation) && fw_rotation_valid(&rotation));
    puts("rotation: reference seating tolerates near-contact blind zones");
    r=initial(-1);r.stuck=1;
    assert(fw_rotation_run(&io,&geometry,&rotation)==FW_CAL_RANGE && !memcmp(&rotation,&previous,sizeof rotation));
    r=initial(-1);r.cancel_spin=4;
    assert(fw_rotation_run(&io,&geometry,&rotation)==FW_CAL_MOTION && !memcmp(&rotation,&previous,sizeof rotation));
    unsigned left_moves=0,left_turns=0;
    for(unsigned side=0;side<2;++side) {
        r=initial((int)side); fw_corner_data_t corner={0};
        result=fw_corner_run(&io.base,&geometry,side,173000,&corner);
        if(result) fprintf(stderr,"corner result %d moves %u at %.1f %.1f heading %.3f\n",result,r.moves,r.x,r.y,r.angle);
        assert(!result && fw_corner_valid(&corner));
        for(unsigned facing=0;facing<2;++facing) for(unsigned v=0;v<FW_CAL_SPEEDS;++v) {
            const fw_corner_point_t *p=&corner.point[facing][v]; assert(p->mask==3);
            for(unsigned s=0;s<2;++s) {
                assert(p->open_um[s]>p->raw_open_um[s]);
                assert(p->close_um[s]<p->raw_close_um[s]);
                assert(facing?p->raw_open_um[s]<0:p->raw_open_um[s]>0);
            }
        }
        int32_t offset;
        assert(!fw_corner_offset(&corner,1,160,1,1,&offset) && offset<0);
        assert(fw_corner_offset(&corner,1,260,1,1,&offset));
        assert(r.x==83500 && r.y==83500);
        if(!side) {left_moves=r.moves;left_turns=r.turns;}
        else assert(r.moves==left_moves && r.turns==left_turns);
        printf("corner: %s fixture, 3 speeds, both orientations, raw and filtered offsets, signed forward reference\n",side?"right":"left");
    }
    puts("extra calibration: stopped-turn refinement, directional tracks, missing optical reference and cancellation");
    return 0;
}
