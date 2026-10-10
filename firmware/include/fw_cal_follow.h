#ifndef FW_CAL_FOLLOW_H
#define FW_CAL_FOLLOW_H
#include "fw_calibration.h"
/* Wheel yaw bias of the wall follower (its integral term). A wheel diameter
 * difference turns a commanded straight into an arc that the follower learns
 * only after metres of tracking cycles; each exploration and run restarts
 * that learning. The calibration drives exploration-like out-and-back passes
 * in a straight corridor walled on both sides and closed at both ends. Each
 * 5 cm threshold crossing pins the lateral position; between crossings the
 * step counts give the path. The bias bends that path into a parabola
 * (29 mm over 70 cm for 1 %): one curvature common to all passes is fitted,
 * with an unknown start position and heading per pass. The result preloads
 * every motion session. */
#define FW_FOLLOW_PASSES 16u      /* 8 at 220 mm/s then 8 at 600 mm/s */
#define FW_FOLLOW_MIN_FITTED 6u   /* passes with enough crossings */
#define FW_FOLLOW_MIN_CELLS 4
#define FW_FOLLOW_MAX_CELLS 16
#define FW_FOLLOW_DEFAULT_CELLS 5
#define FW_FOLLOW_BIAS_LIMIT 300  /* mrad/m, the follower's own learning bound */
#define FW_FOLLOW_SPREAD_LIMIT 60 /* mrad/m between passes, weighted by their information */
typedef struct {
    uint32_t valid, cells;
    fw_cal_geometry_t geometry;
    int32_t yaw_bias_mrad_m;      /* follower convention: true = step yaw + bias * travel */
    uint32_t spread_mrad_m, passes, crossings;
} fw_follow_data_t;
/* One per IR scan during a pass, from the pass start. */
typedef struct {
    int32_t travel_um, yaw_urad;  /* mean wheel travel; step yaw (dl - dr) / track */
    uint32_t lag_um;              /* sensor delay of this scan, as the follower uses */
    uint8_t sensors;              /* active-low bits seen by the follower */
} fw_follow_sample_t;
typedef void (*fw_follow_observer)(void *, const fw_follow_sample_t *);
typedef struct {
    fw_cal_io_t base;
    /* Centre to centre along the corridor with the follower, ending on the
     * far front wall; then a 180-degree pivot at the travel speed. */
    int (*pass)(void *, unsigned cells, unsigned speed, fw_follow_observer, void *);
    int (*u_turn)(void *, unsigned speed);
} fw_follow_io_t;
/* What the last run measured, kept even when it was rejected. */
typedef struct {
    int result;                   /* 0 saved, else FW_CAL_* */
    int32_t yaw_bias_mrad_m;
    int32_t speed_mrad_m[2];      /* weighted mean of the 220 and 600 mm/s passes */
    int32_t bend_um;              /* corridor sag over the pass length; + bends left going out */
    uint32_t spread_mrad_m, out, back, crossings; /* passes fitted per direction */
} fw_follow_diag_t;
const fw_follow_diag_t *fw_follow_last(void);
unsigned fw_follow_pass_speed(unsigned pass);
int fw_follow_valid(const fw_follow_data_t *);
int fw_follow_run(const fw_follow_io_t *, const fw_cal_data_t *walls, unsigned cells, fw_follow_data_t *);
#endif
