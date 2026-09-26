#ifndef FW_CALIBRATION_H
#define FW_CALIBRATION_H
#include <stdint.h>
/* All distances are from the wheel axle to the wall face, in micrometres. */
typedef struct {
    uint32_t nose_um, width_um, inner_um, pitch_um;
} fw_cal_geometry_t;
typedef struct { uint32_t on_um, off_um, spread_um; } fw_cal_front_t;
typedef struct { uint32_t near_um, far_um, spread_um; } fw_cal_side_t;
typedef struct {
    uint32_t valid, repetitions;
    fw_cal_geometry_t geometry;
    fw_cal_front_t front[2]; /* F5, F10: continuous approach/recession thresholds. */
    fw_cal_side_t side[2]; /* L5, R5: static 1 mm brackets, not optical hysteresis. */
} fw_cal_data_t;
typedef void (*fw_cal_observer)(void *, int32_t travelled_um, uint8_t raw, uint8_t filtered);
typedef struct {
    void *context;
    int (*move)(void *, int32_t um, unsigned mm_s, fw_cal_observer, void *);
    int (*turn)(void *, int degrees);
    /* Fresh samples; only stable_mask bits must remain stable. Zero: settle only. */
    int (*read)(void *, uint8_t stable_mask, uint8_t *raw);
    void (*status)(void *, const char *text, int32_t distance_um);
} fw_cal_io_t;
enum { FW_CAL_OK=0, FW_CAL_GEOMETRY=-1, FW_CAL_WALLS=-2,
       FW_CAL_MOTION=-3, FW_CAL_RANGE=-4, FW_CAL_UNSTABLE=-5 };
int fw_cal_geometry_valid(const fw_cal_geometry_t *);
int fw_cal_valid(const fw_cal_data_t *);
int fw_cal_run(const fw_cal_io_t *, const fw_cal_geometry_t *, fw_cal_data_t *);
/* Establish both coordinates by seating on the selected side, then the front.
 * side: 0=left, 1=right; the caller verifies the required walls before movement. */
int fw_cal_reference(const fw_cal_io_t *, const fw_cal_geometry_t *, unsigned side);
#endif
