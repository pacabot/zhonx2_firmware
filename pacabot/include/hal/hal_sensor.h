/*---------------------------------------------------------------------------
 *
 *      hal_sensor.h
 *
 *---------------------------------------------------------------------------*/

#ifndef __HAL_SENSOR_H__
#define __HAL_SENSOR_H__
#include <stdint.h>
typedef struct { uint32_t timestamp, sequence; uint8_t raw, filtered; } hal_sensor_snapshot;
int hal_sensor_snapshot_read(hal_sensor_snapshot *out);


/* Module Identifier */
#define HAL_SENSOR_MODULE_ID    10

#define SENSOR_R5_POS        0x02
#define SENSOR_F5_POS        0x04
#define SENSOR_L5_POS        0x10
#define SENSOR_R10_POS       0x01
#define SENSOR_F10_POS       0x08
#define SENSOR_L10_POS       0x20

/* Pin definitions */
#define SENSOR_R10_PIN      (GPIO_Pin_0) // Right 10cm IR sensor pin
#define SENSOR_R5_PIN       (GPIO_Pin_1) // Right 5cm IR sensor pin
#define SENSOR_F5_PIN       (GPIO_Pin_2) // Front 5cm IR sensor pin
#define SENSOR_F10_PIN      (GPIO_Pin_3) // Front 10cm IR sensor pin
#define SENSOR_L5_PIN       (GPIO_Pin_4) // Left 5cm IR sensor pin
#define SENSOR_L10_PIN      (GPIO_Pin_5) // Left 10cm IR sensor pin

#define SENSOR_CR_PIN       (GPIO_Pin_0) // Right floor sensor pin
#define SENSOR_CL_PIN       (GPIO_Pin_1) // Left floor sensor pin

#define SENSOR_EN_5_PIN     (GPIO_Pin_6) // Enable pin for 10cm sensors
#define SENSOR_EN_10_PIN    (GPIO_Pin_7) // Enable pin for 5cm sensors

#define SENSORS_5CM         (SENSOR_R5_PIN | SENSOR_F5_PIN | SENSOR_L5_PIN)
#define SENSORS_10CM        (SENSOR_R10_PIN | SENSOR_F10_PIN | SENSOR_L10_PIN)

/* Error codes */
#define HAL_SENSOR_E_SUCCESS    0
#define HAL_SENSOR_E_ERROR      MAKE_ERROR(HAL_SENSOR_MODULE_ID, 1)
#define HAL_SENSOR_E_BAD_HANDLE MAKE_ERROR(HAL_SENSOR_MODULE_ID, 2)

/* HAL SENSOR handle */
typedef void *HAL_SENSOR_HANDLE;

/* Exported functions */
int hal_sensor_init(void);
int hal_sensor_terminate(void);
int hal_sensor_open(HAL_SENSOR_HANDLE *handle, void *params);
int hal_sensor_close(HAL_SENSOR_HANDLE handle);
int hal_sensor_enable(void);
int hal_sensor_disable(void);
unsigned char hal_sensor_get_state(HAL_SENSOR_HANDLE handle);
int hal_sensor_get_color(HAL_SENSOR_HANDLE handle);

#endif // __HAL_SENSOR_H__
