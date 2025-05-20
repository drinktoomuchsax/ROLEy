#ifndef __GMR_ENCODER_H
#define __GMR_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* private defines -----------------------------------------------------------*/
#define gmr_encoder_lines                500u    /* 编码器线数 */
#define md60_reduction_ratio             18u    /* 减速比 */
#define gmr_encoder_cpr                  (gmr_encoder_lines * md60_reduction_ratio * 2u)  /* 每转计数 = 线数 * 4 (四倍频) * 减速比 */

/* exported types ------------------------------------------------------------*/
typedef enum {
    ENCODER_1 = 0,
    ENCODER_2 = 1
} encoder_id_t;

typedef enum {
    ENCODER_A_PHASE = 0,
    ENCODER_B_PHASE = 1
} encoder_phase_t;

typedef struct
{
    int32_t count;          /* 当前计数值 */
    int32_t last_count;     /* 上一次计数值 */
    int32_t overflow;       /* 溢出计数 */
    float speed;            /* 当前速度 (度每秒) */
    float position;         /* 当前位置 (度)，0~360 */
    uint8_t last_state;     /* 上一次ab相状态，00为0度，01为90度，10为180度，11为270度 */
} gmr_encoder_t;

/* exported functions prototypes ---------------------------------------------*/
void gmr_encoder_init(void);
void gmr_encoder_update(encoder_id_t id);
float gmr_encoder_get_speed(encoder_id_t id);
float gmr_encoder_get_position(encoder_id_t id);
int32_t gmr_encoder_get_count(encoder_id_t id);
void gmr_encoder_reset(encoder_id_t id);
uint32_t gmr_encoder_get_pulse_count(encoder_id_t id);
uint8_t gmr_encoder_get_phase(encoder_id_t id, encoder_phase_t phase);

#ifdef __cplusplus
}
#endif

#endif /* __GMR_ENCODER_H */ 