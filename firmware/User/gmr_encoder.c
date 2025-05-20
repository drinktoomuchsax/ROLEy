/* includes ------------------------------------------------------------------*/
#include "gmr_encoder.h"
#include "main.h"

/* private typedef -----------------------------------------------------------*/

/* private defines -----------------------------------------------------------*/

/* private macros ------------------------------------------------------------*/

/* private variables ---------------------------------------------------------*/
gmr_encoder_t encoders[2] = {0};

/* private function prototypes -----------------------------------------------*/

/* private constants ---------------------------------------------------------*/
/**
 * 正转
 * 00 0
 * 01 1
 * 11 3
 * 10 2
 * 00 0
 * 01 1
 * 11 3
 * 10 2
 * 00 0
 * 
 * 
 * 反过来
 * 00 0
 * 10 2
 * 11 3
 * 01 1
 * 00 0
 * 10 2
 * 11 3
 * 01 1
 * 00 0
 */

/**
 * 正转
 * 0111 7
 * 1110 14
 * 1000 8
 * 0001 1
 * 
 * 反转
 * 0010 2
 * 1011 11
 * 1101 13
 * 0100 4
 */
// 最后得到状态转移矩阵
static const int8_t state_table[16] = {
    0,  1, -1,  0,
   -1,  0,  0,  1,
    1,  0,  0, -1,
    0, -1,  1,  0
};

/* private functions ---------------------------------------------------------*/
static void update_encoder_state(gmr_encoder_t* encoder, GPIO_TypeDef* port_a, uint16_t pin_a, 
                               GPIO_TypeDef* port_b, uint16_t pin_b)
{
    uint8_t current_state;
    uint8_t state_change;
    
    /* 读取当前ab相状态 */
    current_state = (HAL_GPIO_ReadPin(port_a, pin_a) << 1) |
                   (HAL_GPIO_ReadPin(port_b, pin_b));
    
    /* 计算状态变化 */
    state_change = (encoder->last_state << 2) | current_state;
    
    /* 更新计数值 */
    encoder->count += state_table[state_change];
    
    /* 直接取余之后换算成角度 */
    uint32_t one_lap_count = 2 * gmr_encoder_cpr;
    encoder->position = ((float)(encoder->count % one_lap_count) / one_lap_count) * 360.0f;
    /* 保存当前状态 */
    encoder->last_state = current_state;
}

/* exported functions --------------------------------------------------------*/

/**
  * @brief  初始化编码器
  * @param  none
  * @retval none
  */
void gmr_encoder_init(void)
{
    for (int i = 0; i < 2; i++)
    {
        encoders[i].count = 0;
        encoders[i].last_count = 0;
        encoders[i].overflow = 0;
        encoders[i].speed = 0.0f;
        encoders[i].position = 0.0f;
        encoders[i].last_state = 0;
    }
}

/**
  * @brief  更新编码器状态
  * @param  id: 编码器id
  * @retval none
  */
void gmr_encoder_update(encoder_id_t id)
{
    if (id == ENCODER_1)
    {
        update_encoder_state(&encoders[id], ENCODER_A1_GPIO_Port, ENCODER_A1_Pin,
                           ENCODER_B1_GPIO_Port, ENCODER_B1_Pin);
    }
    else if (id == ENCODER_2)
    {
        update_encoder_state(&encoders[id], ENCODER_A2_GPIO_Port, ENCODER_A2_Pin,
                           ENCODER_B2_GPIO_Port, ENCODER_B2_Pin);
    }
}

/**
  * @brief  获取编码器速度
  * @param  id: 编码器id
  * @retval 速度值 (rpm)
  */
float gmr_encoder_get_speed(encoder_id_t id)
{
    return encoders[id].speed;
}

/**
  * @brief  获取编码器位置
  * @param  id: 编码器id
  * @retval 位置值 (度)
  */
float gmr_encoder_get_position(encoder_id_t id)
{
    return encoders[id].position;
}

/**
  * @brief  获取编码器计数值
  * @param  id: 编码器id
  * @retval 计数值
  */
int32_t gmr_encoder_get_count(encoder_id_t id)
{
    return encoders[id].count;
}

/**
  * @brief  重置编码器
  * @param  id: 编码器id
  * @retval none
  */
void gmr_encoder_reset(encoder_id_t id)
{
    encoders[id].count = 0;
    encoders[id].last_count = 0;
    encoders[id].overflow = 0;
    encoders[id].speed = 0.0f;
    encoders[id].position = 0.0f;
}
