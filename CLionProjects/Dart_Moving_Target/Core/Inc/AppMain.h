//
// Created by zhouzhi on 2026/1/20.
//

#ifndef APPMAIN_H
#define APPMAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

    void CAN_Send(int16_t c1, int16_t c2, int16_t c3);
    void App_Init(void);
    void App_Task_1ms(void);
    void App_CAN_Callback(uint32_t std_id, uint8_t* data);
    void new_App_Init(void);
    void new_App_Task_1ms(void);
    void CAN_TX_1();
    void CAN_TX_2();
    void set_random_position();
    void move_random_position();
    void new_App_CAN_Callback(uint32_t std_id, uint8_t* data);

#ifdef __cplusplus
}
#endif

#endif //APPMAIN_H
