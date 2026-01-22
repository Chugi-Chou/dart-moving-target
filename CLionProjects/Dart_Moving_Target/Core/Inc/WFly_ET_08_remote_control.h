//
// Created by zhouzhi on 2026/1/20.
//

#ifndef WFLY_ET_08_REMOTE_CONTROL_H
#define WFLY_ET_08_REMOTE_CONTROL_H

#include <cstdint>
#include "stm32f4xx_hal.h"

typedef struct {
    uint16_t ch0;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint8_t sa;
    uint8_t sb;
    uint8_t sc;
    uint8_t sd;
}rc_ctrl;

enum et_status{up, mid, down};

class et_08 {
private:
    rc_ctrl ctrl;
    uint32_t currtick;
    et_status s_status_alph[4];
    float left_hori_status;
    float left_vert_status;
    float right_hori_status;
    float right_vert_status;
public:
    bool isConnected();
    bool process_rc_frame(uint8_t *frame, uint8_t size);
    float getLeftHori() { return left_hori_status;}
    uint8_t getsa() { return ctrl.sa;}
    uint8_t getsb() { return ctrl.sb;}
    uint8_t getsc() { return ctrl.sc;}
    uint8_t getsd() { return ctrl.sd;}
};


#endif //WFLY_ET_08_REMOTE_CONTROL_H
