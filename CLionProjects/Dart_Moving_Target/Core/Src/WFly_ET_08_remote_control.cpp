//
// Created by zhouzhi on 2026/1/20.
//

#include "WFly_ET_08_remote_control.h"

bool et_08::isConnected() {
    bool temp = false;
    if (HAL_GetTick() == currtick + 1) temp = true;
    currtick = HAL_GetTick();
    return temp;
}

bool et_08::process_rc_frame(uint8_t *frame, uint8_t size) {
    if (size < 25 || frame[0] != 0x0F) return false;

    uint16_t temp_ch[16];
    const uint16_t ch_offset = 1024;

    for (uint8_t i = 0; i < 16; i++) {
        const uint16_t bit = i * 11;
        const uint8_t start_byte = bit / 8 + 1;
        const uint8_t start_bit = bit % 8;

        if (start_bit < 5) {
            temp_ch[i] = ((frame[start_byte] >> start_bit) |
                          (frame[start_byte + 1] << (8 - start_bit))) & 0x07FF;
        } else {
            temp_ch[i] = ((frame[start_byte] >> start_bit) |
                          (frame[start_byte + 1] << (8 - start_bit)) |
                          (frame[start_byte + 2] << (16 - start_bit))) & 0x07FF;
        }
    }

    ctrl.ch0 = temp_ch[0];
    ctrl.ch1 = temp_ch[1];
    ctrl.ch2 = temp_ch[2];
    ctrl.ch3 = temp_ch[3];

    auto map_tri_switch = [](uint16_t val) -> et_status {
        if (val < 600) return et_status::up;
        if (val> 1400) return et_status::down;
        return et_status::mid;
    };

    auto map_dual_switch = [](uint16_t val) -> et_status {
        return (val > 1024) ? et_status::down : et_status::up;
    };

    ctrl.sb = map_tri_switch(temp_ch[4]);
    ctrl.sc = map_tri_switch(temp_ch[5]);
    ctrl.sa = map_dual_switch(temp_ch[6]);
    ctrl.sd = map_dual_switch(temp_ch[7]);

    const float range = 660.0f;
    const int16_t mid_pos = 1024;

    right_hori_status = (static_cast<float>(temp_ch[0]) - mid_pos) / range;
    right_vert_status = (static_cast<float>(temp_ch[1]) - mid_pos) / range;
    left_vert_status  = (static_cast<float>(temp_ch[2]) - mid_pos) / range;
    left_hori_status  = (static_cast<float>(temp_ch[3]) - mid_pos) / range;
    
    auto constrain = [](float &val) {
        if (val > 1.0f) val = 1.0f;
        if (val < -1.0f) val = -1.0f;
    };
    constrain(right_hori_status);
    constrain(right_vert_status);
    constrain(left_vert_status);
    constrain(left_hori_status);

    currtick = HAL_GetTick();

    return true;
}
