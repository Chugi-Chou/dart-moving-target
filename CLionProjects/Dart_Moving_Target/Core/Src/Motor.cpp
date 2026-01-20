//
// Created by zhouzhi on 2026/1/20.
//

#include "../Inc/Motor.h"

#include "PID.h"
#include "main.h"

Motor::Motor(const PID& pos_pid, const PID& speed_pid, MotorType motor_selection, ControlMode_e ctrl_selection)
    : pos_controller(pos_pid), speed_controller(speed_pid), motor_type(motor_selection), ctrl_mode(ctrl_selection),
      target_value(0), initialized(false) {
    dir = CLOCKWISE;
    switch (motor_type) {
        case M2006: reduction_ratio = 36.0f; break;
        case M3508: reduction_ratio = 3591.0f/187.0f; break;
    }
}

void Motor::UpdateFeedback(uint8_t data[8]) {
    int16_t raw_angle = (data[0] << 8) | data[1];
    current_speed = (float)((int16_t)((data[2] << 8) | data[3]));

    if (!initialized) {
        last_raw_angle = raw_angle;
        initialized = true;
        return;
    }
    if (raw_angle - last_raw_angle > 4096) rounds--;
    else if (raw_angle - last_raw_angle < -4096) rounds++;
    last_raw_angle = raw_angle;
    total_angle = ((float)rounds * 360.0f + (float)raw_angle * 360.0f / 8192.0f) / reduction_ratio;
}

void Motor::SetTarget(ControlMode_e mode, float val) {
    ctrl_mode = mode;

    if (mode == POSITION_MODE) {
        if (dir == CLOCKWISE) {
            while (val < total_angle) {
                val += 360.0f;
            }
        } else if (dir == COUNTERCLOCKWISE) {
            while (val > total_angle) {
                val -= 360.0f;
            }
        }
    }

    target_value = val;
}

int16_t Motor::ExecuteControl() {
    float final_current = 0;

    switch (ctrl_mode) {
        case POSITION_MODE: {
            float speed_target = pos_controller.Calculate(target_value, total_angle);

            if (dir == CLOCKWISE) {
                if (speed_target < 0) speed_target = 0;
            } else {
                if (speed_target > 0) speed_target = 0;
            }

            final_current = speed_controller.Calculate(speed_target, current_speed);
            break;
        }
        case SPEED_MODE:
            if (dir == CLOCKWISE && target_value < 0) target_value = 0;
            if (dir == COUNTERCLOCKWISE && target_value > 0) target_value = 0;
            final_current = speed_controller.Calculate(target_value, current_speed);
            break;
        case CURRENT_MODE:
            final_current = target_value;
            break;
    }
    return (int16_t)final_current;
}

bool Motor::IsTargetReached(float const threshold) {

    if (ctrl_mode != POSITION_MODE) return true;
    float error = target_value - total_angle;

    if (error < 0) error = -error;

    return (error < threshold);
}

void Motor::ToggleDirection() {
    if (dir == CLOCKWISE) dir = COUNTERCLOCKWISE;
    else if (dir == COUNTERCLOCKWISE) dir = CLOCKWISE;
    else dir = EITHERDIRECTION;
}

void Motor::SetDirection(MotorDirection_e direction) {
    dir = direction;
}
