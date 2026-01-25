//
// Created by zhouzhi on 2026/1/20.
//

#include "../Inc/Motor.h"

#include <cmath>
#include "PID.h"
#include "main.h"

Motor::Motor(const PID& pos_pid, const PID& speed_pid, MotorType motor_selection, ControlMode_e ctrl_selection)
    : pos_controller(pos_pid), speed_controller(speed_pid), motor_type(motor_selection), ctrl_mode(ctrl_selection),
      target_value(0), initialized(false), init_ready(false), init_x(0.0f), total_x(0.0f), total_angle(0.0f), last_total_x(0.0f),
      stuck_time(0), stuck_current(3190), max_stuck_time(10) {
    dir = CLOCKWISE;
    switch (motor_type) {
        case M2006: reduction_ratio = 36.0f; break;
        case M3508: reduction_ratio = 3591.0f/187.0f; break;
    }
}

void Motor::init() {

    if (std::fabs(current) > stuck_current && std::fabs(total_x - last_total_x) < 0.005) {
        stuck_time++;
    } else {
        stuck_time = 0;
    }

    if (stuck_time < max_stuck_time) {
        init_x-=0.01;
        SetPosition(init_x);
        last_total_x = total_x;
    } else {
        SetTarget(SPEED_MODE, 0.0f);
        total_angle = 0;
        total_x = 0;
        last_total_x = 0;
        target_position = 0;
        init_x = 0;
        init_ready = true;
        stuck_time = 0;
    }
}


void Motor::UpdateFeedback(const uint8_t data[8]) {
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
void Motor::Update(const uint8_t data[8]){
    int16_t raw_angle = (data[0] << 8) | data[1];
    current_speed = (float)((int16_t)((data[2] << 8) | data[3]));
    if (!initialized) {
        last_raw_angle = raw_angle;
        initialized = true;
        return;
    }
    delta_raw_angle = raw_angle - last_raw_angle;
    if (delta_raw_angle > 4096) delta_raw_angle -= 8192;
    else if (delta_raw_angle < -4096) delta_raw_angle += 8192;
    total_angle += delta_raw_angle / 8192.0f * 360.0f;
    total_x = total_angle / 360.0f * 2.3f;//大致cm
    current = static_cast<int16_t>((data[4] << 8) | data[5]);
    last_raw_angle = raw_angle;
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
            float speed_target = pos_controller.Calculate(target_position * 3600, total_x * 3600);

            if (dir == CLOCKWISE) {
                if (speed_target < 0) speed_target = 0;
            } else if (dir == COUNTERCLOCKWISE){
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

bool Motor::IsPositionReached(float const threshold) {

    //if (ctrl_mode != POSITION_MODE) return true;
    float error = target_position - total_x;

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
