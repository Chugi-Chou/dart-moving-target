//
// Created by zhouzhi on 2026/1/20.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "PID.h"
#include "main.h"

enum ControlMode_e { CURRENT_MODE, SPEED_MODE, POSITION_MODE };
enum MotorDirection_e {CLOCKWISE, COUNTERCLOCKWISE};
enum MotorType {M2006, M3508};

class Motor{
public:
    Motor(const PID& pos_pid, const PID& speed_pid, MotorType motor_selection, ControlMode_e mode_selection);
    void SetTarget(ControlMode_e mode, float val);
    void UpdateFeedback(uint8_t data[8]);
    int16_t ExecuteControl();
    float GetCurrentAngle() const { return total_angle; }
    bool IsTargetReached(float threshold = 1.0f);
    void ToggleDirction();
    void SetDirection(MotorDirection_e direction);

private:
    PID pos_controller;
    PID speed_controller;
    ControlMode_e ctrl_mode;
    MotorDirection_e dir;
    float target_value;
    float current_speed, total_angle;
    int16_t last_raw_angle;
    int32_t rounds;
    bool initialized;
    MotorType motor_type;
    float reduction_ratio;
};

#endif //MOTOR_H
