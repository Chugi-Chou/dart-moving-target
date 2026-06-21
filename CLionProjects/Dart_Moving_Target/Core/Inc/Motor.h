//
// Created by zhouzhi on 2026/1/20.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "PID.h"
#include "main.h"

enum ControlMode_e { CURRENT_MODE, SPEED_MODE, POSITION_MODE };
enum MotorDirection_e {CLOCKWISE, COUNTERCLOCKWISE, EITHERDIRECTION};
enum MotorType {M2006, M3508};

class Motor{
public:
    Motor(const PID& pos_normal, const PID& speed_normal, const PID& pos_init, const PID& speed_init, MotorType motor_selection, ControlMode_e mode_selection);
    void SetTarget(ControlMode_e mode, float val);
    void SetPosition(int16_t target_pos) {
        target_position = target_pos;
        ctrl_mode = POSITION_MODE;
    }
    void UpdateFeedback(const uint8_t data[8]);
    void Update(const uint8_t data[8]);
    int16_t ExecuteControl();
    float GetCurrentAngle() const { return total_angle; }
    bool IsTargetReached(float threshold = 1.0f);
    bool IsPositionReached(float threshold = 1.5f);
    void ToggleDirection();
    void SetDirection(MotorDirection_e direction);
    void UseInitPID();
    void UseNormalPID();
    void init();
    void check();
    bool init_ready;
    float init_x;

private:
    PID pos_controller_normal;
    PID speed_controller_normal;
    PID pos_controller_init;
    PID speed_controller_init;
    PID* current_pos_ctrl;
    PID* current_speed_ctrl;
    ControlMode_e ctrl_mode;
    MotorDirection_e dir;
    float target_value, target_position;
    float current_speed, total_angle, total_x, last_total_x;
    int16_t last_raw_angle, delta_raw_angle, current;
    int32_t rounds;
    bool initialized;
    MotorType motor_type;
    float reduction_ratio;
    uint16_t stuck_time;
    const float stuck_current;
    const float max_stuck_time;
};

#endif //MOTOR_H
