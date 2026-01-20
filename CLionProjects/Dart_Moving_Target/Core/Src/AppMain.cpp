//
// Created by zhouzhi on 2026/1/20.
//

#include "../Inc/AppMain.h"

#include "AppMain.h"
#include "Motor.h"
#include "can.h"
#include "usart.h"
#include "WFly_ET_08_remote_control.h"

PID s_pid_task(1.5f, 0.05f, 0.0f, 0.0f, 10000.0f, 8000.0f, 0.7f);
PID p_pid_task(5.0f, 0.05f, 0.0f, 0.0f, 10000.0f, 0.0f, 0.0f); // 加上 Kd

PID s_pid_always(1.5f, 0.005f, 0.0f, 0.0f, 15000.0f, 10000.0f, 0.5f);
PID p_pid_always(1.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 0.0f, 0.0f);

Motor motor_task(p_pid_task, s_pid_task, M2006, SPEED_MODE);
Motor motor_always(p_pid_always, s_pid_always, M2006, SPEED_MODE);

Motor new_motor(p_pid_task, p_pid_task, M3508, SPEED_MODE);
et_08 rm_controller;

uint8_t d[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //tx_data
uint8_t uart_rx_buf[36];
uint8_t uart_rx_data[18];

float target_speed_debug = 1600.0f;

struct CAN_Debug_t {
    uint32_t id;
    uint8_t data[8];
};

volatile CAN_Debug_t can_rx_debug[2];

void CAN_Send(int16_t c1, int16_t c2, int16_t c3) {
    CAN_TxHeaderTypeDef hdr;
    uint32_t box;

    d[0] = (uint8_t)(c1>>8);
    d[1] = (uint8_t)c1;
    d[2] = (uint8_t)(c2>>8);
    d[3] = (uint8_t)c2;
    d[4] = (uint8_t)(c3>>8);
    d[5] = (uint8_t)c3;
    d[6] = 0;
    d[7] = 0;

    hdr.StdId = 0x200; hdr.IDE = CAN_ID_STD; hdr.RTR = CAN_RTR_DATA; hdr.DLC = 8;
    HAL_CAN_AddTxMessage(&hcan1, &hdr, d, &box);
}

extern "C" {

    void App_Init(void) {
        CAN_FilterTypeDef f = {0, 0, 0, 0, 0, CAN_FILTER_FIFO0, 0, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT, ENABLE};
        HAL_CAN_ConfigFilter(&hcan1, &f);
        HAL_CAN_Start(&hcan1);
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);

        extern TIM_HandleTypeDef htim6;
        HAL_TIM_Base_Start_IT(&htim6);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_rx_buf, sizeof(uart_rx_buf));

        motor_always.SetTarget(SPEED_MODE, target_speed_debug);
        motor_task.SetTarget(POSITION_MODE, 0);
    }

    void App_Task_1ms(void) {
        static uint16_t btn_filter = 0; //防抖滤波
        static bool is_waiting_for_finish = false; //状态锁
        bool raw_pin = (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET);
        static bool current_status = false;

        static uint32_t heartbeat = 0;

        if (++heartbeat >= 500) {
            HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);
            heartbeat = 0;
        } //MCU运行指示灯


        if (!is_waiting_for_finish) {
            if (raw_pin) {
                btn_filter++;
                if (btn_filter >= 20) {
                    if (!current_status) {
                        motor_task.SetDirection(CLOCKWISE);
                        motor_task.SetTarget(POSITION_MODE, motor_task.GetCurrentAngle() + 82.0f);
                        current_status = true;
                    }
                    else {
                        motor_task.SetDirection(COUNTERCLOCKWISE);
                        motor_task.SetTarget(POSITION_MODE, motor_task.GetCurrentAngle() - 82.0f);
                        current_status = false;
                    }
                    is_waiting_for_finish = true;
                    btn_filter = 0;
                    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);
                }
            } else {
                btn_filter = 0;
            }
        } else {

            float error = motor_task.GetCurrentAngle() - (motor_task.GetCurrentAngle());
            if (motor_task.IsTargetReached()) {
                if (!raw_pin) {
                    is_waiting_for_finish = false;
                    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);
                }
            }
        }


        CAN_Send(motor_task.ExecuteControl(), motor_always.ExecuteControl(), new_motor.ExecuteControl());
    }

    void App_CAN_Callback(uint32_t std_id, uint8_t* data) {

        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);

        if (std_id == 0x201) {
            can_rx_debug[0].id = std_id;
            for (int i = 0; i < 8; i++) {can_rx_debug[0].data[i] = data[i];}
            motor_task.UpdateFeedback(data);
        }
        else if (std_id == 0x202) {
            can_rx_debug[1].id = std_id;
            for (int i = 0; i < 8; i++) {can_rx_debug[1].data[i] = data[i];}
            motor_always.UpdateFeedback(data);
        }
    }

    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
        if (htim->Instance == TIM6) {
            new_App_Task_1ms();
        }
    }

    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
        CAN_RxHeaderTypeDef hdr;
        uint8_t data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) == HAL_OK) {
            new_App_CAN_Callback(hdr.StdId, data);
        }
    }

    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
        if (huart->Instance == USART3) {
            if(!rm_controller.process_rc_frame(uart_rx_buf, Size)) HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);
						else HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_rx_buf, sizeof(uart_rx_buf));
        }
    }

    void new_App_Init(void) {
        CAN_FilterTypeDef f = {0, 0, 0, 0, 0, CAN_FILTER_FIFO0, 0, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT, ENABLE};
        HAL_CAN_ConfigFilter(&hcan1, &f);
        HAL_CAN_Start(&hcan1);
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);

        extern TIM_HandleTypeDef htim6;
        HAL_TIM_Base_Start_IT(&htim6);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_rx_buf, sizeof(uart_rx_buf));

        new_motor.SetTarget(SPEED_MODE, 0);
        new_motor.SetDirection(EITHERDIRECTION);
    }

    void new_App_Task_1ms(void) {
        float joystick_val = rm_controller.getLeftHori();

        // 映射到 3508 电机的转速 (M3508 最大转速约 9000rpm，这里设定最大输出 5000 比较安全)
        float target_speed = joystick_val * 5000.0f;

        // 更新电机目标速度
        new_motor.SetTarget(SPEED_MODE, target_speed);

        // 执行 PID 并获取电流值
        int16_t current_m1 = motor_task.ExecuteControl();
        int16_t current_m2 = motor_always.ExecuteControl();
        int16_t current_m3 = new_motor.ExecuteControl();

        // 发送给 CAN 总线
        CAN_Send(current_m1, current_m2, current_m3);
    }

    void new_App_CAN_Callback(uint32_t std_id, uint8_t* data) {
        if (std_id == 0x203) {
            new_motor.UpdateFeedback(data);
        }
    }

}
