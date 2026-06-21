//
// Created by zhouzhi on 2026/1/20.
//

#include <cstdlib>
#include <random>
#include "../Inc/AppMain.h"

#include "AppMain.h"
#include "Motor.h"
#include "can.h"
#include "usart.h"
#include "WFly_ET_08_remote_control.h"

PID s_pid_normal(5.0f, 0.002f, 0.15f, 0.0f, 10000.0f, 2000.0f, 0.7f);
PID p_pid_normal(0.7f, 0.0f, 0.15f, 0.0f, 2000.0f, 1500.0f, 0.0f);

PID s_pid_init(2.0f, 0.002f, 0.15f, 0.0f, 4200.0f, 2000.0f, 0.7f);
PID p_pid_init(1.0f, 0.0f, 0.1f, 0.0f, 1500.0f, 1500.0f, 0.0f);

PID s_pid_always(1.5f, 0.005f, 0.0f, 0.0f, 15000.0f, 10000.0f, 0.5f);
PID p_pid_always(1.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 0.0f, 0.0f);


Motor new_motor(p_pid_normal, s_pid_normal, p_pid_init, s_pid_init, M3508, SPEED_MODE);
et_08 rm_controller;

uint8_t d[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //tx_data
uint8_t uart_rx_buf[36];
uint8_t uart_rx_data[18];

float target_speed_debug = 1600.0f;
bool ismoving = false;
int16_t current_m3 = 0;

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

    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
        if (htim->Instance == TIM6) {
            //new_App_Task_1ms();
            if (rm_controller.getsc() == 2) {
                CAN_Send(0,0,0);
                new_motor.init_ready = false;
                new_motor.init_x = 0.0f;
                s_pid_normal.Reset();
                p_pid_normal.Reset();
                s_pid_init.Reset();
                p_pid_init.Reset();
            } else if (ismoving && new_motor.IsPositionReached()) {
                CAN_Send(0,0,0);
                ismoving = false;
            }
            else if (rm_controller.getsb() != 2) {
                ismoving = false;
                //new_motor.init_ready = false;
                //new_motor.init_x = 0.0f;
                s_pid_normal.Reset();
                p_pid_normal.Reset();
                s_pid_init.Reset();
                p_pid_init.Reset();
            }
            else {
                if (!ismoving) {
                    if (!new_motor.init_ready) {
                        new_motor.UseInitPID();
                        new_motor.init();
                        current_m3 = new_motor.ExecuteControl();
                        CAN_Send(0, 0, current_m3);
                    }
                    else if(rm_controller.getLeftVert() > 0.5){
                        new_motor.UseNormalPID();
                        set_random_position();
                    }
                }
                if (ismoving && !new_motor.IsPositionReached()) {
                    move_random_position();
                }
            }
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
        if(rm_controller.getsc() == 2) {
            CAN_Send(0, 0, 0);
            return;
        }

        float joystick_val = rm_controller.getLeftHori();

        // 映射到 3508 电机的转速 (M3508 最大转速约 9000rpm，这里设定最大输出 5000 比较安全)
        float target_speed = joystick_val * 5000.0f;

        // 更新电机目标速度
        new_motor.SetTarget(SPEED_MODE, target_speed);

        // 执行 PID 并获取电流值
        current_m3 = new_motor.ExecuteControl();

        // 发送给 CAN 总线
        CAN_Send(0, 0, current_m3);
    }

    void set_random_position() {
        uint32_t tick = HAL_GetTick();
        int16_t random_pos = tick % 90 + 5;
        new_motor.SetPosition(random_pos);
        ismoving = true;
    }

    void move_random_position() {
        current_m3 = new_motor.ExecuteControl();
        CAN_Send(0, 0, current_m3);
        //new_motor.init_ready = false;
    }

    void new_App_CAN_Callback(uint32_t std_id, uint8_t* data) {
        if (std_id == 0x203) {
            new_motor.Update(data);
        }
    }

}
