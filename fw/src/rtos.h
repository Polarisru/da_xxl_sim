#ifndef RTOS_H
#define RTOS_H

#ifndef DEF_UNITTEST
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define MONITOR_TASK_STACK_SIZE   100U
#define MOTOR_TASK_STACK_SIZE     150U
#define LOGIC_TASK_STACK_SIZE     150U
#define INET_TASK_STACK_SIZE      200U
#define COMM_TASK_STACK_SIZE      250U
#define INIT_TASK_STACK_SIZE      100U

#define LOGIC_TASK_PRIORITY       (tskIDLE_PRIORITY + 2)
#define MONITOR_TASK_PRIORITY     (tskIDLE_PRIORITY + 3)
#define COMM_TASK_PRIORITY        (tskIDLE_PRIORITY + 4)
#define COMM_CAN_TASK_PRIORITY    (tskIDLE_PRIORITY + 4)
#define COMM_CAN_PER_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define INET_TASK_PRIORITY        (tskIDLE_PRIORITY + 6)
#define MOTOR_TASK_PRIORITY       (tskIDLE_PRIORITY + 5)
#define INIT_TASK_PRIORITY        (tskIDLE_PRIORITY + 7)
#endif

#define LED_BLINKING_DELAY_MS     1000U
#define LOGIC_TASK_DELAY_MS       10U
#define MOTOR_TASK_DELAY_MS       1U
#define HALL_CHECK_DELAY_MS       10U
#define MONITOR_TASK_DELAY_MS     1U

#endif
