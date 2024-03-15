#include "defines.h"
#include "drivers.h"
#include "analog.h"
#include "chain.h"
#include "comm.h"
#include "eeprom.h"
#include "global.h"
#include "logic.h"
#include "monitor.h"
#include "motor.h"
#include "power.h"

#ifdef DEF_DUPLEX
  #include "inetwork.h"
  #include "master.h"
#endif

/**< Variables defined by compiler */
extern uint32_t _szero;
extern uint32_t _ezero;
extern uint32_t _estack;

/**< Initializing task, will be suspended after completion */
static void InitTask(void *pParameters)
{
  (void) pParameters;
  //static BaseType_t xRet;

  GPIO_Init();

  /**< Initialize watchdog */
  #ifdef DEF_RELEASE
    WDT_Init(WDT_PER_256CYCLES);
    WDT_Enable();
  #endif

  /**< Initialize digital inputs/outputs */
  CHAIN_Configuration();
  /**< Initialize EEPROM */
  EEPROM_Configuration();
  /**< Initialize power monitoring module */
  //POWER_Configuration(); !!!
  /**< Initialize DMA */
  DMA_Init();
  TIMER_Init();
  /**< Initialize master/slave roles */
  #ifdef DEF_DUPLEX
    MASTER_Configuration();
    /**< Set initial ID if ID is wrong */
    if (EE_StationId == 0U)
    {
      /**< Default IDs is 1 for master and 2 for slave */
      if (GLOBAL_InternalID == ID_ACE1)
      {
        EE_StationId = 1;
      } else
      {
        EE_StationId = 2;
      }
    }
  #else
    GLOBAL_IsMaster = true;
    GLOBAL_InternalID = ID_ACE1;
    /**< Set initial ID if ID is wrong */
    if (EE_StationId == 0U)
    {
      EE_StationId = 1;
    }
  #endif
  /**< Initialize analog measurements */
  ANALOG_Configuration();
  /**< Initialize external temperature/humidity sensor */
  //HDC1080_Configuration(HDC1080_CHANNEL);

  /**< Check power supply, stay here for supply failure */
  while (POWER_CheckSupply() == false) {}

  /**< Create main task for periodical actions with low priority */
  (void)xTaskCreate(LOGIC_Task, (const char *) "Logic Task", LOGIC_TASK_STACK_SIZE, NULL, LOGIC_TASK_PRIORITY, &xTaskMain);
  /**< Create CAN communication task with high priority */
  (void)xTaskCreate(COMM_CAN_Task, (const char *) "CAN Task", COMM_TASK_STACK_SIZE, NULL, COMM_CAN_TASK_PRIORITY, NULL);
  /**< Create CAN task for periodic messages */
  (void)xTaskCreate(COMM_CAN_Periodic_Task, (const char *) "CAN Per.Task", COMM_TASK_STACK_SIZE, NULL, COMM_CAN_PER_TASK_PRIORITY, NULL);
  /**< Create ADC monitoring task with lowest priority */
  (void)xTaskCreate(MONITOR_Task, (const char *) "Monitor Task", MONITOR_TASK_STACK_SIZE, NULL, MONITOR_TASK_PRIORITY, NULL);
  /**< Create RS485 communication task with high priority */
  (void)xTaskCreate(COMM_RS485_Task, (const char *) "RS485 Task", COMM_TASK_STACK_SIZE, NULL, COMM_TASK_PRIORITY, &xTaskRS485);
  #ifdef DEF_DUPLEX
    /**< Create internal communication task with highest priority */
    (void)xTaskCreate(INETWORK_Task, (const char *) "INET Task", INET_TASK_STACK_SIZE, NULL, INET_TASK_PRIORITY, &xTaskInet);
  #endif
  /**< Create motor task with very high priority */
  (void)xTaskCreate(MOTOR_Task, (const char *) "Motor Task", MOTOR_TASK_STACK_SIZE, NULL, MOTOR_TASK_PRIORITY, NULL);
//  if (xRet == 0)
//  {
//    xRet++;
//  }
  /**< Reset watchdog */
  WDT_Reset();
  /**< Suspend current initialization task, is not deleted because of heap_1, dynamically memory configuration is not wished */
  vTaskSuspend(NULL);
}

/**< Main function, just starts RTOS */
int main(void)
{
  uint32_t s_size;

  CLK_Init();

  /**< configure pin for blinking LED */
  GPIO_SetupPin(LED_PORT, LED_PIN, GPIO_PIN_FUNC_OFF, GPIO_DIRECTION_OUT, GPIO_LEVEL_HIGH);

  /**< Check RAM integrity here */
  DSU_Init();
  s_size = (uint32_t)&_estack - (uint32_t)&_szero;
  DSU_StartMemoryTest((uint32_t)&_szero, s_size);
  while (DSU_IsDone() != true) {}
  GLOBAL_IsMemoryOk = DSU_IsMemoryOk();

  /**< Create initializing task because of usage of RTOS functions during EEPROM initialization */
  xTaskCreate(InitTask, (const char *) "Init Task", INIT_TASK_STACK_SIZE, NULL, INIT_TASK_PRIORITY, NULL);

  /**< Start FreeRTOS Scheduler */
  vTaskStartScheduler();

  return 0;
}
