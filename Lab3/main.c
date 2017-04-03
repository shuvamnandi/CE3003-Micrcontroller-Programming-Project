#include <includes.h>
#include <LED.h>

static  OS_TCB       AppTaskStartTCB;
static  OS_TCB       AppTaskLedTCB;


static  CPU_STK      AppTaskStartStk[APP_TASK_START_STK_SIZE];
static  CPU_STK      AppTaskLedStk[APP_TASK_LED_STK_SIZE];


OS_Q IntQ;

CPU_INT08U Left_tgt;
CPU_INT08U Right_tgt;

static  void  AppTaskStart        (void  *p_arg);
static  void  AppTaskLed        (void  *p_arg);
static  void  RoboMove(CPU_INT16U segL, CPU_INT16U segR, CPU_INT16U speedL, CPU_INT16U speedR);
static  void  IntWheelSensor();

int main()
{
  OS_ERR  err; 
    
      OSInit(&err);
      
    OSTaskCreate((OS_TCB     *)&AppTaskStartTCB,                                        
                 (CPU_CHAR   *)"App Task Start",
                 (OS_TASK_PTR ) AppTaskStart,
                 (void       *) 0,
                 (OS_PRIO     ) APP_TASK_START_PRIO,
                 (CPU_STK    *)&AppTaskStartStk[0],
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK |                                                          OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
    
    OSTaskCreate((OS_TCB     *)&AppTaskLedTCB,                                        
                 (CPU_CHAR   *)"App Task LED",
                 (OS_TASK_PTR ) AppTaskLed,
                 (void       *) 0,
                 (OS_PRIO     ) APP_TASK_LED_PRIO,
                 (CPU_STK    *)&AppTaskLedStk[0],
                 (CPU_STK_SIZE) APP_TASK_LED_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_TASK_LED_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK |                                                          OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
    
    OSStart(&err);  /* give control to uC/OS-III. */

  return 0;
}


static  void  AppTaskLed (void  *p_arg)
{
  OS_ERR      err;
  (void)&p_arg;
  //CPU_INT08U counter = 8; //for Maze 1
  CPU_INT08U counter = 15; //for Maze 2
  LEDsInit();
  
  while(counter > 0)
  {
      LED_On(0);
      OSTimeDlyHMSM(0u, 0u, 1u, 0u,
                        OS_OPT_TIME_HMSM_STRICT,
                        &err); // 1sec for LED on

      LED_Off(0);
      OSTimeDlyHMSM(0u, 0u, 1u, 0u,
                        OS_OPT_TIME_HMSM_STRICT,
                        &err); // 1sec for LED off
      counter--;
      //total 2sec in the loop
  }
}

static  void  AppTaskStart (void  *p_arg)
{
  CPU_INT32U  clk_freq;
  CPU_INT32U  cnts;  
  OS_ERR      err;

   (void)&p_arg;

   BSP_Init();    /* Initialize the BSP services     */    
   CPU_Init();    /* Initialize the uC/CPU services     */
   
   clk_freq = BSP_CPUClkFreq(); 
                              /*Determine SysTick reference freq.*/
    cnts  = clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;        
                                /* Determine nbr SysTick increments*/
    OS_CPU_SysTickInit(cnts);
                        /* Init uC/OS periodic time src (SysTick). */
    CPU_TS_TmrFreqSet(clk_freq);

#if (OS_CFG_STAT_TASK_EN > 0u)
    OSStatTaskCPUUsageInit(&err);                               
                      /* Compute CPU capacity with no task running */
#endif

    CPU_IntDisMeasMaxCurReset();
          
    OSQCreate((OS_Q *)&IntQ,
              (CPU_CHAR *)"ISR Message Queue",
              (OS_MSG_QTY)10,
              (OS_ERR *)&err);
    
    BSP_DisplayInit();
    BSP_MotorsInit();
    BSP_WheelSensorsInit();							// Init functions		
    
    BSP_WheelSensorIntEnable(RIGHT_SIDE, SENSOR_A, (CPU_FNCT_VOID)IntWheelSensor);
    BSP_WheelSensorIntEnable(LEFT_SIDE, SENSOR_A, (CPU_FNCT_VOID)IntWheelSensor);	//Register the ISRs


    //Go forward for 15sec
    //segL,segR,speedL,speedR
    
    // Maze 1
    // Curve way
    // RoboMove(116u,84u,60u,48u);
    
    //Straight way
    RoboMove(5u, 0u, 50u, 0u);
    RoboMove(29u, 29u, 70u, 70u);
    RoboMove(9u, 0u, 50u, 0u);
    RoboMove(34u, 34u, 70u, 70u);
    RoboMove(12u, 0u, 50u, 0u);
    RoboMove(24u, 24u, 70u, 70u);
    //RoboMove(5u, 0u, 50u, 0u);
    //RoboMove(10u, 10u, 65u, 66u);
    
    /*
    // Maze 2
    
    // Right wheel is faster
    // 1st Stretch
    RoboMove(23u, 23u, 69u, 70u);
    // Turn left
    RoboMove(0u, 14u, 0u, 60u);
    // Long stretch
    RoboMove(90u, 90u, 74u, 75u);
    // Turn left
    RoboMove(0u, 14u, 0u, 70u);
    // Small segment
    RoboMove(11u, 11u, 74u, 75u);
    // Turn left
    RoboMove(0u, 14u, 0u, 59u);
    // last stretch
    RoboMove(69u, 69u, 72u, 73u);
    // Turn right
    RoboMove(14u, 0u, 75u, 0u);
    // Move into circle
    RoboMove(2u, 2u, 63u, 64u);
    */
    
    BSP_DisplayClear();
    BSP_DisplayStringDraw("DONE !!!", 30u, 0u);
    
     while (DEF_ON) {                                            /* Task body, always written as an infinite loop.       */
        OSTimeDlyHMSM(0u, 0u, 1u, 0u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

    }


}

void IntWheelSensor()
{
        OS_ERR err;

	CPU_INT32U         ulStatusR_A;
	CPU_INT32U         ulStatusL_A;

	static CPU_INT08U CountL = 0;
	static CPU_INT08U CountR = 0;

	static CPU_INT08U data = 0;

	ulStatusR_A = GPIOPinIntStatus(RIGHT_IR_SENSOR_A_PORT, DEF_TRUE);
	ulStatusL_A = GPIOPinIntStatus(LEFT_IR_SENSOR_A_PORT, DEF_TRUE);

        if (ulStatusR_A & RIGHT_IR_SENSOR_A_PIN)    //Exp
        {
          GPIOPinIntClear(RIGHT_IR_SENSOR_A_PORT, RIGHT_IR_SENSOR_A_PIN);           /* Clear interrupt.*/
          CountR = CountR + 1;
        }

        if (ulStatusL_A & LEFT_IR_SENSOR_A_PIN)      //Exp
        {
          GPIOPinIntClear(LEFT_IR_SENSOR_A_PORT, LEFT_IR_SENSOR_A_PIN);
          CountL = CountL + 1;
        }

	if((CountL >= Left_tgt) && (CountR >= Right_tgt)){
          data = 0x11;
          Left_tgt = 0;
          Right_tgt = 0;
          CountL = 0;
          CountR = 0;
          
          OSQPost((OS_Q *)&IntQ,
             (CPU_INT08U *)data,
             (OS_MSG_SIZE)sizeof(CPU_INT16U *),
             (OS_OPT)OS_OPT_POST_FIFO,
             (OS_ERR *)&err);
        }
        else if(CountL >= Left_tgt){
          data = 0x10;
          Left_tgt = 0;
          CountL = 0;
          
          OSQPost((OS_Q *)&IntQ,
             (CPU_INT08U *)data,
             (OS_MSG_SIZE)sizeof(CPU_INT16U *),
             (OS_OPT)OS_OPT_POST_FIFO,
             (OS_ERR *)&err);
        }
        else if(CountR >= Right_tgt){
          data = 0x01;
          Right_tgt = 0;
          CountR = 0;
          
          OSQPost((OS_Q *)&IntQ,
             (CPU_INT08U *)data,
             (OS_MSG_SIZE)sizeof(CPU_INT16U *),
             (OS_OPT)OS_OPT_POST_FIFO,
             (OS_ERR *)&err);
        }
}

void RoboMove(CPU_INT16U segL, CPU_INT16U segR, CPU_INT16U speedL, CPU_INT16U speedR)
{
    OS_ERR err;
    OS_MSG_SIZE size;
    CPU_TS ts;
        
    CPU_INT08U counts = 0;
    
    Left_tgt = segL;
    Right_tgt = segR; 
    
    // set the target distance
                                  
    BSP_MotorStop(LEFT_SIDE);
    BSP_MotorStop(RIGHT_SIDE);

    BSP_MotorSpeed(LEFT_SIDE, speedL <<8u);
    BSP_MotorSpeed(RIGHT_SIDE,speedR <<8u);     // set speed

    BSP_MotorDir(LEFT_SIDE,FORWARD);
    BSP_MotorDir(RIGHT_SIDE,FORWARD);        // Set the motor parameters
            
    BSP_MotorRun(LEFT_SIDE);
    BSP_MotorRun(RIGHT_SIDE);		    // Start the motors
    
    
    OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);                                 
    // Just a small wait time to let the motors run
    do{
      // wait until the message is received : no timeout
      counts = (CPU_INT08U)OSQPend((OS_Q *)&IntQ,
                                   (OS_TICK)0,                              
                                   (OS_OPT)OS_OPT_PEND_BLOCKING,
                                   (OS_MSG_SIZE *)&size,
                                   (CPU_TS *)&ts,
                                   (OS_ERR *)&err);

          if(counts == 0x11){	    // Both the wheels reached target		
             BSP_MotorStop(RIGHT_SIDE);
             BSP_MotorStop(LEFT_SIDE);
             Left_tgt = 0;
             Right_tgt = 0;
          }
          else if(counts == 0x10){	// Left wheel reached target		
             BSP_MotorStop(LEFT_SIDE);
             Left_tgt = 0;
          }
          else if(counts == 0x01){	 //Right Wheel reached taget		
             BSP_MotorStop(RIGHT_SIDE);
             Right_tgt = 0;
          }
              
      } while((Left_tgt)||(Right_tgt));   //Continue until both wheels reach target
    
}