//  serial atribution
/*
    To change Serial buffer size,
    please change file C:\Users\Thibault-SED\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.6.18\cores\arduino\HardwareSerial.h
*/

#define RPI_CHANEL 2
#define SCS15_CHANEL 0

//  network related
#define MAX_MOTOR_ON_CHANNEL    26
#define NETWORK_SCHEDULER_SIZE  50

//  control byte flags and mask
#define CTRL_RPI_DOWN               (1 << 0)
#define CTRL_ANALOG_ENABLE          (1 << 1)
#define CTRL_SCS15_SCHEDULER_ENABLE (1 << 2)
#define CTRL_WDT_SHUTDOWN           (1 << 3)

//  process time allowed
#define RPI_ON_LOOP_TIME    20   // 50FPS
#define RPI_OFF_LOOP_TIME   33   // 30FPS
#define NETWORK_TIME        15   // 16ms allowed to the network manager

//  latch
#define NET_LATCH_OUT   43
#define FAULT_LATCH_OUT 38
#define FAULT_LATCH_SET 36
#define FAULT_LATCH_RST 39

//  enable pin
#define EN_5V      35
#define EN_Network 42
#define EN_Front   A0
#define EN_Back    14
#define EN_Leg0    A2
#define EN_Leg1    A4
#define EN_Leg2    A6
#define EN_Leg3    11
#define EN_Leg4    7
#define EN_Leg5    3

//  analog pins
#define ANALOG_COUNT 9
#define Sens_Bat   A13
#define Sens_Front A1
#define Sens_Back  A11
#define Sens_Leg0  A3
#define Sens_Leg1  A5
#define Sens_Leg2  A7
#define Sens_Leg3  A8
#define Sens_Leg4  A9
#define Sens_Leg5  A10

//  PWM
#define PWM_Brushless 45
#define PWM_Fan       2

