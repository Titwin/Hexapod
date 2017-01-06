#define RPI_CHANEL 2
#define SCS15_CHANEL 0

#define MAX_MOTOR_ON_CHANNEL 26
#define NETWORK_SCHEDULER_SIZE 50

#define MAX_ANALOG_INPUT 16
#define MAX_ANALOG_OUTPUT 10
#define FIRST_ANALOG_OUTPUT 2

#define CTRL_WDT_SHUTDOWN (1 << 0)
#define CTRL_RPI_DOWN (1 << 1)
#define CTRL_ANALOG_ENABLE (1 << 2)
#define CTRL_SCS15_SCHEDULER_ENABLE (1 << 3)

#define RPI_ON_LOOP_TIME 20   // 50FPS
#define RPI_OFF_LOOP_TIME 33  // 30FPS
#define NETWORK_TIME 16       // 16ms allowed to the network manager



