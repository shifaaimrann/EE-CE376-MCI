#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#include "main.h"

/* Public Functions */
void BT_Init(void);
void BT_Listen(void); // Start the interrupt listener

/* Debugging Functions */
void BT_Debug_EchoMode(void); // Simple echo test to verify UART pins
void BT_PrintGains(void);     // Send current PID values to terminal

#endif