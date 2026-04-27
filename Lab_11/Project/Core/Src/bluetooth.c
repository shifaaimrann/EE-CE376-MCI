#include "bluetooth.h"
#include "pid.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart2; // Standard USART for Balance Shield
uint8_t rx_data[16];             // Buffer for incoming command strings
uint8_t rx_index = 0;
uint8_t rx_byte;

/**
 * @brief Enables the UART Receive Interrupt.
 */
void BT_Init(void) {
    BT_Listen();
}

void BT_Listen(void) {
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

/**
 * @brief ISR Callback: Triggered every time a character is received.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Build the string until a newline or carriage return is found
        if (rx_byte == '\n' || rx_byte == '\r') {
            rx_data[rx_index] = '\0';
            
            // Parse Command: Format is [Letter][Value] e.g. "P15.2"
            char type = rx_data[0];
            float val = atof((char*)&rx_data[1]);

            if (type == 'P') Kp = val;
            else if (type == 'I') Ki = val;
            else if (type == 'D') Kd = val;

            rx_index = 0; // Reset buffer
            BT_PrintGains(); // Confirm change back to phone
        } else {
            if (rx_index < 15) {
                rx_data[rx_index++] = rx_byte;
            }
        }
        BT_Listen(); // Re-enable interrupt
    }
}

/* === DEBUGGING FUNCTIONS === */

/**
 * @brief Prints current PID gains to the Bluetooth terminal.
 */
void BT_PrintGains(void) {
    char msg[64];
    sprintf(msg, "\r\nUpdated: Kp=%.2f Ki=%.2f Kd=%.2f\r\n", Kp, Ki, Kd);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

/**
 * @brief Use this to check if the 8V mistake fried the UART pins.
 * If you type something on your phone and it doesn't echo back, 
 * the STM32's USART2 peripheral is compromised.
 */
void BT_Debug_EchoMode(void) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"UART Echo Test Start\r\n", 22, 100);
    // This mode just sends back what it receives
}