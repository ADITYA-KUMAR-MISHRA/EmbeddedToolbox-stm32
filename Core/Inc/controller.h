/**
 * @file controller.h
 *
 * Code for Embedded ToolBox project
 *
 * EB = Embedded Toolbox
 * Implementation and Prototype files are same and its that file
 *
 */
#include "main.h"
#include "serialcommands.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_conf.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_uart.h"


/***** VARIABLES ******/
typedef struct __ET_ConfStruct
{
	GPIO_TypeDef *GPIO_PORT;  		// GPIO port this library will provide access to
	UART_HandleTypeDef *UART; // USART to use for comm with desktop client
}ET_Config;

ET_Config ET;

/***** FUNCTIONS ******/
void ET_action(unsigned char *cmd);
/**
 * @brief decode and execute command send by PC, requied 3*byte array
 *
 * @return void.
 */

void ET_send_data();
/**
 * @brief Send various data to serial like gpio, adc etc
 *
 * @return void.
 */

void ET_set_pinMode(unsigned char pin, unsigned char pinmode);
/**
 * @brief set pinmode(i.e output/input) of a pin
 *
 * @param pin:	Index unsigned charo pounsigned charer array to set value
 * @param pinmode:	it can be either PinMode::PinMode_Input or PinMode::PinMode_Output
 *
 * @return void.
 */

void ET_set_digitalWrite(unsigned char pin, unsigned char output);
/**
 * @brief set pin to either high or low (in Push Push mode)
 *
 * @param pin:	pin on port A, 16 bit port
 * @param output:	can be either Output::Output_Low or Output::Output_Low
 *
 * @return void.
 */

void ET_send_digitalRead();
/**
 * @brief sends input reading of PortA
 *
 *
 * @return void.
 */

void ET_send_analogRead();
/**
 * @brief
 *
 *
 * @return
 */



/**************************** IMPLEMENTATION **************************************/
/**
 * Data formats coming from PC
 * byte0			byte1			byte2
 * CMD_PinMode 		pin 			pinmode
 * CMD_DigitalWrite	pin				output
 * */


void ET_action(unsigned char * cmd)
{
	if(cmd[1] > 15 || cmd[1] < 0) // pin must be in range [0,15]
		return;
	if(cmd[0] < 0  || cmd[0] > 1)
		return;

	unsigned char test_pin = 1<<cmd[1];
	HAL_UART_Transmit(ET.UART, cmd, 3, 10); //DEBUG
	HAL_UART_Transmit(ET.UART, &test_pin, 1, 10); //DEBUG


	switch(cmd[0])
	{
	case CMD_PinMode:
		ET_set_pinMode(cmd[1], cmd[2]);
		break;
	case CMD_DigitalWrite:
		ET_set_digitalWrite(cmd[1], cmd[2]);
		break;
	default:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_Delay(100);
		return;
	}
}

void ET_send_data()
{
//	unsigned char gpio_data[2];
//	gpio_data[0] = ET.GPIO_PORT->IDR << 8;  // LSB
//	gpio_data[0] = ET.GPIO_PORT->IDR << 8;  // MSB

}

void ET_set_pinMode(unsigned char pin, unsigned char pinmode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = 1<<pin; // Set pin

	switch(pinmode)
	{
	case PinMode_Input:

		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;

		HAL_UART_Transmit(ET.UART, "PinMode_Input\nPin:", 18, 10); //DEBUG
		HAL_UART_Transmit(ET.UART, pin+48, 1, 10); //DEBUG
		HAL_UART_Transmit(ET.UART, "\n", 1, 10); //DEBUG
		break;

	case PinMode_Output:

		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

		HAL_UART_Transmit(ET.UART, "PinMode_Output\nPin:", 18, 10); //DEBUG
		HAL_UART_Transmit(ET.UART, pin+48, 1, 10); //DEBUG
		HAL_UART_Transmit(ET.UART, "\n", 1, 10); //DEBUG


	default: return;
	}

//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void ET_set_digitalWrite(unsigned char pin, unsigned char output){
	HAL_GPIO_WritePin(GPIOB, 1<<pin, output);

//	HAL_UART_Transmit(ET.UART, str_buff, 50, 50); //DEBUG
//	HAL_UART_Transmit(ET.UART, "CMD_DigitalWrite\nPin:", 21, 10); //DEBUG
//	HAL_UART_Transmit(ET.UART, pin+10, 1, 10); //DEBUG
//	HAL_UART_Transmit(ET.UART, "OP:", 3, 10); //DEBUG
//	HAL_UART_Transmit(ET.UART, output? "High":"Low", 1, 10); //DEBUG
//	HAL_UART_Transmit(ET.UART, "\n", 1, 10); //DEBUG

}

void ET_send_digitalRead()
{
//	HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
}

void ET_send_analogRead()
{

}
