/*
 * SerialPort.hpp
 *
 *  Created on: 2018/10/20
 *      Author: Shibata
 */

#ifndef SERIALPORT_HPP_
#define SERIALPORT_HPP_

#include <stdint.h>
#include "stm32f446xx.h"
//#include "stm32f103xb.h"

extern int usart1_putchar(uint8_t c);
extern int usart1_getchar(void);
extern int usart1_write(const uint8_t *data, int size);
extern int usart1_read(uint8_t *data, int size);
extern int usart1_printf(const char *format, ...);
extern int usart2_putchar(uint8_t c);
extern int usart2_getchar(void);
extern int usart2_write(const uint8_t *data, int size);
extern int usart2_read(uint8_t *data, int size);
extern int usart2_printf(const char *format, ...);
extern int usart3_putchar(uint8_t c);
extern int usart3_getchar(void);
extern int usart3_write(const uint8_t *data, int size);
extern int usart3_read(uint8_t *data, int size);
extern int usart3_printf(const char *format, ...);

#endif /* SERIALPORT_HPP_ */
