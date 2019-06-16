/*
 * SerialPort.cpp
 *
 *  Created on: 2018/10/20
 *      Author: Shibata
 */

#include "SerialPort.hpp"

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

int usart1_putchar(uint8_t c);
int usart1_getchar(void);
int usart1_write(const uint8_t *data, int size);
int usart1_read(uint8_t *data, int size);
int usart1_printf(const char *format, ...);

int usart2_putchar(uint8_t c);
int usart2_getchar(void);
int usart2_write(const uint8_t *data, int size);
int usart2_read(uint8_t *data, int size);
int usart2_printf(const char *format, ...);

int usart1_putchar(uint8_t c){
    while((USART1->SR & USART_SR_TC) == 0);
    USART1->DR = c;
    while((USART1->SR & USART_SR_TXE) == 0);

	return (int)c;
}

int usart1_getchar(void){
    uint8_t c;
    c = USART1->DR;

	return (int)c;
}

int usart1_write(const uint8_t *data, int size){
    const uint8_t *p = data;
    int cnt = 0;

    if(p == NULL) return 0;
    while(size>0){
        usart1_putchar(*(p++));
        size--;
        cnt++;
    }

    return cnt;
}

int usart1_read(uint8_t *data, int size){
    int cnt=0;
    while(size > 0){
        usart1_getchar();   //�ǂݎ��
        size--;
        cnt++;
    }

    return cnt;
}

int usart1_printf(const char *format, ...){
    char buffer[128];
    va_list ap;
    int len;
    va_start(ap, format);
    len = vsprintf(buffer, format, ap);
    va_end(ap);
    return usart1_write((uint8_t*)buffer, len);
}

int usart2_putchar(uint8_t c){
    while((USART2->SR & USART_SR_TC) == 0);
    USART2->DR = c;
    while((USART2->SR & USART_SR_TXE) == 0);

	return (int)c;
}

int usart2_getchar(void){
    uint8_t c;
    c = USART2->DR;

	return (int)c;
}

int usart2_write(const uint8_t *data, int size){
    const uint8_t *p = data;
    int cnt = 0;

    if(p == NULL) return 0;
    while(size>0){
        usart2_putchar(*(p++));
        size--;
        cnt++;
    }

    return cnt;
}

int usart2_read(uint8_t *data, int size){
    int cnt=0;
    while(size > 0){
        usart2_getchar();   //�ǂݎ��
        size--;
        cnt++;
    }

    return cnt;
}

int usart2_printf(const char *format, ...){
    char buffer[128];
    va_list ap;
    int len;
    va_start(ap, format);
    len = vsprintf(buffer, format, ap);
    va_end(ap);
    return usart2_write((uint8_t*)buffer, len);
}

int usart3_putchar(uint8_t c){
    while((USART3->SR & USART_SR_TC) == 0);
    USART3->DR = c;
    while((USART3->SR & USART_SR_TXE) == 0);

	return (int)c;
}

int usart3_getchar(void){
    uint8_t c;
    c = USART3->DR;

	return (int)c;
}

int usart3_write(const uint8_t *data, int size){
    const uint8_t *p = data;
    int cnt = 0;

    if(p == NULL) return 0;
    while(size>0){
        usart3_putchar(*(p++));
        size--;
        cnt++;
    }

    return cnt;
}

int usart3_read(uint8_t *data, int size){
    int cnt=0;
    while(size > 0){
        usart3_getchar();   //�ǂݎ��
        size--;
        cnt++;
    }

    return cnt;
}

int usart3_printf(const char *format, ...){
    char buffer[128];
    va_list ap;
    int len;
    va_start(ap, format);
    len = vsprintf(buffer, format, ap);
    va_end(ap);
    return usart3_write((uint8_t*)buffer, len);
}
