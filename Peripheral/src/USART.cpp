/*
 * USART.cpp
 *
 *  Created on: 2019/05/02
 *      Author: Shibata
 */

#include "USART.hpp"
#include "stm32f446xx.h"
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

void USART1_Setup(void);
void USART2_Setup(void);
void USART3_Setup(void);
void UART4_Setup(void);
//void USART_Setup(void);

int USART::putcher(uint8_t c){
    while((USART1->SR & USART_SR_TC) == 0);
    USART1->DR = c;
    while((USART1->SR & USART_SR_TXE) == 0);

    return (int)c;
}
int USART::getcher(void){
    uint8_t c;
    c = USART1->DR;

	return (int)c;

}
int USART::write(const uint8_t *data, int size){
    const uint8_t *p = data;
    int cnt = 0;

    if(p == NULL) return 0;
    while(size>0){
        //usart_putchar(*(p++));
        putchar(*(p++));
        size--;
        cnt++;
    }

    return cnt;

}
int USART::read(uint8_t *data, int size){
    int cnt=0;
    while(size > 0){
        //usart_getchar();   //�ǂݎ��
        getchar();
        size--;
        cnt++;
    }

    return cnt;
}
int USART::printf(const char *format, ...){
    char buffer[128];
    va_list ap;
    int len;
    va_start(ap, format);
    len = vsprintf(buffer, format, ap);
    va_end(ap);
    return write((uint8_t*)buffer, len);
}

void USART::setup(int usart_num){

//void USART_Setup(void){
//    //PB6:TX
//    //PB7:RX
//    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
//    GPIOB->AFR[0] |= 0x77000000;    //AF7
//    USART1->BRR = 0x00000271;   //115200bps
//    NVIC_EnableIRQ(USART1_IRQn);
//    USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);
//    USART1->CR1 |= USART_CR1_UE;
//
//    //PD5:TX
//    //PD6:RX
//    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
//    GPIOD->AFR[0] |= 0x07700000;    //AF7
//    USART2->BRR = 0x00000139;   //115200bps
//    NVIC_EnableIRQ(USART2_IRQn);
//    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);
//    USART2->CR1 |= USART_CR1_UE;
//
//    //PD8:TX
//    //PD9:RX
//    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
//    GPIOD->AFR[1] |= 0x00000077;    //AF7
//    USART3->BRR = 0x00000139;   //115200bps
//    NVIC_EnableIRQ(USART3_IRQn);
//    USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE);
//    USART3->CR1 |= USART_CR1_UE;
//
//    //PC10:TX
//    //PC11:RX
//    RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
//    GPIOC->AFR[1] |= 0x00008800;    //AF8
//    UART4->BRR = 0x00000139;   //115200bps
//    NVIC_EnableIRQ(UART4_IRQn);
//    UART4->CR1 |= (USART_CR1_TE | USART_CR1_RE);
//    UART4->CR1 |= USART_CR1_UE;
//}
    this->usart_num = usart_num;
    switch(usart_num){
        case 1:
            USART1_Setup();
            break;
        case 2:
            USART2_Setup();
            break;
        case 3:
            USART3_Setup();
            break;
        case 4:
            UART4_Setup();
            break;
    }
}

void USART::enableRxInterrupt(void){
    switch(usart_num){
        case 1:
            USART1->CR1 |= USART_CR1_RXNEIE;
            break;
        case 2:
            USART2->CR1 |= USART_CR1_RXNEIE;
            break;
        case 3:
            USART3->CR1 |= USART_CR1_RXNEIE;
            break;
        case 4:
            UART4->CR1 |= USART_CR1_RXNEIE;
            break;
        default:
            //error
            break;
    }
}
void USART::disableRxInterrupt(void){
    switch(usart_num){
        case 1:
            USART1->CR1 &= ~(USART_CR1_RXNEIE);
            break;
        case 2:
            USART2->CR1 &= ~(USART_CR1_RXNEIE);
            break;
        case 3:
            USART3->CR1 &= ~(USART_CR1_RXNEIE);
            break;
        case 4:
            UART4->CR1 &= ~(USART_CR1_RXNEIE);
            break;
        default:
            //error
            break;
    }
}
//int USART::isEnableRxInterrupt(void){
//
//}

void USART1_Setup(void){
    //PB6:TX
    //PB7:RX
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    GPIOB->AFR[0] |= 0x77000000;    //AF7
    USART1->BRR = 0x00000139;   //115200bps

    USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    USART1->CR1 |= USART_CR1_UE;
}

void USART2_Setup(void){
    //PD5:TX
    //PD6:RX
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOD->AFR[0] |= 0x07700000;    //AF7

    USART2->BRR = 0x00000139;   //115200bps
    NVIC_EnableIRQ(USART2_IRQn);
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    USART2->CR1 |= USART_CR1_UE;
}

void USART3_Setup(void){
    //PD8:TX
    //PD9:RX
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    GPIOD->AFR[1] |= 0x00000077;    //AF7

    USART3->BRR = 0x00000139;   //115200bps

    //���荞�݂̐ݒ�
    NVIC_EnableIRQ(USART3_IRQn);

    USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    USART3->CR1 |= USART_CR1_UE;
}

void UART4_Setup(void){
    //PC10:TX
    //PC11:RX
    RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
    GPIOC->AFR[1] |= 0x00008800;    //AF8
}

