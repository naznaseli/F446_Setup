/*
 * USART.hpp
 *
 *  Created on: 2019/05/01
 *      Author: Shibata
 */

#ifndef USART_HPP_
#define USART_HPP_

#include <stdint.h>

class USART{
public:
    void setup(int usart_num);
    void enableRxInterrupt(void);
    void disableRxInterrupt(void);
    int isEnableRxInterrupt(void);

    //void clear(void);
    int putcher(uint8_t c);
    int getcher(void);
    int write(const uint8_t *data, int size);
    int read(uint8_t *data, int size);
    int printf(const char *format, ...);
private:
    int usart_num;
};

//:**********************************************************************
//!
//! @brief  UART,USART�����ݒ�
//!
//! @note   USART1  : Write, PC communication
//!         USART2  : SBDBT
//!         USART3   : Bluetooth debug, Other
//!         UART4   : Serial servo
//!
//:**********************************************************************
//USART3->BRR = 0x00000182; //180MHz
//USART3->BRR = 0x00000165; //168MHz
//USART3->BRR = 0x0000009C; //72MHz
//USART3->BRR = 0x00000139; //144MHz


#endif /* USART_HPP_ */
