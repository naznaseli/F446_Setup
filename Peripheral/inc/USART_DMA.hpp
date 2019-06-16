/*
 * USART_DMA.hpp
 *
 *  Created on: 2019/05/10
 *      Author: Shibata
 */

#ifndef USART_DMA_HPP_
#define USART_DMA_HPP_

#include "stm32f446xx.h"
#include "GPIO.hpp"
#include <stdint.h>

//DMA1_Stream3->PAR = (uint32_t)&USART3->DR;//peripheral address
#ifdef __cplusplus
extern "C"{
#endif
extern void DMA2_Stream7_IRQHandler(void); //usart1_tx
extern void DMA1_Stream6_IRQHandler(void); //usart2_tx
extern void DMA1_Stream3_IRQHandler(void); //usart3_tx
#ifdef __cplusplus
}
#endif


class USART_DMA{
public:
    static const uint8_t BufferSize = 128;
    uint8_t tx_buf_ptr;
    uint8_t rx_buf_ptr;
    uint8_t tx_buf[BufferSize] = {0};
    uint8_t rx_buf[BufferSize] = {0};

    void setup(USART_TypeDef *usart, GPIO_TypeDef *usart_tx, uint8_t tx_pin, GPIO_TypeDef *usart_rx, uint8_t rx_pin, uint16_t baudrate);

    //↓あとで消す
    void setup(USART_TypeDef *usart, GPIO *tx, GPIO *rx, uint16_t baudrate);
    void clockEnable(void);
    void setAlternate(void);

    void UsartEnable(void);
    void TxEnable(void);
    void RxEnable(void);

    void UsartDisable(void);
    void TxDisable(void);
    void RxDisable(void);

    void isUsartEnable(void);
    void isTxEnable(void);
    void isRxEnable(void);

	//int putchar(uint8_t c);
	int putchar(uint8_t c);
	//int getchar(void);
	int getchar(void);
	int write(const uint8_t *data, int size);
	int read(uint8_t *data, int size);
	//int printf(const char *format, ...);
	int printf(const char *format, ...);

private:
	USART_TypeDef *USARTx;
	GPIO *tx;
	GPIO *rx;

	//uint16_t baudrate;

};



#endif /* USART_DMA_HPP_ */
