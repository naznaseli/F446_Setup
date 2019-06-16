/*
 * USART_DMA.cpp
 *
 *  Created on: 2019/05/10
 *      Author: Shibata
 */

#include "USART_DMA.hpp"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

void DMA2_Stream7_IRQHandler(void); //usart1_tx
void DMA1_Stream6_IRQHandler(void); //usart2_tx
void DMA1_Stream3_IRQHandler(void); //usart3_tx

void DMA1_Stream3_IRQHandler(void){
    //if(DMA1->HISR & DMA_HISR_TCIF3){
    //    //DMA1->HIFCR |= DMA_HIFCR_CTCIF3;
    //}

    //こっち多分TX
//    if(DMA1->ISR & DMA_ISR_TCIF7){
//        DMA1->IFCR |= DMA_IFCR_CTCIF7;
//        DMA1_Channel7->CCR &= ~(DMA_CCR_EN);
//    }
}

//全部f103の設定なのでF446っ用に新しく設定する必要
    //アンダーバーの方は最初に変な文字化けとかしない？
    //原因わからん

//:**********************************************************************
//!
//! @brief  USART configuration
//!
//:**********************************************************************
void USART_DMA::setup(USART_TypeDef *usart, GPIO_TypeDef *usart_tx, uint8_t tx_pin, GPIO_TypeDef *usart_rx, uint8_t rx_pin, uint16_t baudrate){
    this->USARTx = usart;
    tx->setup(usart_tx, tx_pin, GPIO::FLOATING);
    rx->setup(usart_rx, rx_pin, GPIO::FLOATING);

    //this->baudrate = baudrate;

    clockEnable();
    setAlternate();

    //remap

    //RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    //GPIOB->AFR[0] |= 0x77000000;    //AF7

    //抽象化ガン無視設定
    //DMAの設定
    USART1->BRR = 0x00000271;   //115200bps
    if(USARTx == USART1)
        NVIC_EnableIRQ(USART1_IRQn);
    if(USARTx == USART3){
        NVIC_EnableIRQ(USART3_IRQn);
    }
    //USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    //USART1->CR1 |= USART_CR1_UE;
    TxEnable();
    RxEnable();
    UsartEnable();
}

void USART_DMA::clockEnable(void){
    if(USARTx == USART1)
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    if(USARTx == USART2)
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    if(USARTx == USART3)
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    if(USARTx == UART4)
        RCC->APB2ENR |= RCC_APB1ENR_UART4EN;
    if(USARTx == UART5)
        RCC->APB2ENR |= RCC_APB1ENR_UART5EN;
    if(USARTx == USART6)
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
}
void USART_DMA::setAlternate(void){
    //5不明
    if((USARTx == USART1) | (USARTx == USART2) | (USARTx == USART3)){
        tx->setAlternate(AF7);
    }
    if((USARTx == UART4) | (USARTx == USART6)){
        tx->setAlternate(AF8);
    }
}

void USART_DMA::setup(USART_TypeDef *usart, GPIO *tx, GPIO *rx, uint16_t baudrate){
//    tx_buf_ptr = 0;
//    rx_buf_ptr = 0;
//
//    //USART1 remap config
//    //PB6,PB7
//    if(tx->GPIOx == GPIOB && tx->pin == 6 && rx->GPIOx == GPIOB && rx->pin == 7){
//        //remap configuration
//        //USART1はデフォルトでremapされている(?)
//        //2019/05/08　データシート見たけどリセット状態で0って書いてある
//        AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;
//    }else{
//        AFIO->MAPR &= ~(AFIO_MAPR_USART1_REMAP);
//    }
//
//    //ここ抽象化
//    //ストップビットとかパリティとか設定するprivate関数
//    //USARTConfig(stopbit parity, baudrate, );
//    //クロック設定と設定ボーレートの値からBRRの値を算出
//    if(USARTx == USART1){
//        USARTx->BRR = 0x00000271;   //72*100000/115200 = 625 = 0x271
//        USARTx->CR1 |= USART_CR1_TE;
//        USARTx->CR1 |= USART_CR1_RE;
//        //TxEnable();
//        USARTx->CR1 |= USART_CR1_UE;
//        //UsartEnable();
//    }
//
//    if(USARTx == USART2){
//        USARTx->BRR = 0x00000139;   //115200bps
//
//        RCC->AHBENR |= RCC_AHBENR_DMA1EN;
//
//        //TX DMA config
//        USARTx->CR3 |= USART_CR3_DMAT;
//        DMA1_Channel7->CPAR = (uint32_t)&USARTx->DR;
//        DMA1_Channel7->CMAR = (uint32_t)tx_buf;
//        //DMA1_Channel7->CNDTR = BufferSize;
//        //DMA1_Channel7->CCR |= DMA_CCR_CIRC;
//        DMA1_Channel7->CCR |= DMA_CCR_MINC;
//        DMA1_Channel7->CCR |= DMA_CCR_TCIE;
//        DMA1_Channel7->CCR |= DMA_CCR_DIR;
//        NVIC_EnableIRQ(DMA1_Channel7_IRQn);
//        //DMA1_Channel7->CCR |= DMA_CCR_EN;   //enable
//
//        //RX DMA config
//        //USARTx->CR3 |= USART_CR3_DMAR;
//        //DMA1_Channel6->CPAR = (uint32_t)&USARTx->DR;
//        //DMA1_Channel6->CMAR = (uint32_t)rx_buf;
//        //DMA1_Channel6->CNDTR = BufferSize;
//        //DMA1_Channel6->CCR |= DMA_CCR_CIRC;
//        //DMA1_Channel6->CCR |= DMA_CCR_MINC;
//        ////DMA1_Channel7->CCR |= DMA_CCR_TCIE;
//        //DMA1_Channel6->CCR |= DMA_CCR_DIR;
//        //NVIC_EnableIRQ(DMA1_Channel6_IRQn);
//        //DMA1_Channel6->CCR |= DMA_CCR_EN;   //enable
//
//        //普通の受信
//        USARTx->CR1 |= USART_CR1_RXNEIE;
//        NVIC_EnableIRQ(USART2_IRQn);
//
//        USARTx->CR1 |= USART_CR1_TE;
//        USARTx->CR1 |= USART_CR1_RE;
//        USARTx->CR1 |= USART_CR1_UE;
//    }
//
//    if(USARTx == USART3){
//        ;
//    }
}

void USART_DMA::UsartEnable(void){
    USARTx->CR1 |= USART_CR1_UE;
}

void USART_DMA::TxEnable(void){
    USARTx->CR1 |= USART_CR1_TE;
}

void USART_DMA::RxEnable(void){
    USARTx->CR1 |= USART_CR1_RE;
}


void USART_DMA::UsartDisable(void){
    USARTx->CR1 &= ~(USART_CR1_UE);
}
void USART_DMA::TxDisable(void){
    USARTx->CR1 &= ~(USART_CR1_TE);
}
void USART_DMA::RxDisable(void){
    USARTx->CR1 &= ~(USART_CR1_RE);
}

//int USART::putchar(uint8_t c){
//    //while((USARTx->SR & USART_SR_TC) == 0);
//    //USARTx->DR = c;
//    //while((USARTx->SR & USART_SR_TXE) == 0);
//
//    //DMA circular
//    tx_buf[tx_buf_ptr++] = c;
//    tx_buf_ptr &= (BufferSize-1);
//
//    return (int)c;
//}

//DMA not circular
int USART_DMA::putchar(uint8_t c){
    tx_buf[0] = c;
    //DMA1_Channel7->CNDTR = 1;
    //DMA1_Channel7->CCR |= DMA_CCR_EN;

}

//int USART_DMA::getchar(void){
//    uint8_t c;
//    c = USARTx->DR;
//
//    return (int)c;
//}

int USART_DMA::getchar(void){
    //if(rx_buf_ptr != (BufferSize-DMA1_Channel6->CNDTR)){
    //    return rx_buf[rx_buf_ptr++];
    //}else{
        return -1;
    //}

}

int USART_DMA::write(const uint8_t *data, int size){
    const uint8_t *p = data;
    int cnt = 0;

    if(p == NULL) return 0;
    while(size>0){
        putchar(*(p++));
        size--;
        cnt++;
    }

    return cnt;
}

int USART_DMA::read(uint8_t *data, int size){
    int cnt=0;
    while(size > 0){
        getchar();   //読み取り
        size--;
        cnt++;
    }

    return cnt;
}

//int USART::readDMA(uint8_t *data, int size){
//
//}

int USART_DMA::printf(const char *format, ...){
    char buffer[128];
    va_list ap;
    int len;
    va_start(ap, format);
    len = vsprintf(buffer, format, ap);
    va_end(ap);

    if(len <= 128){
        memcpy(tx_buf, buffer, len);
        //DMA1_Channel7->CNDTR = len;
        //DMA1_Channel7->CCR |= DMA_CCR_EN;
    }
}


