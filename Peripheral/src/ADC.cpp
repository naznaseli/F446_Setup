/*
 * ADC.cpp
 *
 *  Created on: 2019/05/02
 *      Author: Shibata
 */

#include "ADC.hpp"
#include "stm32f446xx.h"
#include "CatchRobo2019.hpp"

void ADC_Setup(void);

void ADC_Setup(void){
    //PA4,PA5,PC4,PC5
    //IN4:PA4
    //IN5:PA5
    //IN14:PC4
    //IN15:PC5
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    //CR1
    //ADC CR1 DISCEN = 1    //for regular group
    //ADC1->CR1 |= (ADC_CR1_OVRIE | ADC_CR1_SCAN | ADC_CR1_EOCIE);
    ADC1->CR1 |= (ADC_CR1_SCAN | ADC_CR1_EOCIE);

    //CR2
    //ADC1->CR2 |= (ADC_CR2_EOCS | ADC_CR2_CONT);
    ADC1->CR2 |= (ADC_CR2_EOCS | ADC_CR2_DDS | ADC_CR2_DMA | ADC_CR2_CONT);
    //ADC1->CR2 |= (ADC_CR2_EOCS | ADC_CR2_DMA | ADC_CR2_CONT);
    //ADC1->CR2 |= ADC_CR2_CONT;

    //SMPR1,2
    uint8_t sample_time = 1;
    ADC1->SMPR1 |= 0x00000000;   //sampling time ; 3 cycles
    //ADC1->SMPR2 |= 0x00000029;   //sampling time ; 3 cycles
    ADC1->SMPR2 = ((sample_time & 0x7) << 9 | (sample_time & 0x7) << 6 | (sample_time & 0x7) << 3 | (sample_time & 0x7));

    //Select the channel which use
    uint8_t sequence_length = 4;
    uint8_t sq[sequence_length] = {4,5,14,15};

    ADC1->SQR1 |= ((sequence_length-1) & 0x0F) << 20;

    switch(sequence_length){
        case 16:
        case 15:
        case 14:
        case 13:
            for(int i = 12; i < sequence_length; i++){
                ADC1->SQR1 |= (sq[i] & 0x1F) << 5*(i-12);
            }
            //fallthrough
        case 12:
        case 11:
        case 10:
        case 9:
        case 8:
        case 7:
            for(int i = 6; i < sequence_length; i++){
                ADC1->SQR2 |= (sq[i] & 0x1F) << 5*(i-6);
            }
            //fallthrough
        case 6:
        case 5:
        case 4:
        case 3:
        case 2:
        case 1:
        case 0:
            for(int i = 0; i < sequence_length; i++){
                ADC1->SQR3 |= (sq[i] & 0x1F) << 5*i;
            }
            //fallthrough
        default:
            break;
    }

    //DMA
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;  //peripheral address
    DMA2_Stream0->M0AR = (uint32_t)(analog+CIRCUIT_ID*4);      //memory address
    DMA2_Stream0->NDTR = 4; //number of data items to transfer
    //DMA2_Stream0->CR |= (DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_CIRC);
    DMA2_Stream0->CR |= (DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_CIRC);
    DMA2_Stream0->CR |= DMA_SxCR_MINC;  //MSIZE�ݒ肵����ɐݒ�
    DMA2_Stream0->CR |= DMA_SxCR_EN;    //Channel enabled

    ADC1->CR2 |= ADC_CR2_ADON;      //powered on
    ADC1->CR2 |= ADC_CR2_SWSTART;   //start conversion of regular channels
}
