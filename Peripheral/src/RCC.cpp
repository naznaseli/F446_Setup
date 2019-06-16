/*
 * RCC.cpp
 *
 *  Created on: 2019/05/02
 *      Author: Shibata
 */

#include "stm32f446xx.h"
#include "RCC.hpp"

//:**********************************************************************
//!
//! @brief  Clock configuration
//!
//! @note
//!         HSE                 : 12
//!         SYSCLK              : 180
//!         HCLK                : 180
//!         PCLK1 peripheral    : 45
//!         PCLK1 timer         : 90
//!         PCLK2 peripheral    : 90
//!         PCLK2 timer         : 180
//!
//!         (Unit:MHz)
//:**********************************************************************
void RCC_Setup(void){
    //PLLOFF
    RCC->CR &= ~RCC_CR_PLLON;
    while(RCC->CR & RCC_CR_PLLRDY);

    //HSEON
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));

    //PLLM      1/6
    //PLLN      *180
    //PLLP      1/2
    //PLLSRC    HSE
    //PLLQ      1/2
    //PLLR      1/2
    //RCC->CFGR = 0b00100010010000000010110100000110;
    //RCC->PLLCFGR = 0x22402D06;    //180MHz
    //RCC->PLLCFGR = 0x22402A06;    //168MHz
    //RCC->PLLCFGR = 0x22401206;  //72MHz
    RCC->PLLCFGR = 0x22402406;    //144MHz

    //MCO�g��Ȃ�
    //PPRE2 1/2
    //PPRE1 1/4
    //HPRE  1/1
    //SW    PLL_P
    RCC->CFGR = 0x00009402;
    FLASH->ACR |= FLASH_ACR_LATENCY_4WS;

    //PLLON
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));
}
