/*
 * TIM.cpp
 *
 *  Created on: 2019/05/01
 *      Author: Shibata
 */

#include "TIM.hpp"
#include "CatchRobo2019.hpp"
#include <stdio.h>
#include "SerialPort.hpp"

extern void Interrupt_1ms(void);
extern void Interrupt_5ms(void);
//extern void Interrupt_10us(void);

void TIM6_DAC_IRQHandler(void);
void TIM7_IRQHandler(void);

void TIM6_DAC_IRQHandler(void){
    TIM6->SR = 0;
    Interrupt_1ms();
}

void TIM7_IRQHandler(void){
    TIM7->SR = 0;
    Interrupt_5ms();
}

//setupの共通部分
//void TIM::setup(void){
//}

void TIM::clockEnable(void){
    if(TIMx == TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    if(TIMx == TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    if(TIMx == TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    if(TIMx == TIM4) RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    if(TIMx == TIM5) RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    if(TIMx == TIM6) RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    if(TIMx == TIM7) RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    if(TIMx == TIM8) RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    if(TIMx == TIM9) RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
    if(TIMx == TIM10) RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
    if(TIMx == TIM11) RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
    if(TIMx == TIM12) RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
    if(TIMx == TIM13) RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
    if(TIMx == TIM14) RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
}

void TIM::setupTimer(TIM_TypeDef *tim){
    this->TIMx = tim;
    clockEnable();

    //抽象化ガン無視コンフィギュレーション
    if(TIMx == TIM6){
        TIMx->CR1 = 0x0000;
        TIMx->DIER |= TIM_DIER_UIE;

        //1ms period
        TIMx->PSC = 120-1;
        TIMx->ARR = 600-1;

        NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }

    if(TIMx == TIM7){
        TIMx->CR1 = 0x0000;
        TIMx->DIER |= TIM_DIER_UIE;

        //5ms period
        TIMx->PSC = 600-1;
        TIMx->ARR = 600-1;

        NVIC_EnableIRQ(TIM7_IRQn);
    }
    resetCount();
    countEnable();
}

void TIM::setAlternate(void){
    if((TIMx == TIM1) | (TIMx == TIM2)){
        if(ch1 != NULL) ch1->gpio->setAlternate(AF1);
        if(ch2 != NULL) ch2->gpio->setAlternate(AF1);
        if(ch3 != NULL) ch3->gpio->setAlternate(AF1);
        if(ch4 != NULL) ch4->gpio->setAlternate(AF1);
    }
    if((TIMx == TIM3) | (TIMx == TIM4) | (TIMx == TIM5)){
        if(ch1 != NULL) ch1->gpio->setAlternate(AF2);
        if(ch2 != NULL) ch2->gpio->setAlternate(AF2);
        if(ch3 != NULL) ch3->gpio->setAlternate(AF2);
        if(ch4 != NULL) ch4->gpio->setAlternate(AF2);
    }
    if((TIMx == TIM8) | (TIMx == TIM9) | (TIMx == TIM10) | (TIMx == TIM11)){
        if(ch1 != NULL) ch1->gpio->setAlternate(AF3);
        if(ch2 != NULL) ch2->gpio->setAlternate(AF3);
        if(ch3 != NULL) ch3->gpio->setAlternate(AF3);
        if(ch4 != NULL) ch4->gpio->setAlternate(AF3);
    }
    if((TIMx == TIM12) | (TIMx == TIM13) | (TIMx == TIM14)){
        if(ch1 != NULL) ch1->gpio->setAlternate(AF9);
        if(ch2 != NULL) ch2->gpio->setAlternate(AF9);
        if(ch3 != NULL) ch3->gpio->setAlternate(AF9);
        if(ch4 != NULL) ch4->gpio->setAlternate(AF9);
    }
}

//pwm 1ch

//pwm 2ch
//使うチャンネルを選択制にする
void TIM::setupPwmOutput(TIM_TypeDef *tim,
        GPIO_TypeDef *pwm_ch1, uint8_t ch1_pin,
        GPIO_TypeDef *pwm_ch2, uint8_t ch2_pin, uint16_t period){

    this->TIMx = tim;
    this->period = period;
    ch1 = new Channel(tim, 1, pwm_ch1, ch1_pin, period);
    ch2 = new Channel(tim, 2, pwm_ch2, ch2_pin, period);

    clockEnable();
    setAlternate();

    TIMx->CR1 |= TIM_CR1_ARPE;
    TIMx->CCMR1 |= 0x6868;  //PWM1 mode

    //これ使うにはchを選択できるようにしないといけない
    TIMx->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);  //ピン出力

    TIMx->PSC = 48-1;
    TIMx->ARR = 60000-1;    //50Hz, 20ms
    TIMx->CCR1 = 3000-1;    //1ms
    TIMx->CCR2 = 3000-1;    //1ms

    TIMx->EGR |= TIM_EGR_UG;
    countEnable();
}

//pwm 4ch
//periodの設定をどうするか
//晴天逆転どこでいじれるか
void TIM::setupPwmOutput(TIM_TypeDef *tim,
        GPIO_TypeDef *pwm_ch1, uint8_t ch1_pin,
        GPIO_TypeDef *pwm_ch2, uint8_t ch2_pin,
        GPIO_TypeDef *pwm_ch3, uint8_t ch3_pin,
        GPIO_TypeDef *pwm_ch4, uint8_t ch4_pin, uint16_t period){

    this->TIMx = tim;
    this->period = period;
    ch1 = new Channel(tim, 1, pwm_ch1, ch1_pin, period);
    ch2 = new Channel(tim, 2, pwm_ch2, ch2_pin, period);
    ch3 = new Channel(tim, 3, pwm_ch3, ch3_pin, period);
    ch4 = new Channel(tim, 4, pwm_ch4, ch4_pin, period);

    clockEnable();
    setAlternate();

    TIMx->CR1 |= TIM_CR1_ARPE;


    //period条件判定
    //使うTIMによってクロック周波数が異なるので場合分け
    TIMx->PSC = 24-1;
    TIMx->ARR = 60000-1;    //50Hz, 20ms

    TIMx->CCMR1 |= 0x0068;  //ch1 PWM1 mode
    TIMx->CCER |= TIM_CCER_CC1E;    //ピン出力
    TIMx->CCR1 = 0;    //1ms

    TIMx->CCMR1 |= 0x6800;  //ch2 PWM1 mode
    TIMx->CCER |= TIM_CCER_CC2E;
    TIMx->CCR2 = 0;    //1ms

    TIMx->CCMR2 |= 0x0068;  //ch3 PWM1 mode
    TIMx->CCER |= TIM_CCER_CC3E;
    TIMx->CCR3 = 3000-1;

    TIMx->CCMR2 |= 0x6800;  //ch4 PWM1 mode
    TIMx->CCER |= TIM_CCER_CC4E;
    TIMx->CCR4 = 3000-1;
    //**********************************************************************

    TIMx->EGR |= TIM_EGR_UG;
    if(TIMx == TIM1){
        TIMx->BDTR &= ~(TIM_BDTR_OSSR);
        TIMx->BDTR |= TIM_BDTR_MOE;
        //TIMx->BDTR |= TIM_BDTR_OSSI;
        //TIMx->BDTR |= TIM_BDTR_AOE;
        //TIMx->BDTR |= TIM_BDTR_BKP;
        //TIMx->BDTR |= TIM_BDTR_BKE;
    }
    countEnable();
}

void TIM::setupEncoder(TIM_TypeDef *tim, GPIO_TypeDef *enc_ch1, uint8_t ch1_pin, GPIO_TypeDef *enc_ch2, uint8_t ch2_pin, uint16_t period){
    this->TIMx = tim;
    this->period = period;

    ch1 = new Channel(tim, 1, enc_ch1, ch1_pin);
    ch2 = new Channel(tim, 2, enc_ch2, ch2_pin);
    //ch1 = new Channel(tim, 1);
    //ch2 = new Channel(tim, 2);
    //ch1->gpio = new GPIO(enc_ch1, ch1_pin, GPIO::ALTERNATE);
    //ch2->gpio = new GPIO(enc_ch2, ch2_pin, GPIO::ALTERNATE);

    clockEnable();
    setAlternate();

    TIMx->SMCR = 0x0003;    //encoder mode 3
    TIMx->CCMR1 |= 0x0101;
    TIMx->ARR = 65535;
    countEnable();
}

void TIM::setCount(uint16_t value){
    TIMx->CNT = value;
}

uint16_t TIM::getCount(void){
    return TIMx->CNT;
}

void TIM::resetCount(void){
    setCount(0);
}

void TIM::countEnable(void){
    TIMx->CR1 |= TIM_CR1_CEN;

}
void TIM::countDisable(void){
    TIMx->CR1 &= ~TIM_CR1_CEN;
}

bool TIM::isEnable(void){
    if(TIMx->CR1 & TIM_CR1_CEN) return true;
    return false;
}

TIM::Channel::Channel(TIM_TypeDef *tim, int num, GPIO_TypeDef *ch_gpio, uint8_t pin){
    this->TIMx = tim;
    if(!(num == 1 || num == 2 || num == 3 || num == 4)){
        return;
    }
    ch_num = num;

    gpio = new GPIO(ch_gpio, pin, GPIO::ALTERNATE);
}

//for pwmoutpu
TIM::Channel::Channel(TIM_TypeDef *tim, int num, GPIO_TypeDef *ch_gpio, uint8_t pin, uint16_t period){
    this->TIMx = tim;
    this->period = period;
    if(!(num == 1 || num == 2 || num == 3 || num == 4)){
        return;
    }
    ch_num = num;

    gpio = new GPIO(ch_gpio, pin, GPIO::ALTERNATE);
}

//void TIM::Channel::setupPwmOutput(){
//
//}
//:**********************************************************************
//!
//! @param
//!     duty:   デューティ比、-32767~32767(32768は使わない)
//!
//:**********************************************************************

//void TIM::Channel::setDuty(uint16_t duty){
//
//
//}

//:**********************************************************************
//!
//! @param
//!     duty:   デューティ比、0~32767
//!     direction:  回転方向、1:正回転(停止)、0:逆回転
//!
//:**********************************************************************
void TIM::Channel::setDuty(uint16_t duty, int direction){
    if(duty > period) duty = period;  //100%になるからよろしくない

    //回転方向設定
    if(direction)
    {  //positive(pwm mode 1)
        switch(ch_num){
            case 1:
                TIMx->CCMR1 &= ~(0x0070);
                TIMx->CCMR1 |= 0x0060;
                break;
            case 2:
                TIMx->CCMR1 &= ~(0x0070);
                TIMx->CCMR1 |= 0x6000;
                break;
            case 3:
                TIMx->CCMR2 &= ~(0x0070);
                TIMx->CCMR2 |= 0x0060;
                break;
            case 4:
                TIMx->CCMR2 &= ~(0x0070);
                TIMx->CCMR2 |= 0x6000;
                break;
        }

    }else
    {  //negative(pwm mode 2)
        switch(ch_num){
            case 1:
                TIMx->CCMR1 &= ~(0x0070);
                TIMx->CCMR1 |= 0x0070;
                break;
            case 2:
                TIMx->CCMR1 &= ~(0x0070);
                TIMx->CCMR1 |= 0x7000;
                break;
            case 3:
                TIMx->CCMR2 &= ~(0x0070);
                TIMx->CCMR2 |= 0x0070;
                break;
            case 4:
                TIMx->CCMR2 &= ~(0x0070);
                TIMx->CCMR2 |= 0x7000;
                break;
        }
    }

    //-1？
    switch(ch_num){
        case 1:
            TIMx->CCR1 = (uint32_t)(duty);
            break;
        case 2:
            TIMx->CCR2 = (uint32_t)(duty);
            break;
        case 3:
            TIMx->CCR3 = (uint32_t)(duty);
            break;
        case 4:
            TIMx->CCR4 = (uint32_t)(duty);
            break;
    }
}

//:**********************************************************************
//!
//! @param
//!     duty: デューティ比、0-100.0(%)
//:**********************************************************************
void TIM::Channel::setDuty(int duty, int direction){
    uint16_t uduty = (uint16_t)(duty * 0.01 * period);
    //setDuty(uduty, direction);
    setDuty((uint16_t)(duty*600-1), direction);
}

void TIM::Channel::resetDuty(void){
    //setDuty(0, 1);
}
