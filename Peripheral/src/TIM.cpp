/*
 * TIM.cpp
 *
 *  Created on: 2019/05/01
 *      Author: Shibata
 */

#include "TIM.hpp"
#include "CatchRobo2019.hpp"
#include <stdio.h>

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
        if(ch1->gpio != NULL) ch1->gpio->setAlternate(AF1);
        if(ch2->gpio != NULL) ch2->gpio->setAlternate(AF1);
        if(ch3->gpio != NULL) ch3->gpio->setAlternate(AF1);
        if(ch4->gpio != NULL) ch4->gpio->setAlternate(AF1);
    }
    if((TIMx == TIM3) | (TIMx == TIM4) | (TIMx == TIM5)){
        if(ch1->gpio != NULL) ch1->gpio->setAlternate(AF2);
        if(ch2->gpio != NULL) ch2->gpio->setAlternate(AF2);
        if(ch3->gpio != NULL) ch3->gpio->setAlternate(AF2);
        if(ch4->gpio != NULL) ch4->gpio->setAlternate(AF2);
    }
    if((TIMx == TIM8) | (TIMx == TIM9) | (TIMx == TIM10) | (TIMx == TIM11)){
        if(ch1->gpio != NULL) ch1->gpio->setAlternate(AF3);
        if(ch2->gpio != NULL) ch2->gpio->setAlternate(AF3);
        if(ch3->gpio != NULL) ch3->gpio->setAlternate(AF3);
        if(ch4->gpio != NULL) ch4->gpio->setAlternate(AF3);
    }
    if((TIMx == TIM12) | (TIMx == TIM13) | (TIMx == TIM14)){
        if(ch1->gpio != NULL) ch1->gpio->setAlternate(AF9);
        if(ch2->gpio != NULL) ch2->gpio->setAlternate(AF9);
        if(ch3->gpio != NULL) ch3->gpio->setAlternate(AF9);
        if(ch4->gpio != NULL) ch4->gpio->setAlternate(AF9);
    }
}

//pwm 2ch
void TIM::setupPwmOutput(TIM_TypeDef *tim,
        GPIO_TypeDef *pwm_ch1, uint8_t ch1_pin,
        GPIO_TypeDef *pwm_ch2, uint8_t ch2_pin){

    this->TIMx = tim;
    ch1->gpio = new GPIO(pwm_ch1, ch1_pin, GPIO::ALTERNATE);
    ch2->gpio = new GPIO(pwm_ch2, ch2_pin, GPIO::ALTERNATE);

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
void TIM::setupPwmOutput(TIM_TypeDef *tim,
        GPIO_TypeDef *pwm_ch1, uint8_t ch1_pin,
        GPIO_TypeDef *pwm_ch2, uint8_t ch2_pin,
        GPIO_TypeDef *pwm_ch3, uint8_t ch3_pin,
        GPIO_TypeDef *pwm_ch4, uint8_t ch4_pin){

    this->TIMx = tim;
    ch1 = new Channel(1);
    ch2 = new Channel(2);
    ch3 = new Channel(3);
    ch4 = new Channel(4);

    ch1->gpio = new GPIO(pwm_ch1, ch1_pin, GPIO::ALTERNATE);
    ch2->gpio = new GPIO(pwm_ch2, ch2_pin, GPIO::ALTERNATE);
    ch3->gpio = new GPIO(pwm_ch3, ch3_pin, GPIO::ALTERNATE);
    ch4->gpio = new GPIO(pwm_ch4, ch4_pin, GPIO::ALTERNATE);

    clockEnable();
    setAlternate();

    TIMx->CR1 |= TIM_CR1_ARPE;
    TIMx->CCMR1 |= 0x6868;  //PWM1 mode
    TIMx->CCMR2 |= 0x6868;  //PWM1 mode
    TIMx->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);  //pin output

    TIMx->PSC = 24-1;
    TIMx->ARR = 60000-1;    //50Hz, 20ms
    TIMx->CCR1 = 3000-1;    //1ms
    TIMx->CCR2 = 3000-1;    //1ms
    TIMx->CCR3 = 3000-1;
    TIMx->CCR4 = 3000-1;

    TIMx->EGR |= TIM_EGR_UG;
    countEnable();
}

void TIM::setupEncoder(TIM_TypeDef *tim, GPIO_TypeDef *enc_ch1, uint8_t ch1_pin, GPIO_TypeDef *enc_ch2, uint8_t ch2_pin){
    this->TIMx = tim;
    ch1->gpio = new GPIO(enc_ch1, ch1_pin, GPIO::ALTERNATE);
    ch2->gpio = new GPIO(enc_ch2, ch2_pin, GPIO::ALTERNATE);

    clockEnable();
    setAlternate();

    TIMx->SMCR = 0x0003;    //encoder mode 3
    TIMx->CCMR1 |= 0x0101;
    TIMx->ARR = 65535;
    //TIMx->CR1 |= TIM_CR1_CEN;
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

TIM::Channel::Channel(int num){
    if(!(num == 1 || num == 2 || num == 3 || num == 4)){
        return;
    }

    ch_num = num;

    //Channelごとに分割して初期setup
    //ネガポジ注意
    switch(ch_num){
        case 1:
            break;
        case 2:
            break;
        case 3:
            break;
        case 4:
            break;
    }
}

//チャンネルごとに作る必要
//TIM内部にchクラスを作成
void TIM::Channel::setDuty(uint16_t duty, int positive){
    //if(duty > pwm_period) duty = pwm_period;
    if(positive){

    }else{

    }

    //float f_duty = (float)(duty/65535);
    switch(ch_num){
        case 1:
            //TIMx->CCR1 = (uint32_t)(60000*duty-1);
            break;
        case 2:
            break;
        case 3:
            break;
        case 4:
            break;
    }
}
