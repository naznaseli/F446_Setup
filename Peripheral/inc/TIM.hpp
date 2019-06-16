/*
 * TIM.hpp
 *
 *  Created on: 2019/05/01
 *      Author: Shibata
 */

#ifndef TIM_HPP_
#define TIM_HPP_

#include "stm32f446xx.h"
#include "GPIO.hpp"
#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif
extern void TIM6_DAC_IRQHandler(void);
extern void TIM7_IRQHandler(void);
#ifdef __cplusplus
}
#endif

//apb1 timer : 72MHz
//apb2 timer : 36MHz

class TIM{
public:
    enum MODE{
        TIMER, PWMOUT, ENCODER
    }mode;

    class Channel{
    public:
        Channel(int num);
        int ch_num;
        GPIO *gpio;
        void setDuty(uint16_t duty, int positive);
    };
    Channel *ch1, *ch2, *ch3, *ch4;


    //setup
    void setupTimer(TIM_TypeDef *tim);
    void setupPwmOutput(TIM_TypeDef *tim,
            GPIO_TypeDef *pwm_ch1, uint8_t ch1_pin,
            GPIO_TypeDef *pwm_ch2, uint8_t ch2_pin);
    void setupPwmOutput(TIM_TypeDef *tim,
            GPIO_TypeDef *pwm_ch1, uint8_t ch1_pin,
            GPIO_TypeDef *pwm_ch2, uint8_t ch2_pin,
            GPIO_TypeDef *pwm_ch3, uint8_t ch3_pin,
            GPIO_TypeDef *pwm_ch4, uint8_t ch4_pin);
    void setupEncoder(TIM_TypeDef *tim, GPIO_TypeDef *ch1, uint8_t ch1_pin, GPIO_TypeDef *ch2, uint8_t ch2_pin);

    //common
    void clockEnable(void);
    void setCount(uint16_t value);
    uint16_t getCount(void);
    void resetCount(void);

    void setAlternate(void);

    void countEnable(void);
    void countDisable(void);
    bool isEnable(void);

    //PWM
    void PwmMode1Setup();

private:
    TIM_TypeDef *TIMx;

};

#endif /* TIM_HPP_ */
