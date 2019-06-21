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

class TIM{
public:
    enum MODE{
        TIMER, PWMOUT, ENCODER
    }mode;

    //for PWMOUT**********************************************************************
    //for encoder
    class Channel{
    public:
        Channel(TIM_TypeDef *tim, int num, GPIO_TypeDef *ch_gpio, uint8_t pin);
        Channel(TIM_TypeDef *tim, int num, GPIO_TypeDef *ch_gpio, uint8_t pin, uint16_t period);
        int ch_num; //ch1-4
        GPIO *gpio;
        uint16_t period;    //pwm周期
        uint16_t duty;      //duty比
        void setDuty(int duty, int direction);
        void resetDuty(void);   //duty0

    private:
        TIM_TypeDef *TIMx;
        void setDuty(uint16_t duty, int direction);
    };
    Channel *ch1, *ch2, *ch3, *ch4;

    //setup**********************************************************************
    void setupTimer(TIM_TypeDef *tim);
    void setupPwmOutput(TIM_TypeDef *tim,
            GPIO_TypeDef *pwm_ch1, uint8_t ch1_pin,
            GPIO_TypeDef *pwm_ch2, uint8_t ch2_pin, uint16_t period);
    void setupPwmOutput(TIM_TypeDef *tim,
            GPIO_TypeDef *pwm_ch1, uint8_t ch1_pin,
            GPIO_TypeDef *pwm_ch2, uint8_t ch2_pin,
            GPIO_TypeDef *pwm_ch3, uint8_t ch3_pin,
            GPIO_TypeDef *pwm_ch4, uint8_t ch4_pin, uint16_t period);
    void setupEncoder(TIM_TypeDef *tim, GPIO_TypeDef *ch1, uint8_t ch1_pin, GPIO_TypeDef *ch2, uint8_t ch2_pin, uint16_t period);

    //common**********************************************************************
    void clockEnable(void);
    void setCount(uint16_t value);
    uint16_t getCount(void);
    void resetCount(void);

    void setAlternate(void);

    void countEnable(void);
    void countDisable(void);
    bool isEnable(void);

private:
    TIM_TypeDef *TIMx;
    uint16_t period;
};

#endif /* TIM_HPP_ */
