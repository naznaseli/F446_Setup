/*
 * GPIO.cpp
 *
 *  Created on: 2019/05/02
 *      Author: Shibata
 */

#include "GPIO.hpp"

GPIO::GPIO(GPIO_TypeDef *gpio, uint8_t pin, Mode mode){
    if(pin < 0 | pin > 15) return;

    this->GPIOx = gpio;
    this->pin = pin;
    //this->mode = mode;

    //clock enable
    if((GPIOx == GPIOA) && (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN))){
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    }
    if((GPIOx == GPIOB) && (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOBEN))){
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    }
    if((GPIOx == GPIOC) && (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOCEN))){
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    }
    if((GPIOx == GPIOD) && (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIODEN))){
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    }
    if((GPIOx == GPIOD) && (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOEEN))){
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    }

    //pin mode configuration
    GPIOx->MODER &= ~(0x3 << pin*2);
    GPIOx->OTYPER &= ~(0x1 << pin);
    GPIOx->OSPEEDR &= ~(0x3 << pin*2);
    GPIOx->PUPDR &= ~(0x3 << pin*2);
    switch(mode){
        case ANALOG:
            GPIOx->MODER |= (0x3 << pin*2);
            break;
        case FLOATING:
            break;
        case INPUT_PU:
            GPIOx->PUPDR |= (0x1 << pin*2);     //pullup
            break;
        case INPUT_PD:
            GPIOx->PUPDR |= (0x2 << pin*2);     //pulldown
            break;
        case PUSHPULL:
            GPIOx->MODER |= (0x1 << pin*2);
            GPIOx->OSPEEDR |= (0x3 << pin*2);   //high speed
            break;
        case OPENDRAIN:
            GPIOx->MODER |= (0x1 << pin*2);
            GPIOx->OTYPER |= (0x1 << pin);
            GPIOx->OSPEEDR |= (0x3 << pin*2);   //high speed
            break;
        case OPENDRAIN_PU:
            GPIOx->MODER |= (0x1 << pin*2);
            GPIOx->OTYPER |= (0x1 << pin);
            GPIOx->OSPEEDR |= (0x3 << pin*2);   //high speed
            GPIOx->PUPDR |= (0x1 << pin*2);     //pullup
            break;
        case ALTERNATE:
            GPIOx->MODER |= (0x2 << pin*2);
            //GPIOx->OSPEEDR |= (0x3 << pin*2);   //いるんか？
            break;
    }
}

void GPIO::setup(GPIO_TypeDef *gpio, uint8_t pin, Mode mode){
    if(pin < 0 | pin > 15) return;

    this->GPIOx = gpio;
    this->pin = pin;
    //this->mode = mode;

    //clock enable
    if((GPIOx == GPIOA) && (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN))){
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    }
    if((GPIOx == GPIOB) && (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOBEN))){
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    }
    if((GPIOx == GPIOC) && (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOCEN))){
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    }
    if((GPIOx == GPIOD) && (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIODEN))){
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    }
    if((GPIOx == GPIOD) && (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOEEN))){
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    }

    //pin mode configuration
    GPIOx->MODER &= ~(0x3 << pin*2);
    GPIOx->OTYPER &= ~(0x1 << pin);
    GPIOx->OSPEEDR &= ~(0x3 << pin*2);
    GPIOx->PUPDR &= ~(0x3 << pin*2);
    switch(mode){
        case ANALOG:
            GPIOx->MODER |= (0x3 << pin*2);
            break;
        case FLOATING:
            break;
        case INPUT_PU:
            GPIOx->PUPDR |= (0x1 << pin*2);     //pullup
            break;
        case INPUT_PD:
            GPIOx->PUPDR |= (0x2 << pin*2);     //pulldown
            break;
        case PUSHPULL:
            GPIOx->MODER |= (0x1 << pin*2);
            GPIOx->OSPEEDR |= (0x3 << pin*2);   //high speed
            break;
        case OPENDRAIN:
            GPIOx->MODER |= (0x1 << pin*2);
            GPIOx->OTYPER |= (0x1 << pin);
            GPIOx->OSPEEDR |= (0x3 << pin*2);   //high speed
            break;
        case OPENDRAIN_PU:
            GPIOx->MODER |= (0x1 << pin*2);
            GPIOx->OTYPER |= (0x1 << pin);
            GPIOx->OSPEEDR |= (0x3 << pin*2);   //high speed
            GPIOx->PUPDR |= (0x1 << pin*2);     //pullup
            break;
        case ALTERNATE:
            GPIOx->MODER |= (0x2 << pin*2);
            GPIOx->OSPEEDR |= (0x3 << pin*2);   //high speed
            break;
    }
}

void GPIO::setAlternate(uint8_t alternate){
    if(pin <= 7){   //AFRL
        GPIOx->AFR[0] &= ~(alternate << pin*4);
        GPIOx->AFR[0] |= alternate << pin*4;
    }else{          //AFRH
        GPIOx->AFR[1] &= ~(alternate << (pin-8)*4);
        GPIOx->AFR[1] |= alternate << (pin-8)*4;
    }
}

void GPIO::write(uint8_t value){
    if(value)
        GPIOx->ODR |= (0x0001 << pin);
    else
        GPIOx->ODR &= ~(0x0001 << pin);
}

uint8_t GPIO::read(void){
    return (GPIOx->IDR >> pin) & 0x01;

}
void GPIO::toggle(void){
    if(read())
        write(0);
    else
        write(1);
}
