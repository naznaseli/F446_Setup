/*
 * RCC.hpp
 *
 *  Created on: 2019/05/01
 *      Author: Shibata
 */

#ifndef RCC_HPP_
#define RCC_HPP_

#define SYSCLK  144000000
#define HCLK    144000000
#define PCLK1   36000000
#define PCLK2   72000000
#define TIM1CLK (2*PCLK1)
#define TIM2CLK (2*PCLK2)

#define RCC_HSE 0x00
#define RCC_HSI 0x01
#define RCC_PLLCLK  0x02

//class _RCC{
//public:
//    void setup();
//
//private:
//};
extern void RCC_Setup(void);



#endif /* RCC_HPP_ */
