/*
 * CAN.hpp
 *
 *  Created on: 2019/05/01
 *      Author: Shibata
 */

#ifndef CAN_HPP_
#define CAN_HPP_

#include "GPIO.hpp"
#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif
void CAN1_RX0_IRQHandler(void);
void CAN1_RX1_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);
void CAN2_RX1_IRQHandler(void);
#ifdef __cplusplus
}
#endif

//自作した方がよさそう
//TXとRX
typedef struct{
    uint16_t StdId;
    uint32_t ExtId;
    uint8_t IDE;
    uint8_t RTR;
    uint8_t DLC;
    uint8_t Data[8];
}CanMsg;


class bxCAN{
public:
    enum OPERATING_MODE{
        SLEEP,
        INITIALIZATION,
        NORMAL
    };

    //setup
    void setup(CAN_TypeDef *can, GPIO_TypeDef *can_tx, uint8_t tx_pin, GPIO_TypeDef *can_rx, uint8_t rx_pin);
    void CAN_Setup(void);

    //mode
    void modeTransition(OPERATING_MODE mode);

    void clockEnalbe(void);
    void setAlternate(void);
    void clockEnable(void);
    void reset(uint16_t address);
    void filterAdd();
    void send(uint16_t id, uint8_t length, uint8_t *data);
    void receive();

    void transmit(CanMsg *txMessage);   //クラス
    void transmit(uint16_t id, uint8_t length, uint8_t data[]);

    uint64_t latestReceivedTime;

private:
    CAN_TypeDef *CANx;
    GPIO *tx;
    GPIO *rx;
};

#endif /* CAN_HPP_ */
