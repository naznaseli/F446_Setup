/*
 * bxCAN.cpp
 *
 *  Created on: 2019/05/01
 *      Author: Shibata
 */

#include "stm32f446xx.h"
#include "bxCAN.hpp"
#include "CatchRobo2019.hpp"
#include <stdio.h>
#include "SerialPort.hpp"

void CAN2_Transmit();
void CAN2_FilterSetup(void);

void CAN1_RX0_IRQHandler(void){}

void CAN1_RX1_IRQHandler(void){}

void CAN2_RX0_IRQHandler(void){
            usart3_printf("receive\n");
//    //if(can_Enc->rxMessage.StdId == can_Enc_Address){
//    //    can_Enc_value = uchar42int(canEnc_can->rxMessage.Data);
    //receive
    switch(CAN2->RF0R){
        case CAN_RF0R_FMP0:
            led1.write(1);
            break;

            CanMsg rxMessage;
            //いずれCANのメンバに
            //スタックにする？
            //各センサごとに別々にした方が？
            //clearCanMsg();

            rxMessage.StdId = (CAN2->sFIFOMailBox[0].RIR >> 21) & 0x07FF;
            //rxMessage.ExtId = (CAN2->SFIFOMailBox[0].RIR >> 3) & 0x7FFFF;
            //rxMessage.IDE = (CAN2->sFIFOMailBox[0].RIR >> 2) & 0x01;
            //rxMessage.RTR = (CAN2->sFIFOMailBox[0].RIR) & 0x01;
            rxMessage.DLC = (CAN2->sFIFOMailBox[0].RDTR) & 0x0F;
            rxMessage.Data[0] = (CAN2->sFIFOMailBox[0].RDLR) & 0xFF;
            rxMessage.Data[1] = (CAN2->sFIFOMailBox[0].RDLR>>8) & 0xFF;
            rxMessage.Data[2] = (CAN2->sFIFOMailBox[0].RDLR>>16) & 0xFF;
            rxMessage.Data[3] = (CAN2->sFIFOMailBox[0].RDLR>>24) & 0xFF;
            rxMessage.Data[4] = (CAN2->sFIFOMailBox[0].RDHR) & 0xFF;
            rxMessage.Data[5] = (CAN2->sFIFOMailBox[0].RDHR>>8) & 0xFF;
            rxMessage.Data[6] = (CAN2->sFIFOMailBox[0].RDHR>>16) & 0xFF;
            rxMessage.Data[7] = (CAN2->sFIFOMailBox[0].RDHR>>24) & 0xFF;

            canNode.receiveInterrupt(&rxMessage);
            //場所ここで合ってる？
            //1つずつしか更新されないので更新フラグが必要
#ifdef CENTRALIZED_SLAVE
    getActuatorFromCanToLocal();
#endif

            //FIFO release
            CAN2->RF0R &= ~(CAN_RF0R_RFOM0);

            break;

        //FIFO0 full
        case CAN_RF0R_FULL0:
            break;

        //overrun
        case CAN_RF0R_FOVR0:
            break;


       // case CAN_RF0R_RFOM0:
       //     break;
        default:
            break;
    }


}

//RX0の後実装コピペ
void CAN2_RX1_IRQHandler(void){
    usart3_printf("received1\n");
//    //if(can_Enc->rxMessage.StdId == can_Enc_Address){
//    //    can_Enc_value = uchar42int(canEnc_can->rxMessage.Data);
    //receive
    if(CAN2->RF1R & CAN_RF1R_FMP1){
        led1.write(1);
        return;
        //フラグの削除
        //CAN2->MSR = 0;

        CanMsg rxMessage;
        //いずれCANのメンバに
        //スタックにする？
        //各センサごとに別々にした方が？
        //clearCanMsg();

        rxMessage.StdId = (CAN2->sFIFOMailBox[0].RIR >> 21) & 0x07FF;
        //rxMessage.ExtId = (CAN2->SFIFOMailBox[0].RIR >> 3) & 0x7FFFF;
        //rxMessage.IDE = (CAN2->sFIFOMailBox[0].RIR >> 2) & 0x01;
        //rxMessage.RTR = (CAN2->sFIFOMailBox[0].RIR) & 0x01;
        rxMessage.DLC = (CAN2->sFIFOMailBox[0].RDTR) & 0x0F;
        rxMessage.Data[0] = (CAN2->sFIFOMailBox[0].RDLR) & 0xFF;
        rxMessage.Data[1] = (CAN2->sFIFOMailBox[0].RDLR>>8) & 0xFF;
        rxMessage.Data[2] = (CAN2->sFIFOMailBox[0].RDLR>>16) & 0xFF;
        rxMessage.Data[3] = (CAN2->sFIFOMailBox[0].RDLR>>24) & 0xFF;
        rxMessage.Data[4] = (CAN2->sFIFOMailBox[0].RDHR) & 0xFF;
        rxMessage.Data[5] = (CAN2->sFIFOMailBox[0].RDHR>>8) & 0xFF;
        rxMessage.Data[6] = (CAN2->sFIFOMailBox[0].RDHR>>16) & 0xFF;
        rxMessage.Data[7] = (CAN2->sFIFOMailBox[0].RDHR>>24) & 0xFF;

        canNode.receiveInterrupt(&rxMessage);
    }

    //full
    //overrun

}

void bxCAN::setup(CAN_TypeDef *can, GPIO_TypeDef *can_tx, uint8_t tx_pin, GPIO_TypeDef *can_rx, uint8_t rx_pin){
    this->CANx = can;
    tx = new GPIO(can_tx, tx_pin, GPIO::ALTERNATE);
    rx = new GPIO(can_tx, tx_pin, GPIO::ALTERNATE);

    clockEnable();
    setAlternate();

    //RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;
   //GPIOB->AFR[1] |= (0x09 << 16);
   //GPIOB->AFR[1] |= (0x09 << 20);


    CAN_Setup();
}

//500kbps
void bxCAN::CAN_Setup(void){
    modeTransition(INITIALIZATION);

    //BTR
    CANx->BTR |= (6-1) << CAN_BTR_BRP_Pos;
    CANx->BTR |= (7-1) << CAN_BTR_TS1_Pos;
    CANx->BTR |= (4-1) << CAN_BTR_TS2_Pos;
    CANx->BTR |= (1-1) << CAN_BTR_SJW_Pos;

    CANx->BTR |= CAN_BTR_LBKM;    //if loopback mode
    //CANx->BTR |= CAN_BTR_SILM;    //if silent mode

    //CANx->FMR &= ~(CAN_FMR_FINIT);

    //MCR
    //CANx->MCR |= CAN_MCR_NART;    //再送禁止
    //CANx->MCR &= ~(CAN_MCR_DBF);

    //IER
    //CANx->IER |= (CAN_IER_ERRIE | CAN_IER_FOVIE1 | CAN_IER_FFIE1 | CAN_IER_FMPIE1 | CAN_IER_TMEIE);
    //CANx->IER |= (CAN_IER_FMPIE0 | CAN_IER_FMPIE1);
    //CANx->IER |= CAN_IER_FMPIE0;
    //interrupt configuration
    //if(CANx == CAN1){
    //    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    //    NVIC_EnableIRQ(CAN1_RX1_IRQn);
    //}
    //if(CANx == CAN2){
    //    NVIC_EnableIRQ(CAN2_RX0_IRQn);
    //    NVIC_EnableIRQ(CAN2_RX1_IRQn);
    //}

    //enter the normal mode
    modeTransition(NORMAL);
}

void bxCAN::clockEnable(void){
    if(CANx == CAN1) RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    if(CANx == CAN2) RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;
}

void bxCAN::setAlternate(void){
    if(tx == NULL || rx == NULL) return;
    tx->setAlternate(AF9);
    rx->setAlternate(AF9);
}

void bxCAN::modeTransition(OPERATING_MODE mode){
    switch(mode){
        case SLEEP:
            break;

        case INITIALIZATION:
            if(CANx->MSR & CAN_MSR_SLAK)    //if sleep mode now
                CANx->MCR &= ~(CAN_MCR_SLEEP);
            CANx->MCR |= CAN_MCR_INRQ;
            while(((CANx->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) || ((CANx->MSR & CAN_MSR_SLAK) == CAN_MSR_SLAK));
            break;

        case NORMAL:
            if(CANx->MSR & CAN_MSR_SLAK)    //if sleep mode now
                CANx->MCR &= ~(CAN_MCR_SLEEP);
            CANx->MCR &= ~(CAN_MCR_INRQ);
            usart3_printf("before");
            while(((CANx->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) || ((CANx->MSR & CAN_MSR_SLAK) == CAN_MSR_SLAK));
            usart3_printf("after");
            break;

        default:
            //error
            break;
    }
}

void bxCAN::send(uint16_t id, uint8_t length, uint8_t data[8]){
    //送信参考
    //CAN::sendAddress = id;

    CanMsg txMessage;
    txMessage.StdId = id;
    txMessage.ExtId = 0;
    txMessage.IDE   = 0;    //ID is STDID
    if(length == 0){
        txMessage.RTR = 1;  //Remote frame
    }else{
        txMessage.RTR = 0;  //Data frame
    }
    txMessage.DLC = length;
    for(int i = 0; i < 8; i++)
        txMessage.Data[i] = data[i];

    transmit(&txMessage);
}

void bxCAN::receive(void){

}

void bxCAN::reset(uint16_t address){
    uint8_t canData[1];
    canData[0] = 0;
    //canEnc_can->send();
    //send(address,1,canData);
}


void bxCAN::transmit(CanMsg *txMessage){
    uint8_t transmit_mailbox = 0;
    if(CANx->TSR & CAN_TSR_TME0){

    }
    //��̑��M���[���{�b�N�X��T��
    switch(CANx->TSR){
        case CAN_TSR_TME0:
            transmit_mailbox = 0;
            break;
        case CAN_TSR_TME1:
            transmit_mailbox = 1;
            break;
        case CAN_TSR_TME2:
            transmit_mailbox = 2;
            break;
        default:
            //�󂫂��Ȃ�
            return;
    }

    //CANx->sTxMailBox[transmit_mailbox].TIR &= ~(TMIDxR_TXBQ);
    //if(TxMessage->IDE == CAN_Id_Standard){
    //    //stdid
    //    CANx->sTxMailBox[transmit_mailbox].TIR |= ((TxMessage->StdId << 21) | (TxMessage->RTR));
    //}else{
    //    //extid
    //    CANx->sTxMailBox[transmit_mailbox].TIR |= ((TxMessage->ExtId << 3) | (TxMessage->IDE) | (TxMessage->RTR));
    //}

    //Set up the DLC

    //Set up the ata
    //CANx->sTxMailBox[transmit_mailbox].TDLR = ((uint32_t)txMessage->Data[3] << 24) | ((uint32_t)txMessage->Data[2]) | ((uint32_t)txMessage->Data[1] << 8) | ((uint32_t)txMessage->Data[0] << 0);

    //Request transmission
    //CANx->sTxMailBox[transmit_mailbox].TIR |= TMIDxR_TXRQ;
}

void bxCAN::transmit(uint16_t id, uint8_t length, uint8_t data[]){
    uint8_t mailbox;    //0-2

    //check that all the tx mailboxes are not full
    if(CANx->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)){
        mailbox = (CANx->TSR & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;
    }else{
        return;
    }

    //set up the STID
    CANx->sTxMailBox[mailbox].TIR = ((id & 0x7FF) << CAN_TI0R_STID_Pos) | (1 << CAN_TI0R_RTR_Pos);

    //set up the DLC
    CANx->sTxMailBox[mailbox].TDTR = (length & 0xF);

    //set up the data
    CANx->sTxMailBox[mailbox].TDHR = (
            (uint32_t)data[7] << CAN_TDH0R_DATA7_Pos |
            (uint32_t)data[6] << CAN_TDH0R_DATA6_Pos |
            (uint32_t)data[5] << CAN_TDH0R_DATA5_Pos |
            (uint32_t)data[4] << CAN_TDH0R_DATA4_Pos);
    CANx->sTxMailBox[mailbox].TDLR = (
            (uint32_t)data[3] << CAN_TDL0R_DATA3_Pos |
            (uint32_t)data[2] << CAN_TDL0R_DATA2_Pos |
            (uint32_t)data[1] << CAN_TDL0R_DATA1_Pos |
            (uint32_t)data[0] << CAN_TDL0R_DATA0_Pos);

    //request transmission
    CANx->sTxMailBox[mailbox].TIR |= CAN_TI0R_TXRQ;
}
