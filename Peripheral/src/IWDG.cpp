/*
 * IWDG.cpp
 *
 *  Created on: 2019/05/08
 *      Author: Shibata
 */

#include "stm32f446xx.h"

//低速クロック40kHz使用
//LSI
//メインクロックに異常があった場合でも動作
//時間精度は高くない(30~60kHzまでありうる)

//IWDG_KRレジスタにCCCChが書き込まれる→カウントダウンスタート
//FFFFh -> 0000h    →IWDGReset発生

//KRにAAAAhが書き込まれる →RLRレジスタの値が再びカウンタにロード

//ハードウェアウォッチドッグ
//電源投入と同時に有効

//PRレジスタ、RLRレジスタは書き込み保護あり

void IWDG_Setup(void){
    //アクセス保護
    IWDG->KR = 0x5555;  //アクセス保護解除
    while((IWDG->SR & IWDG_SR_PVU) == IWDG_SR_PVU);
    IWDG->PR = 0x7; //256分周
    while((IWDG->SR & IWDG_SR_RVU) == IWDG_SR_RVU);
    IWDG->RLR = 0x0FFF; //再ロード値設定

    //タイムアウト26214ms
    //あとで30hz60hzでも安定して1ms割り込みでタイムアウトを防げるできるだけ小さい値を設定

    //Count start
    IWDG->KR = 0xCCCC;
}

void IWDG_Reset(void){
    IWDG->KR = 0xAAAA;
}
