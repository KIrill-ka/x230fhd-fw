#ifndef COMMON_H
#define COMMON_H

#include <stm8s.h>


#define PWMCAP_TTL (((u16)32)*1)

extern volatile u16 pwmcap_dc_num, pwmcap_dc_denom;
extern volatile u8 pwmcap_st;
extern volatile u16 pwmcap_alive;
extern volatile u16 tim4_tout;
extern volatile u16 tim2_dc;

#define mset(reg, mask, val) (reg) = (reg) & ~(mask) | (val)
//#define bset(reg, bit) (reg) |= (bit)
//#define bclr(reg, bit) (reg) &= ~(bit)
#pragma disable_warning 126
#define bset(reg, bit, val) do {if(val) (reg) |= (bit); else (reg) &= ~(bit);} while(0)

#define BIT0 0b00000001
#define BIT1 0b00000010
#define BIT2 0b00000100
#define BIT3 0b00001000
#define BIT4 0b00010000
#define BIT5 0b00100000
#define BIT6 0b01000000
#define BIT7 0b10000000
#endif
