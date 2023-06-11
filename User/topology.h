/*
 BitzOS (BOS) V0.2.7 - Copyright (C) 2017-2022 Hexabitz
 All rights reserved

 File Name     : topology.h
 Description   : Array topology definition.

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef topology_H
#define __topology_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

#define __N  2         // Number of array modules

// Array modules
#define _mod1  1<<3
#define _mod2  2<<3

// Topology
static uint16_t array[__N ][7] ={
 {_H0FR7, _mod5 | P1,0,0, _mod2 | P1, 0, 0},                         // Module 1      e
 {_H18R1,  _mod1 | P4, 0,0,_mod3 | P1 , 0, 0},              // Module 2      y
};

// Configurations for duplex serial ports
#if ( _module == 1 )
  #define  H0FR7  1
  #define  _P1pol_normal  1
  #define  _P2pol_normal  1
  #define  _P3pol_normal  1
  #define  _P4pol_normal  1
  #define  _P5pol_normal  1
  #define  _P6pol_normal  1
#endif

#if ( _module == 2 )
  #define  H18R1  1
  #define  _P1pol_reversed  1
  #define  _P2pol_normal  1
  #define  _P3pol_normal  1
  #define  _P4pol_normal  1
  #define  _P5pol_normal  1
  #define  _P6pol_normal  1
#endif

#if ( _module == 3 )
  #define  H18R1  1
  #define  _P1pol_reversed  1
  #define  _P2pol_normal  1
  #define  _P3pol_normal  1
  #define  _P4pol_normal  1
  #define  _P5pol_normal  1
  #define  _P6pol_normal  1
#endif

#if ( _module == 4 )
  #define  H18R1  1
  #define  _P1pol_reversed  1
  #define  _P2pol_normal  1
  #define  _P3pol_normal  1
  #define  _P4pol_normal  1
  #define  _P5pol_normal  1
  #define  _P6pol_normal  1
#endif

#if ( _module == 5 )
  #define  _H0FR7   1
  #define  _P1pol_reversed  1
  #define  _P2pol_normal  1
  #define  _P3pol_normal  1
  #define  _P4pol_normal  1
  #define  _P5pol_normal  1
  #define  _P6pol_normal  1
#endif

#ifdef __cplusplus
}
#endif
#endif /* topology_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
