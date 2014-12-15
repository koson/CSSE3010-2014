/**
  ******************************************************************************
  * @file    netconfig.h
  * @author  MDS
  * @date    22-April-2014 
  * @brief   This file contains MAC, IP, MASK and Gateway address definitions.
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NETCONFIG_H
#define __NETCONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif
 
/* MAC ADDRESS*/
#define MAC_ADDR0   0x02
#define MAC_ADDR1   0x67
#define MAC_ADDR2   0x00
#define MAC_ADDR3   0x4A
#define MAC_ADDR4   0x86
#define MAC_ADDR5   0x5C
 
/*Static IP ADDRESS*/
#define IP_ADDR0   192
#define IP_ADDR1   168
#define IP_ADDR2   0
#define IP_ADDR3   10
   
/*NETMASK*/
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0

/*Gateway Address*/
#define GW_ADDR0   192
#define GW_ADDR1   168
#define GW_ADDR2   0
#define GW_ADDR3   1  

#ifdef __cplusplus
}
#endif

#endif /* __NETCONFIG_H */


