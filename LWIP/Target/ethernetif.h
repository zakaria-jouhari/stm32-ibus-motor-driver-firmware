/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * File Name          : ethernetif.h
  * Description        : This file provides initialization code for LWIP
  *                      middleWare.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__
#include "lwip/err.h"
#include "lwip/netif.h"

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Exported functions ------------------------------------------------------- */
err_t ethernetif_init(struct netif *netif);

void ethernetif_input(struct netif *netif);
void ethernet_link_check_state(struct netif *netif);

void Error_Handler(void);
u32_t sys_jiffies(void);
u32_t sys_now(void);

/* USER CODE BEGIN 1 */
/* Debug-only: link/PHY diagnostics, read by main.c's VCP debug print so the
 * real on-hardware PHY state is directly observable instead of inferred. */
extern volatile uint8_t  eth_dbg_phy_addr;      /* 0xFF = no PHY answered the scan */
extern volatile uint32_t eth_dbg_bsr_at_addr0;  /* raw BSR read at PHY_ADDR (0x00) */
extern volatile uint8_t  eth_dbg_read_ok;       /* did HAL_ETH_ReadPHYRegister at PHY_ADDR succeed? */
extern volatile uint8_t  eth_dbg_link_up;       /* netif_is_link_up() as last seen by ethernet_link_check_state() */
extern volatile uint8_t  eth_dbg_started;       /* has HAL_ETH_Start() actually been called? */
extern volatile uint32_t eth_dbg_mac_speed;      /* ETH_SPEED_10M/100M actually applied to the MAC */
extern volatile uint32_t eth_dbg_mac_duplex;     /* ETH_HALFDUPLEX/FULLDUPLEX_MODE actually applied */
extern volatile uint32_t eth_dbg_rx_frames;      /* count of frames actually surfaced by HAL_ETH_ReadData */
extern volatile uint32_t eth_dbg_error_code;     /* HAL_ETH_GetError(&heth) */
extern volatile uint32_t eth_dbg_dma_error;       /* heth.DMAErrorCode */
extern volatile uint32_t eth_dbg_mac_error;       /* heth.MACErrorCode */
extern volatile uint32_t eth_dbg_tx_attempts;     /* every low_level_output() call, regardless of outcome */
extern volatile uint32_t eth_dbg_tx_ok;           /* how many HAL_ETH_Transmit() calls reported HAL_OK */
extern volatile uint32_t eth_dbg_tx_last_status;  /* most recent HAL_ETH_Transmit() return value */
extern volatile uint8_t  eth_dbg_loopback_active;    /* has eth_debug_enable_loopback() run yet? */
extern volatile uint32_t eth_dbg_loopback_baseline;  /* eth_dbg_rx_frames value captured at that moment */
void eth_debug_enable_loopback(void); /* one-shot: forces LAN8742 internal loopback, see ethernetif.c */
void eth_debug_loopback_send_test_frame(void); /* call periodically once loopback is active */

/* USER CODE END 1 */
#endif
