/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.h
  * @version        : v1.0_Cube
  * @brief          : Header for usb_device.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_DEVICE__H__
#define __USB_DEVICE__H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "usbd_def.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup USBD_OTG_DRIVER
  * @{
  */

/** @defgroup USBD_DEVICE USBD_DEVICE
  * @brief Device file for Usb otg low level driver.
  * @{
  */

/** @defgroup USBD_DEVICE_Exported_Variables USBD_DEVICE_Exported_Variables
  * @brief Public variables.
  * @{
  */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
 void MX_USB_DEVICE_DeInit(void);

/* USB EFT/lock-up recovery API.
 *
 * The OTG_HS core (and external ULPI PHY) can be left in a stuck state by an
 * EFT burst on nearby cabling. The symptoms are: the device stays "connected"
 * from the host's point of view, but no transfers ever complete and the bus
 * goes silent (no SOF interrupts). The helpers below detect that condition
 * and rebuild the USB stack so the host re-enumerates the device.
 *
 *  - USB_NotifySof()        : called from the SOF ISR; cheap timestamp.
 *  - USB_NotifyTxFailure()  : called by class TX paths whenever SendData
 *                             returns BUSY/FAIL or its completion times out.
 *  - USB_NotifyTxSuccess()  : called when a TX completes normally; resets
 *                             the consecutive-failure counter.
 *  - USB_RecoveryCheck()    : call periodically from the main loop. Triggers
 *                             USB_ForceRecover() when SOF starvation or
 *                             repeated TX failures are observed while the
 *                             device is in the CONFIGURED state.
 *  - USB_ForceRecover()     : tear down + bring back up the USB device stack
 *                             (also pulses the OTG_HS / ULPI peripheral
 *                             clocks so the core/PHY are hardware-reset).
 *  - USB_GetRecoverCount()  : how many recoveries have run since boot
 *                             (useful for diagnostics over UART).
 */
void USB_NotifySof(void);
void USB_NotifyTxFailure(void);
void USB_NotifyTxSuccess(void);
void USB_RecoveryCheck(void);
void USB_ForceRecover(void);
uint32_t USB_GetRecoverCount(void);
/* USER CODE END PFP */

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN VARIABLES */

/* USER CODE END VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_DEVICE_Exported_FunctionsPrototype USBD_DEVICE_Exported_FunctionsPrototype
  * @brief Declaration of public functions for Usb device.
  * @{
  */

/** USB Device initialization function. */
void MX_USB_DEVICE_Init(void);

/*
 * -- Insert functions declaration here --
 */
/* USER CODE BEGIN FD */

/* USER CODE END FD */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USB_DEVICE__H__ */
