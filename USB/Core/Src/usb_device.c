/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device
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

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_comms.h"
#include "usbd_histo.h"
#include "usbd_imu.h"
#include "usbd_composite_builder.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceHS;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

uint8_t COMMS_EpAdd_Inst[3] = {COMMS_IN_EP, COMMS_OUT_EP}; 	/* COMMS Endpoint Addresses array */
uint8_t HISTO_EpAdd_Inst[1] = { HISTO_IN_EP }; 	/* HISTO Endpoint */
uint8_t IMU_EpAdd_Inst[1] = { IMU_IN_EP }; 	/* IMU Endpoint */

uint8_t COMMS_InstID = 0, HISTO_InstID = 0, IMU_InstID = 0;

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* Helper: close a list of non-control endpoints.
 * Safe to call even if EP was never opened; the LL will ignore gracefully.
 */
static void CloseEPList(USBD_HandleTypeDef *pdev, const uint8_t *eps, uint8_t count)
{
  if (!eps) return;
  for (uint8_t i = 0; i < count; ++i) {
    uint8_t ea = eps[i];
    if ((ea & 0x0F) != 0x00) {                 // never try to close EP0
      (void)USBD_LL_CloseEP(pdev, ea);
    }
  }
}


void MX_USB_DEVICE_DeInit(void){

	  /* Block ISR races while we tear down (optional but helpful) */
	  uint32_t primask = __get_PRIMASK();
	  __disable_irq();

	  /* 1) Proactively stop the stack (halts transfers, sets D+ pull-up off) */
	  (void)USBD_Stop(&hUsbDeviceHS);

	  CloseEPList(&hUsbDeviceHS, COMMS_EpAdd_Inst, 2);
	  CloseEPList(&hUsbDeviceHS, HISTO_EpAdd_Inst, 1);
	  CloseEPList(&hUsbDeviceHS, IMU_EpAdd_Inst, 1);

	  /* 3) Fully de-initialize the device core (calls USBD_LL_DeInit → HAL_PCD_DeInit) */
	  (void)USBD_DeInit(&hUsbDeviceHS);

	  /* 4) Symmetry with your Init: turn off the USB voltage detector if you don’t want it left on */
	  HAL_PWREx_DisableUSBVoltageDetector();

	  /* 5) Clear your composite instance bookkeeping (optional but nice) */
	  COMMS_InstID = -1;
	  HISTO_InstID = -1;
	  IMU_InstID   = -1;

	  /* class pointers/callbacks in hUsbDeviceHS, clear them here.
	     Example:
	     // hUsbDeviceHS.pClass = NULL;
	     // hUsbDeviceHS.pClassData = NULL;
	  */
	  USBD_UnRegisterClassComposite(&hUsbDeviceHS);

	  /* Restore IRQ state */
	  if (!primask) __enable_irq();
}

/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceHS, &HS_Desc, DEVICE_HS) != USBD_OK)
  {
    Error_Handler();
  }

  /* Store the COMMS Class */
  COMMS_InstID = hUsbDeviceHS.classId;

  /* Register CDC Class First Instance */
  if(USBD_RegisterClassComposite(&hUsbDeviceHS, USBD_COMMS_CLASS, CLASS_TYPE_COMMS, COMMS_EpAdd_Inst) != USBD_OK)
  {
	Error_Handler();
  }

  /* Store HISTO Instance Class ID */
  HISTO_InstID = hUsbDeviceHS.classId;

  /* Register the HISTO Class */
  if(USBD_RegisterClassComposite(&hUsbDeviceHS, USBD_HISTO_CLASS, CLASS_TYPE_HISTO, HISTO_EpAdd_Inst) != USBD_OK)
  {
	Error_Handler();
  }

  /* Store HISTO Instance Class ID */
  IMU_InstID = hUsbDeviceHS.classId;

  /* Register the HISTO Class */
  if(USBD_RegisterClassComposite(&hUsbDeviceHS, USBD_IMU_CLASS, CLASS_TYPE_IMU, IMU_EpAdd_Inst) != USBD_OK)
  {
	Error_Handler();
  }


  if (USBD_Start(&hUsbDeviceHS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */
  HAL_PWREx_EnableUSBVoltageDetector();

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

