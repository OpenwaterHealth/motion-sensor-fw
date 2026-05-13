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
#include <stdio.h>
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


void MX_USB_DEVICE_DeInit(void)
{

    /* Stop USB Device */
    if (USBD_Stop(&hUsbDeviceHS) != USBD_OK)
    {
        Error_Handler();
    }

    /* De-initialize the USB Device Library */
    if (USBD_DeInit(&hUsbDeviceHS) != USBD_OK)
    {
        Error_Handler();
    }

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

/* USER CODE BEGIN 2 */

/*============================================================================
 * USB EFT / lock-up recovery
 *
 * During EMC / EFT testing the OTG_HS core (and the external ULPI PHY) can be
 * left in a "deaf" state: the host still sees the device on the bus, but no
 * IN/OUT transactions complete and the bus stops generating SOF interrupts.
 * Without a watchdog the firmware would have to be power-cycled to recover.
 *
 * The logic below:
 *   1. Timestamps the most recent SOF (called from HAL_PCD_SOFCallback).
 *   2. Counts consecutive TX failures observed by the class TX paths.
 *   3. From the main loop, decides whether the bus is dead while we still
 *      think we are CONFIGURED and, if so, does a full PHY+stack rebuild.
 *
 * The recovery routine intentionally does NOT call Error_Handler() on any
 * intermediate failure - the existing MX_USB_DEVICE_Init() will spin in
 * Error_Handler on failure, so we re-implement the bring-up here with
 * best-effort behavior so a single bad attempt cannot brick the device.
 *==========================================================================*/

#define USB_SOF_TIMEOUT_MS        500u   /* No SOF for this long while CONFIGURED -> bus is dead   */
#define USB_TX_FAIL_THRESHOLD     10u    /* Consecutive class TX failures that trip a recovery     */
#define USB_RECOVER_BACKOFF_MS    2000u  /* Minimum spacing between recovery attempts              */

static volatile uint32_t s_usb_last_sof_ms       = 0;
static volatile uint32_t s_usb_tx_failure_count  = 0;
static volatile uint8_t  s_usb_sof_seen          = 0;   /* set to 1 by first SOF after (re)init  */
static uint32_t          s_usb_last_recover_ms   = 0;
static uint32_t          s_usb_recover_count     = 0;

/* Provided by uart_comms.c so we can re-arm the host-side comms state machine
 * after the USB stack has been rebuilt. */
extern void comms_host_start(void);

void USB_NotifySof(void)
{
    s_usb_last_sof_ms = HAL_GetTick();
    s_usb_sof_seen    = 1;
}

void USB_NotifyTxFailure(void)
{
    /* Saturating increment - we only care that it crosses the threshold. */
    if (s_usb_tx_failure_count < 0xFFFFFFFFu)
    {
        s_usb_tx_failure_count++;
    }
}

void USB_NotifyTxSuccess(void)
{
    s_usb_tx_failure_count = 0;
}

uint32_t USB_GetRecoverCount(void)
{
    return s_usb_recover_count;
}

/* Best-effort tear-down + rebuild of the USB device stack. Safe to call from
 * the main loop. Must NOT be called from an ISR. */
void USB_ForceRecover(void)
{
    uint32_t now = HAL_GetTick();

    /* Throttle: don't hammer recovery if something is wrong at a deeper level. */
    if ((s_usb_last_recover_ms != 0u) &&
        ((now - s_usb_last_recover_ms) < USB_RECOVER_BACKOFF_MS))
    {
        return;
    }
    s_usb_last_recover_ms = now;
    s_usb_recover_count++;

    printf("[USB] Recovery #%lu: dev_state=0x%02X tx_fail=%lu - rebuilding stack\r\n",
           (unsigned long)s_usb_recover_count,
           (unsigned)hUsbDeviceHS.dev_state,
           (unsigned long)s_usb_tx_failure_count);

    /* 1. Best-effort tear-down. Ignore return codes: the whole point is that
     *    the stack may already be in a bad state. */
    (void)USBD_Stop(&hUsbDeviceHS);
    (void)USBD_DeInit(&hUsbDeviceHS);

    /* 2. Quiesce and clear any pending OTG interrupts left over from EFT. */
    HAL_NVIC_DisableIRQ(OTG_HS_IRQn);
    HAL_NVIC_DisableIRQ(OTG_HS_EP1_IN_IRQn);
    HAL_NVIC_DisableIRQ(OTG_HS_EP1_OUT_IRQn);
    __NVIC_ClearPendingIRQ(OTG_HS_IRQn);
    __NVIC_ClearPendingIRQ(OTG_HS_EP1_IN_IRQn);
    __NVIC_ClearPendingIRQ(OTG_HS_EP1_OUT_IRQn);

    /* 3. Hardware-reset the OTG_HS core and ULPI PHY by power-cycling their
     *    peripheral clocks. This is the bit that recovers a wedged ULPI link
     *    that USBD_DeInit() alone cannot reach. */
    __HAL_RCC_USB_OTG_HS_CLK_DISABLE();
    __HAL_RCC_USB_OTG_HS_ULPI_CLK_DISABLE();
    HAL_Delay(20);
    __HAL_RCC_USB_OTG_HS_CLK_ENABLE();
    __HAL_RCC_USB_OTG_HS_ULPI_CLK_ENABLE();

    /* 4. Reset state tracking BEFORE bring-up so SOFs from the new session
     *    immediately update the timestamp. */
    s_usb_last_sof_ms      = HAL_GetTick();
    s_usb_tx_failure_count = 0;
    s_usb_sof_seen         = 0;   /* must observe a fresh SOF before re-arming */

    /* 5. Re-bring-up the stack. If any step fails, give up for this attempt;
     *    USB_RecoveryCheck() will trigger us again after the backoff. */
    if (USBD_Init(&hUsbDeviceHS, &HS_Desc, DEVICE_HS) != USBD_OK)
    {
        printf("[USB] Recovery: USBD_Init failed\r\n");
        return;
    }

    COMMS_InstID = hUsbDeviceHS.classId;
    if (USBD_RegisterClassComposite(&hUsbDeviceHS, USBD_COMMS_CLASS,
                                    CLASS_TYPE_COMMS, COMMS_EpAdd_Inst) != USBD_OK)
    {
        printf("[USB] Recovery: register COMMS failed\r\n");
        (void)USBD_DeInit(&hUsbDeviceHS);
        return;
    }

    HISTO_InstID = hUsbDeviceHS.classId;
    if (USBD_RegisterClassComposite(&hUsbDeviceHS, USBD_HISTO_CLASS,
                                    CLASS_TYPE_HISTO, HISTO_EpAdd_Inst) != USBD_OK)
    {
        printf("[USB] Recovery: register HISTO failed\r\n");
        (void)USBD_DeInit(&hUsbDeviceHS);
        return;
    }

    IMU_InstID = hUsbDeviceHS.classId;
    if (USBD_RegisterClassComposite(&hUsbDeviceHS, USBD_IMU_CLASS,
                                    CLASS_TYPE_IMU, IMU_EpAdd_Inst) != USBD_OK)
    {
        printf("[USB] Recovery: register IMU failed\r\n");
        (void)USBD_DeInit(&hUsbDeviceHS);
        return;
    }

    if (USBD_Start(&hUsbDeviceHS) != USBD_OK)
    {
        printf("[USB] Recovery: USBD_Start failed\r\n");
        (void)USBD_DeInit(&hUsbDeviceHS);
        return;
    }

    HAL_PWREx_EnableUSBVoltageDetector();

    /* 6. Re-arm application-level comms (clears stuck rx_flag/tx_flag, queues, etc.). */
    comms_host_start();

    printf("[USB] Recovery #%lu complete\r\n", (unsigned long)s_usb_recover_count);
}

/* Periodic health check. Call from the main loop (no ISR context). */
void USB_RecoveryCheck(void)
{
    uint32_t now = HAL_GetTick();

    /* If we are not configured, the bus may simply be unplugged or still
     * enumerating. Keep the SOF timestamp current so we don't trip the moment
     * we transition to CONFIGURED. */
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED)
    {
        s_usb_last_sof_ms = now;
        s_usb_tx_failure_count = 0;
        return;
    }

    /* Don't arm the SOF watchdog until we've actually observed at least one
     * SOF since the last (re)init. This prevents a boot-time false trip if
     * Sof_enable somehow ends up disabled, and also gives the host a moment
     * after enumeration before we start counting. */
    if (!s_usb_sof_seen)
    {
        s_usb_last_sof_ms = now;
    }
    else
    {
        /* Snapshot the timestamp atomically: the SOF ISR fires every 125 us
         * at HS and writes s_usb_last_sof_ms, so a torn read here would race
         * the ISR and produce nonsense (e.g. last > now -> unsigned wrap). */
        uint32_t primask  = __get_PRIMASK();
        __disable_irq();
        uint32_t last_sof = s_usb_last_sof_ms;
        __set_PRIMASK(primask);

        /* Treat last > now (ISR ran between reading `now` and `last_sof`) as
         * zero elapsed time - we don't want a 1 ms ISR/main-loop race to
         * trigger a recovery. */
        uint32_t elapsed = (now >= last_sof) ? (now - last_sof) : 0u;

        if (elapsed > USB_SOF_TIMEOUT_MS)
        {
            /* SOF starvation while configured -> OTG core / PHY is wedged. */
            printf("[USB] No SOF for %lu ms while CONFIGURED - forcing recovery\r\n",
                   (unsigned long)elapsed);
            USB_ForceRecover();
            return;
        }
    }

    /* Persistent class-level TX failures -> endpoint stuck. */
    if (s_usb_tx_failure_count >= USB_TX_FAIL_THRESHOLD)
    {
        printf("[USB] %lu consecutive TX failures - forcing recovery\r\n",
               (unsigned long)s_usb_tx_failure_count);
        USB_ForceRecover();
        return;
    }
}

/* USER CODE END 2 */

