/**
  ******************************************************************************
  * @file    usbd_histo.c
  * @brief   Histogram data streaming implementation
  ******************************************************************************
  */
#include "usbd_histo.h"
#include "usbd_ctlreq.h"
#include "usbd_desc.h"

/* Private typedef */

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif

/* Private variables */
static uint8_t USBD_Histo_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_Histo_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_Histo_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_Histo_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
#ifndef USE_USBD_COMPOSITE
static uint8_t *USBD_Histo_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_Histo_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_Histo_GetOtherSpeedCfgDesc(uint16_t *length);
uint8_t *USBD_Histo_GetDeviceQualifierDescriptor(uint16_t *length);
#endif /* USE_USBD_COMPOSITE  */

USBD_ClassTypeDef USBD_HISTO = {
  USBD_Histo_Init,
  USBD_Histo_DeInit,
  USBD_Histo_Setup,
  NULL,                 /* EP0_TxSent */
  NULL,                 /* EP0_RxReady */
  USBD_Histo_DataIn,    /* DataIn */
  NULL,                 /* DataOut */
  NULL,                 /* SOF */
  NULL,
  NULL,
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
  USBD_Histo_GetHSCfgDesc,
  USBD_Histo_GetFSCfgDesc,
  USBD_Histo_GetOtherSpeedCfgDesc,
  USBD_Histo_GetDeviceQualifierDescriptor,
#endif /* USE_USBD_COMPOSITE  */
};

#ifndef USE_USBD_COMPOSITE
/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_Histo_GetDeviceQualifierDescriptor[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};
#endif /* USE_USBD_COMPOSITE  */

extern uint8_t HISTO_InstID;
uint8_t* pTxHistoBuff = 0; // large buffer, should be one of the double buffers
static uint16_t tx_histo_total_len = 0;
static uint16_t tx_histo_ptr = 0;
static __IO uint8_t histo_ep_enabled = 0;
__IO uint8_t histo_ep_data = 0;
static uint8_t HISTOInEpAdd = HISTO_IN_EP;

/* Private functions */
static uint8_t USBD_Histo_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  uint8_t ret = USBD_OK;
  UNUSED(cfgidx);

  #ifdef USE_USBD_COMPOSITE
    /* Get the Endpoints addresses allocated for this class instance */
    HISTOInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
  #endif /* USE_USBD_COMPOSITE */
    printf("HISTO_Init DATA IN EP: 0x%02X ClassID: 0x%02X\r\n", HISTOInEpAdd, (uint8_t)pdev->classId);
    pTxHistoBuff = (uint8_t*)malloc(HISTO_USB_FIFO_MAX_SIZE);
    if(pTxHistoBuff == NULL){
    	Error_Handler();
    }

    if (pdev->dev_speed == USBD_SPEED_HIGH)
    {
      /* Open EP IN */
      (void)USBD_LL_OpenEP(pdev, HISTOInEpAdd, USBD_EP_TYPE_BULK, HISTO_HS_MAX_PACKET_SIZE);

    }
    else
    {
    /* Open EP IN */
    (void)USBD_LL_OpenEP(pdev, HISTOInEpAdd, USBD_EP_TYPE_BULK, HISTO_FS_MAX_PACKET_SIZE);
    }
    histo_ep_enabled = 1;
    pdev->ep_in[HISTOInEpAdd & 0xFU].bInterval = 0;
    pdev->ep_in[HISTOInEpAdd & 0xFU].is_used = 1U;

	/* Send ZLP */
	ret = USBD_LL_Transmit (pdev, HISTOInEpAdd, NULL, 0U);

    return ret;
}

extern uint8_t HISTO_InstID;

static uint8_t USBD_Histo_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this CDC class instance */
  HISTOInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, (uint8_t)HISTO_InstID);
#endif /* USE_USBD_COMPOSITE */

  /* Close EP IN */
  (void)USBD_LL_CloseEP(pdev, HISTOInEpAdd);
  pdev->ep_in[HISTOInEpAdd & 0xFU].is_used = 0U;
  pdev->ep_in[HISTOInEpAdd & 0xFU].total_length = 0U;
  histo_ep_enabled = 0;

  if(pTxHistoBuff){
	free(pTxHistoBuff);
	pTxHistoBuff = 0;
  }
#ifdef USE_USBD_COMPOSITE
  if (pdev->pClassDataCmsit[pdev->classId] != NULL)
  {
    // ((USBD_HISTO_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit();
    (void)USBD_free(pdev->pClassDataCmsit[pdev->classId]);
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    pdev->pClassData = NULL;
  }

#else
  /* Free memory */
  if (pdev->pClassData != NULL) {
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }
#endif

  return (uint8_t)USBD_OK;
}

static uint8_t USBD_Histo_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  // Ignore everything, don't stall
  return (uint8_t)USBD_OK;
}

static uint8_t USBD_Histo_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	uint8_t ret = USBD_OK;

#ifdef USE_USBD_COMPOSITE
	  /* Get the Endpoints addresses allocated for this CDC class instance */
	HISTOInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, HISTO_InstID);
#endif /* USE_USBD_COMPOSITE */

  if(histo_ep_data==1){
      tx_histo_ptr += (pdev->dev_speed == USBD_SPEED_HIGH)?HISTO_HS_MAX_PACKET_SIZE:HISTO_FS_MAX_PACKET_SIZE;

      if (tx_histo_ptr < tx_histo_total_len)
      {
          uint16_t remaining = tx_histo_total_len - tx_histo_ptr;
          uint16_t pkt_len = MIN((pdev->dev_speed == USBD_SPEED_HIGH)?HISTO_HS_MAX_PACKET_SIZE:HISTO_FS_MAX_PACKET_SIZE, remaining);
          // printf("r: %d\r\n",  remaining / HISTO_HS_MAX_PACKET_SIZE);

          ret =  USBD_LL_Transmit(pdev, HISTOInEpAdd, &pTxHistoBuff[tx_histo_ptr], pkt_len);
      }
      else
      {
          // Transfer complete
          histo_ep_data = 0;
          USBD_HISTO_TxCpltCallback(pTxHistoBuff, tx_histo_total_len, HISTOInEpAdd);
		  /* Send ZLP */
		  // ret = USBD_LL_Transmit (pdev, HISTOInEpAdd, NULL, 0U);
      }
  } else {
	  /* Send ZLP */
    pdev->ep_in[HISTOInEpAdd & 0xFU].total_length = 0U;
	  ret = USBD_LL_Transmit (pdev, HISTOInEpAdd, NULL, 0U);
  }

  return ret;
}

uint8_t USBD_HISTO_SendData(USBD_HandleTypeDef *pdev, uint8_t *data, uint16_t len, uint8_t ep_idx)
{
  return (uint8_t)USBD_OK;
}

uint8_t  USBD_HISTO_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t  *pbuff, uint16_t length)
{
	uint8_t ret = USBD_OK;

	if(histo_ep_enabled == 1 && histo_ep_data==0)
	{
#ifdef USE_USBD_COMPOSITE
		/* Get the Endpoints addresses allocated for this CDC class instance */
		HISTOInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, HISTO_InstID);
#endif /* USE_USBD_COMPOSITE */
		USBD_LL_FlushEP(pdev, HISTOInEpAdd);
    // printf("t: %d\r\n", length);
    pTxHistoBuff = pbuff;

    tx_histo_total_len = length;
    tx_histo_ptr = 0;

    uint16_t pkt_len = MIN((pdev->dev_speed == USBD_SPEED_HIGH)?HISTO_HS_MAX_PACKET_SIZE:HISTO_FS_MAX_PACKET_SIZE, tx_histo_total_len);

		pdev->ep_in[HISTOInEpAdd & 0xFU].total_length = tx_histo_total_len;
		histo_ep_data = 1;

		ret = USBD_LL_Transmit(pdev, HISTOInEpAdd, pTxHistoBuff, pkt_len);
	}
	else
	{
		ret = USBD_BUSY;
	}
  return ret;
}

uint8_t USBD_HISTO_RegisterInterface(USBD_HandleTypeDef *pdev, uint8_t *buffer)
{
  UNUSED(pdev);
  UNUSED(buffer);
  return (uint8_t)USBD_OK;
}

__weak void USBD_HISTO_TxCpltCallback(uint8_t *Buf, uint32_t Len, uint8_t epnum)
{
	UNUSED(Buf);
	UNUSED(Len);
	UNUSED(epnum);
}
