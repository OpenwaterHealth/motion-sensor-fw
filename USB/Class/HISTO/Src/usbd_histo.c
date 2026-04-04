/**
  ******************************************************************************
  * @file    usbd_histo.c
  * @brief   Histogram data streaming implementation
  ******************************************************************************
  */
#include "usbd_histo.h"
#include "usbd_ctlreq.h"
#include "usbd_desc.h"
#include "common.h"
#include "logging.h"
#include "utils.h"

#define HISTO_THROTTLE_INTERVAL_MS 5000u

/* Private typedef */
typedef struct {
  uint8_t *buffer;
  uint16_t length;
} histo_queue_entry_t;

#define HISTO_QUEUE_SIZE 6  /* 6 × 32837 B = ~192 KB in D2 SRAM, well within the 288 KB budget.
                             * The extra 2 slots give the host an additional ~50 ms of headroom
                             * (2 frames × 25 ms) before the queue backs up, absorbing brief GIL
                             * stalls in the Python USB-read threads on the host. */

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif

/* Private variables */
static histo_queue_entry_t histo_queue[HISTO_QUEUE_SIZE];
static uint8_t histo_queue_head = 0;
static uint8_t histo_queue_tail = 0;
static uint8_t histo_queue_count = 0;
static USBD_HandleTypeDef *histo_pdev = NULL;

/* Private function prototypes */
static uint8_t histo_queue_enqueue(uint8_t *data, uint16_t length);
static uint8_t histo_queue_dequeue(uint8_t **data, uint16_t *length);
static uint8_t histo_queue_is_empty(void);
static uint8_t histo_queue_is_full(void);
static uint8_t histo_process_queue(void);
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
uint8_t* pTxHistoBuff = 0;
static uint16_t tx_histo_total_len = 0;
static uint16_t tx_histo_ptr = 0;
static __IO uint8_t histo_ep_enabled = 0;
__IO uint8_t histo_ep_data = 0;
static uint8_t HISTOInEpAdd = HISTO_IN_EP;
volatile uint32_t histo_enq_count = 0;
volatile uint32_t histo_deq_count = 0;
volatile uint32_t histo_datain_count = 0;
volatile uint32_t histo_tx_fail_count = 0;

#ifndef USB_RAM_D2
#define USB_RAM_D2 __attribute__((section(".ram_d2")))
#endif

USB_RAM_D2 __ALIGN_BEGIN static uint8_t histo_tx_buffer[USB_HISTO_MAX_SIZE] __ALIGN_END;
USB_RAM_D2 __ALIGN_BEGIN static uint8_t histo_queue_buffers[HISTO_QUEUE_SIZE][USB_HISTO_MAX_SIZE] __ALIGN_END;

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
    pTxHistoBuff = histo_tx_buffer;

    /* Allocate buffers for queue entries */
    for (uint8_t i = 0; i < HISTO_QUEUE_SIZE; i++) {
      histo_queue[i].buffer = histo_queue_buffers[i];
      histo_queue[i].length = 0;
    }

    /* Initialize queue */
    histo_queue_head = 0;
    histo_queue_tail = 0;
    histo_queue_count = 0;
    histo_pdev = pdev;

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
    pTxHistoBuff = 0;
  }

  /* Free queue entry buffers */
  for (uint8_t i = 0; i < HISTO_QUEUE_SIZE; i++) {
    histo_queue[i].buffer = NULL;
  }

  histo_queue_head = 0;
  histo_queue_tail = 0;
  histo_queue_count = 0;
  histo_pdev = NULL;
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

/* Queue management functions */
static uint8_t histo_queue_is_empty(void)
{
  return (histo_queue_count == 0);
}

static uint8_t histo_queue_is_full(void)
{
  return (histo_queue_count >= HISTO_QUEUE_SIZE);
}

static volatile uint32_t histo_enq_fail_count = 0;  /* monotonic count of queue-full drops */

static uint8_t histo_queue_enqueue(uint8_t *data, uint16_t length)
{
  if (histo_queue_is_full()) {
    /* NEVER printf here — this runs inside the FSIN ISR at priority 0.
     * printf -> logging_uart_dma_write can spin-wait up to 10 ms when the
     * UART TX ring buffer is full, because the DMA-complete ISR (also
     * priority 0) cannot preempt us.  That 10 ms blocks ALL SPI/USART
     * interrupts, causing camera histogram overruns and the cascade
     * failure.  Instead, just bump a counter; the scan-finished summary
     * will report the total. */
    histo_enq_fail_count++;
    return USBD_FAIL;
  }

  if (length > USB_HISTO_MAX_SIZE) {
    return USBD_FAIL;
  }

  /* Copy data into queue entry buffer */
  memcpy(histo_queue[histo_queue_tail].buffer, data, length);
  histo_queue[histo_queue_tail].length = length;

  /* Update queue pointers */
  histo_queue_tail = (histo_queue_tail + 1) % HISTO_QUEUE_SIZE;
  histo_queue_count++;
  histo_enq_count++;

  return USBD_OK;
}

static uint8_t histo_queue_dequeue(uint8_t **data, uint16_t *length)
{
  if (histo_queue_is_empty()) {
    return USBD_FAIL;
  }

  *data = histo_queue[histo_queue_head].buffer;
  *length = histo_queue[histo_queue_head].length;

  /* Update queue pointers */
  histo_queue_head = (histo_queue_head + 1) % HISTO_QUEUE_SIZE;
  histo_queue_count--;
  histo_deq_count++;

  return USBD_OK;
}

static uint8_t histo_process_queue(void)
{
  uint8_t *data;
  uint16_t length;

  if (histo_pdev == NULL) {
    return USBD_FAIL;
  }

  if (histo_queue_is_empty()) {
    return USBD_OK;
  }

  /* If still sending, don't process queue yet */
  if (histo_ep_data != 0) {
    return USBD_BUSY;
  }

  /* Dequeue next packet */
  if (histo_queue_dequeue(&data, &length) != USBD_OK) {
    return USBD_FAIL;
  }

  /* Send the dequeued packet */
  return USBD_HISTO_SetTxBuffer(histo_pdev, data, length);
}

static uint8_t USBD_Histo_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  // Ignore everything, don't stall
  return (uint8_t)USBD_OK;
}

static uint8_t USBD_Histo_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	uint8_t ret = USBD_OK;
  histo_datain_count++;

#ifdef USE_USBD_COMPOSITE
	  /* Get the Endpoints addresses allocated for this CDC class instance */
	HISTOInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, HISTO_InstID);
#endif /* USE_USBD_COMPOSITE */

  if(histo_ep_data==1){
      uint16_t max_pkt = (pdev->dev_speed == USBD_SPEED_HIGH)
                          ? HISTO_HS_MAX_PACKET_SIZE
                          : HISTO_FS_MAX_PACKET_SIZE;
      tx_histo_ptr += max_pkt;

      if (tx_histo_ptr < tx_histo_total_len)
      {
          uint16_t remaining = tx_histo_total_len - tx_histo_ptr;
          uint16_t pkt_len = MIN(max_pkt, remaining);

          ret =  USBD_LL_Transmit(pdev, HISTOInEpAdd, &pTxHistoBuff[tx_histo_ptr], pkt_len);
          if (ret != USBD_OK) {
            histo_tx_fail_count++;
            histo_ep_data = 0;
          }
      }
      else if ((tx_histo_total_len % max_pkt) == 0)
      {
          /* Transfer complete but the last USB packet was exactly
           * max_pkt bytes — the host cannot distinguish this from a
           * mid-transfer chunk.  Send a Zero-Length Packet (ZLP) so
           * the host's dev.read() returns immediately instead of
           * blocking until the 100 ms timeout.  Without this, every
           * compressed histogram whose byte count is a multiple of 512
           * causes a 100 ms stall + data loss on the host. */
          histo_ep_data = 2;  /* state 2 = ZLP in flight */
          ret = USBD_LL_Transmit(pdev, HISTOInEpAdd, NULL, 0U);
          if (ret != USBD_OK) {
            histo_tx_fail_count++;
            histo_ep_data = 0;
          }
      }
      else
      {
          /* Last USB packet was a short packet — host already knows
           * the transfer is done.  Complete normally. */
          histo_ep_data = 0;
          USBD_HISTO_TxCpltCallback(pTxHistoBuff, tx_histo_total_len, HISTOInEpAdd);

		  /* Process queue to send next packet if available */
		  histo_process_queue();
      }
  } else if (histo_ep_data == 2) {
      /* ZLP sent and ACK'd — transfer truly complete. */
      histo_ep_data = 0;
      USBD_HISTO_TxCpltCallback(pTxHistoBuff, tx_histo_total_len, HISTOInEpAdd);
      histo_process_queue();
  } else {
    /* ep_data == 0: no active transfer.
     * Attempt to send the next queued packet.  If the queue is empty,
     * return without arming anything so the endpoint NAKs IN tokens
     * until new data arrives via USBD_HISTO_SendData.
     *
     * Previously this branch unconditionally re-armed a ZLP, which
     * created an unbounded ZLP storm (hundreds of DataIn ISR callbacks
     * per second) while the endpoint was idle.  The storm wasted ISR
     * cycles, inflated the datain counter, and made it harder to observe
     * real transfer activity in logs and diagnostics. */
    pdev->ep_in[HISTOInEpAdd & 0xFU].total_length = 0U;
    ret = histo_process_queue();
  }

  return ret;
}

/* Last timestamp when a histogram packet was actually sent (for DEBUG_FLAG_HISTO_THROTTLE) */
static uint32_t histo_last_send_ms = 0;

uint8_t USBD_HISTO_SendData(USBD_HandleTypeDef *pdev, uint8_t *data, uint16_t len, uint8_t ep_idx)
{
  UNUSED(ep_idx);
  
  if (pdev == NULL || data == NULL || len == 0 || len > USB_HISTO_MAX_SIZE) {
    return USBD_FAIL;
  }

  /* Debug flag: only send histogram packet every 5 seconds; others pretend success */
  if ((logging_get_debug_flags() & DEBUG_FLAG_HISTO_THROTTLE) != 0u) {
    uint32_t now_ms = get_timestamp_ms();
    uint32_t elapsed = (histo_last_send_ms != 0u) ? (now_ms - histo_last_send_ms) : HISTO_THROTTLE_INTERVAL_MS;
    if (elapsed < HISTO_THROTTLE_INTERVAL_MS) {
      return USBD_OK;  /* Pretend sent, do not enqueue or transmit */
    }
    histo_last_send_ms = now_ms;
  }

  /* Ensure pdev is stored (in case called before Init, though this shouldn't happen) */
  if (histo_pdev == NULL) {
    histo_pdev = pdev;
  }

  /* If queue is empty and not currently sending, send directly */
  if (histo_queue_is_empty() && histo_ep_data == 0) {
    return USBD_HISTO_SetTxBuffer(pdev, data, len);
  }
  
  if (histo_ep_data == 0 && !histo_queue_is_empty()) {
    (void)histo_process_queue();
  }

  /* Otherwise, add to queue */
  return histo_queue_enqueue(data, len);
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
		/* Copy only the live payload — no need to zero the rest of the
		 * 32 KB buffer first.  The DMA transfer length is controlled by
		 * tx_histo_total_len (set below), so bytes beyond `length` are
		 * never read by the USB engine.  Removing the full-buffer memset
		 * saves ~70 µs of ISR time per packet (benchmarked at 48 MHz AHB
		 * with the D2 SRAM bus at ~133 MHz). */
		memcpy(pTxHistoBuff,pbuff,length);

        tx_histo_total_len = length;
        tx_histo_ptr = 0;

        uint16_t pkt_len = MIN((pdev->dev_speed == USBD_SPEED_HIGH)?HISTO_HS_MAX_PACKET_SIZE:HISTO_FS_MAX_PACKET_SIZE, tx_histo_total_len);

		pdev->ep_in[HISTOInEpAdd & 0xFU].total_length = tx_histo_total_len;

		ret = USBD_LL_Transmit(pdev, HISTOInEpAdd, pTxHistoBuff, pkt_len);
		if (ret == USBD_OK) {
			histo_ep_data = 1;
		} else {
			histo_tx_fail_count++;
			histo_ep_data = 0;
		}
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
