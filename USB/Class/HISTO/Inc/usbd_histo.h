/**
  ******************************************************************************
  * @file    usbd_histo.h
  * @brief   Header file for USB Histogram Data Streaming
  ******************************************************************************
  */
#ifndef __USB_HISTO_H
#define __USB_HISTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_ioreq.h"


#ifndef HISTO_IN_EP
#define HISTO_IN_EP                                   0x81U  /* EP1 for data IN */
#endif /* HISTO_IN_EP */

#define HISTO_FS_MAX_PACKET_SIZE         64U    /* Full-speed USB */
#define HISTO_HS_MAX_PACKET_SIZE         512U   /* High-speed USB */
#define USB_HISTO_MAX_SIZE 				 5120
extern USBD_ClassTypeDef USBD_HISTO;
#define USBD_HISTO_CLASS &USBD_HISTO

uint8_t  USBD_HISTO_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t  *pbuff, uint16_t length);
void USBD_HISTO_TxCpltCallback(uint8_t *Buf, uint32_t Len, uint8_t epnum);

#ifdef __cplusplus
}
#endif

#endif /* __USB_HISTO_H */
